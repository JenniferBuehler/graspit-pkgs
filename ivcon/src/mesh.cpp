/*****************************************************************************/
/*                                                                           */
/*  Program for Mesh Generation                                              */
/*                                                                           */
/*  Author:         Dragos A. Harabor                                        */
/*  Last Modified:  May 19, 1997                                             */
/*                                                                           */
/*****************************************************************************/

#include <fstream>
#include <iostream>
#include <string>
#include <math.h>

/*****************************************************************************/
const double DIST = 0.5;	/* maximum distance between 2 points */
const int MIN_NEIGHBORS = 4;	/* minimum number of valid neighbors */
const double MAIN_POINT_WEIGHT = 0.3;
				/* weight for computing average of a point */ 
#ifndef __linux__
const double INFINITY = 1e308; 
#else
#include <string.h>
#include <stdlib.h>
#endif
const long MAX_TRIANGLES = 80000;

/*****************************************************************************/

using namespace std;

ifstream range_file;	/* file streams */
ifstream params_file;
ofstream out_file;


double *X, *Y, *Z;	/* data matrix, Nx3, N=ROWS*COLS, 3D points */
int ROWS, COLS;		/* number of rows and columns */
int min_yc;		/* value subtracted from yc coordinate */
double cc_x, cc_y, fc_x, fc_y, kc;
			/* parameters from .params file */

unsigned long num_valid_points = 0;
			/* number of valid points, i.e. not INFINITY */
unsigned long num_pts_tr = 0;
			/* number of points that form triangles */
unsigned long num_tr = 0;	
			/* number of triangles */

unsigned long tr[MAX_TRIANGLES][3];
			/* vertices of the triangles, index in real_index*/
long *i_to_c, *c_to_i;	/* look-up table for indexes */

bool VRMLformat = false;/* VRML or Inventor format */ 


/*****************************************************************************/
/*****************************************************************************/

void xc_yc(double X, double Y, double Z, double& xc, double& yc)
{
  // pinhole projection
  xc = X/Z; yc = Y/Z;
  
  // radial distortion
  double norm2 = xc*xc + yc*yc;
  xc *= (1 + kc*norm2);
  yc *= (1 + kc*norm2);

  // pixel coordinates
  xc = fc_x*xc + cc_x;
  yc = fc_y*yc + cc_y;
}


/*****************************************************************************/
/*****************************************************************************/

void open_files(char *file_name)
{
    char of_name[500];
    char rangef[500];
    char paramsf[500];
    
    // name of output file
    strcpy(of_name, file_name);
    if (VRMLformat)
	strcat(of_name, ".wrl");
    else
	strcat(of_name, ".iv");

    // name of range file
    strcpy(rangef, file_name);
    strcat(rangef, ".range");

    // name of params file
    strcpy(paramsf, file_name);
    strcat(paramsf, ".params");

    range_file.open(rangef, ios::in);
    params_file.open(paramsf, ios::in);
    out_file.open(of_name, ios::out);

    if ( (!range_file) || (!params_file) || (!out_file)) {
	cout << endl << "Error opening one of the files!" << endl << endl;
	exit(1);
    }
}
	
/*****************************************************************************/

void close_files()
{
    range_file.close();
    params_file.close();
    out_file.close();
}

/*****************************************************************************/

void show_params()
{
  cerr << "kcam = " << kc << endl;
  cerr << "fxcam = " << fc_x << endl;
  cerr << "fycam = " << fc_y << endl;
  cerr << "cxcam = " << cc_x << endl;
  cerr << "cycam = " << cc_y << endl;
  
}

/*****************************************************************************/

void read_params_file()
{
  // some string
	std::string str;

  while (!params_file.eof()) {
    // read a new token from the file
    params_file >> str;
    
    if (str=="kcam")  params_file >> kc;
    if (str=="fxcam") params_file >> fc_x;
    if (str=="fycam") params_file >> fc_y;
    if (str=="cxcam") params_file >> cc_x;
    if (str=="cycam") params_file >> cc_y;
    
  }

  // show_params();
  // exit(1);
  
}

/*****************************************************************************/

void write_file(char *rgb_file)
{
    char rgb_f[500];
    strcpy(rgb_f, rgb_file);
    strcat(rgb_f, ".rgb");

    cout << "\nWriting output file ..." << endl;

    if (VRMLformat)
	out_file << "#VRML V1.0 ascii" << endl;
    else
	out_file << "#Inventor V2.1 ascii" << endl;

    out_file << "Separator {" << endl;
    out_file << "\tTransform { rotation 1 0 0 3.1415 }" << endl;


    // write Texture2 node
    out_file << "\tTexture2 {" << endl;
    out_file << "\t\tfilename \"" << rgb_f << "\"\n"; 
    out_file << "\t\tmodel DECAL" << endl;
    out_file << "\t}" << endl;

    // write TextureCoordinateBiding node
    out_file << "\tTextureCoordinateBinding {" << endl;
    out_file << "\t\tvalue PER_VERTEX_INDEXED" << endl;
    out_file << "\t}\n";

    // write TextureCoordinate2 node
    out_file << "\tTextureCoordinate2 {" << endl;
    out_file << "\t\tpoint [" << endl;
    
    for (int i=0; i<num_pts_tr; ++i) {
      double xc, yc;
      // compute xc, yc coordinates in the texture image
      xc_yc(X[i_to_c[i]], Y[i_to_c[i]], Z[i_to_c[i]], xc, yc);

      out_file << (xc/639) << ' ' << (1-yc/479);
      if (i!=num_pts_tr-1)
	out_file << ',';
      out_file << endl;
    }
    out_file << "\t\t]" << endl << "\t}" << endl;


    // write Coordinate3 node
    out_file << "\tCoordinate3 {" << endl;
    out_file << "\t\tpoint [" << endl;
    // write all points
    for (int i=0; i<num_pts_tr; ++i) {
      out_file << X[i_to_c[i]] << ' ' << Y[i_to_c[i]] << ' ' << Z[i_to_c[i]] ;
      if (i!=num_pts_tr-1)
	out_file << ',';
      out_file << endl;
    }
    out_file << "\t\t]" << endl << "\t}" << endl;
    
    // write IndexedFaceSet node
    out_file << "\tIndexedFaceSet {" << endl;
    out_file << "\t\tcoordIndex [" << endl;
    for (int i=0; i<num_tr; ++i) {
	out_file << tr[i][0] << ", " << tr[i][1] << ", " << tr[i][2] << ", -1";
	if (i!=num_tr-1)
	    out_file << ',';
	out_file << endl;
    }
    out_file << "\t\t]" << endl << "\t}" << endl;

    // final closing bracket
    out_file << '}' << endl;
}


/*****************************************************************************/
/*****************************************************************************/

double dist(long p1, long p2)
{
    // compute distance between 2 points
    return sqrt( (X[p1]-X[p2])*(X[p1]-X[p2])+(Y[p1]-Y[p2])*(Y[p1]-Y[p2])+(Z[p1]-Z[p2])*(Z[p1]-Z[p2]) );
}

/*****************************************************************************/
bool close_points(long p1, long p2, long p3)
{
    // check if the points are close to each other
    return (Z[p1]>-INFINITY) && (Z[p2]>-INFINITY) && (Z[p3]>-INFINITY) && (dist(p1, p2)<=DIST) && (dist(p2,p3)<=DIST) && (dist(p3,p1)<=DIST);
}

/*****************************************************************************/
long get_index(unsigned long p)
{
    if (c_to_i[p] == -1) {
	// get a new index for this point
	c_to_i[p] = num_pts_tr;
	i_to_c[num_pts_tr] = p;
	num_pts_tr++;
    }

    // return index of this point
    return c_to_i[p];
}

/*****************************************************************************/

void create_mesh()
{
    unsigned long fixed_holes = 0;
    
    unsigned long p, np[8];
    int cnt = 0;
    double sx, sy, sz;

    // PASS 1: fill in the holes
    for (int i=1; i<ROWS-1; ++i)
	for (int j=1; j<COLS-1; ++j) {
	    // index of current point
	    p = i*COLS + j;
	    // get the 8 "neighbors" of this current point
	    np[0] = (i-1)*COLS + (j-1);
	    np[1] = np[0] + 1;
	    np[2] = np[1] + 1;
	    np[3] = p + 1;
	    np[4] = (i+1)*COLS + (j+1);
	    np[5] = np[4] - 1;
	    np[6] = np[5] - 1;
	    np[7] = p - 1;

	    // count the number of valid neighbors far from this point
	    cnt=0; sx=0; sy=0; sz=0;
	    for (int k=0; k<8; ++k)
		if ((Z[np[k]]>-INFINITY) && (dist(p,np[k])>DIST)) {
		    // increment number of valid points
		    cnt++;
		    // compute sum of coordinates
		    sx+=X[np[k]];
		    sy+=Y[np[k]];
		    sz+=Z[np[k]];
		}
	    if (cnt>=MIN_NEIGHBORS) {
	      fixed_holes++;
	      
	      // fix the whole, by taking the average of neighboring points
	      X[p] = sx/cnt;
	      Y[p] = sy/cnt;
	      Z[p] = sz/cnt;
	    }

	    // low-pass filter
	    if (Z[p]>-INFINITY) {
		cnt=0; sx=0; sy=0; sz=0;
		for (int k=0; k<8; ++k)
		    if (Z[np[k]]>-INFINITY && (dist(p,np[k])<=DIST)) {
			// increment number of valid points
			cnt++;
			// compute sum of coordinates
			sx+=X[np[k]];
			sy+=Y[np[k]];
			sz+=Z[np[k]];
		    }
  	   
		X[p] = MAIN_POINT_WEIGHT * X[p] + (1-MAIN_POINT_WEIGHT)*sx/cnt;
		Y[p] = MAIN_POINT_WEIGHT * Y[p] + (1-MAIN_POINT_WEIGHT)*sy/cnt;
		Z[p] = MAIN_POINT_WEIGHT * Z[p] + (1-MAIN_POINT_WEIGHT)*sz/cnt;
	    
	    }
	}

    cout << endl << "Fixed " << fixed_holes << " holes." << endl << endl;
    
    // update number of valid points
    num_valid_points += fixed_holes;

    // allocate memory for index to coordinates look up table
    i_to_c = new long[num_valid_points];

    // PASS 2: generate triangles
    for (int i=0; i<ROWS-1; ++i)
	for (int j=0; j<COLS-1; ++j) {
	    // get the 4 points, indexed counter clockwise [0 3; 1 2]
	    np[0] = i*COLS + j;
	    np[1] = (i+1)*COLS + j;
	    np[2] = np[1] + 1;
	    np[3] = np[0] + 1;
	    
	    if (dist(np[0],np[2]) > dist(np[1],np[3])) {
		// choose the diagonal np[1] to np[3], rotate points
		long tmp = np[3];
		np[3] = np[0];
		np[0] = np[1];
		np[1] = np[2];
		np[2] = tmp;
	    }

	    // try to create 2 triangles
	    if (close_points(np[0], np[1], np[2])) {
		// found a good triangle, add it
		tr[num_tr][0] = get_index(np[0]);
		tr[num_tr][1] = get_index(np[1]);
		tr[num_tr][2] = get_index(np[2]);
		num_tr++;
	    }
	    
	    if (close_points(np[0], np[2], np[3])) {
		// good triangle, add it
		tr[num_tr][0] = get_index(np[0]);
		tr[num_tr][1] = get_index(np[2]);
		tr[num_tr][2] = get_index(np[3]);
		num_tr++;
	    }

	    if (num_tr==MAX_TRIANGLES) {
		cerr << "Increase MAX_TRIANGLES !" << endl;
		exit(1);
	    }
	    
	}
    
}


/*****************************************************************************/

int main(int argc, char **argv)
{
    int N = 0;		/* number of points */
    int offset = 0;
    double d;

    // parse command line    
    if (argc==3 && strcmp(argv[1], "-v")==0)
	// got -v flag
	VRMLformat = true;
    else
	if ( ((argc<2) || (strcmp(argv[1], "-i")==0) && argc!=3) || (argv[1][0]!='-' && argc!=2) ) {
	    // error, display error message and qut
	    cerr << endl << "Usage:  " << argv[0] << "  [-i|-v]  <base_file_name>" << endl << endl;
	    exit(1);
	}

    offset = (argv[1][0]=='-') ? 1 : 0;

    // open files
    open_files(argv[1+offset]);

    // read .params file
    read_params_file();

    // get data from range file
    range_file >> d; ROWS = (int) d;
    range_file >> d; COLS = (int) d;
    // allocate memory for X, Y, Z
    X = new double[ROWS*COLS];
    Y = new double[ROWS*COLS];
    Z = new double[ROWS*COLS];
    c_to_i = new long[ROWS*COLS]; // (-1); // not available in VC6

    for (int i=0; i<ROWS; ++i)
	for (int j=0; j<COLS; ++j) {
	    int indx = i*COLS + j;
	    range_file >> X[indx];
	    range_file >> Y[indx];
	    range_file >> Z[indx];
	    
	    // count number of valid points
	    if (Z[indx]>-INFINITY)
		num_valid_points++;
	}


    cout << "\nGot " << num_valid_points << " valid points." << endl;

    // create mesh
    create_mesh();


    cerr << "Created " << num_tr << " triangles." << endl;

    // write .iv or .wrl file
    write_file(argv[1+offset]);

    cout << "\nUsed " << num_pts_tr << " points for mesh." << endl;
    
    // deallocate memory
    delete [] X;
    delete [] Y;
    delete [] Z;
    delete [] c_to_i;
    delete [] i_to_c;

    // close all files
    close_files();
}

/*****************************************************************************/


