
/* ivcon.c  24 May 2001 */
// http://www.math.iastate.edu/burkardt/g_src/ivcon/ivcon.html

/* ivcon.cpp  22 Dec 2002 */

/*
Purpose:

IVCON converts various 3D graphics files.

Acknowledgements:

Coding, comments, and advice were supplied by a number of collaborators.

John F Flanagan made some corrections to the 3D Studio Max routines.

Zik Saleeba (zik@zikzak.net) enhanced the DXF routines, and added the 
Golgotha GMOD routines.

Thanks to Susan M. Fisher, University of North Carolina,
Department of Computer Science, for pointing out a coding error
in FACE_NULL_DELETE that was overwriting all the data!


Author: John Burkardt 

Modified: 04 July 2000

Modified: 23 Dec 2002 Ph. Guglielmetti www.DynaBits.com moved to C++

*/


#include "ivconv.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

#undef ERROR	// defined by windows
const bool ERROR=false ;
const bool SUCCESS=true;

const char* default_material_name="Material_0000";

float IVCONV::SCALE_FACTOR=1.0f;

IVCONV::IVCONV() 
{
	scale_factor=SCALE_FACTOR;
	init_program_data();
}

bool IVCONV::read(const string& file)
{
	data_init ( );
	strcpy(filein_name,file.c_str());
	return data_read ( );
}

bool IVCONV::write(const string& file)
{
	strcpy(fileout_name,file.c_str());
	return data_write ( );
}




bool debug;
bool byte_swap;

void IVCONV::cor3_normal_set ()


/*
Purpose:

  COR3_NORMAL_SET computes node normal vectors.
  
	Modified:
	
	  18 November 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int   icor3;
	int   iface;
	int   ivert;
	int   j;
	float norm;
	float temp;
	
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		for ( j = 0; j < 3; ++j ) {
			cor3_normal[icor3][j] = 0.0;
		}
	}
	/*
	Add up the normals at all the faces to which the node belongs.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			icor3 = face[ivert][iface];
			
			for ( j = 0; j < 3; ++j ) {
				cor3_normal[icor3][j] = cor3_normal[icor3][j]
					+ vertex_normal[ivert][iface][j];
			}
		}
	}
	/*
	Renormalize.
	*/
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		
		norm = 0.0;
		for ( j = 0; j < 3; ++j ) {
			temp = cor3_normal[icor3][j];
			norm = norm + temp * temp;
		}
		
		if ( norm == 0.0 ) {
			norm = 3.0;
			for ( j = 0; j < 3; ++j ) {
				cor3_normal[icor3][j] = 1.0;
			}
		}
		
		norm = ( float ) sqrt ( norm );
		
		for ( j = 0; j < 3; ++j ) {
			cor3_normal[icor3][j] = cor3_normal[icor3][j] / norm;
		}
	}
	
	return;
}
/******************************************************************************/

void IVCONV::cor3_range ()

/******************************************************************************/

/*
Purpose:

  COR3_RANGE computes the coordinate minima and maxima.
  
	Modified:
	
	  31 August 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int   i;
	float xave;
	float xmax;
	float xmin;
	float yave;
	float ymax;
	float ymin;
	float zave;
	float zmax;
	float zmin;
	
	xave = cor3[0][0];
	xmax = cor3[0][0];
	xmin = cor3[0][0];
	
	yave = cor3[0][1];
	ymax = cor3[0][1];
	ymin = cor3[0][1];
	
	zave = cor3[0][2];
	zmax = cor3[0][2];
	zmin = cor3[0][2];
	
	for ( i = 1; i < cor3_num; ++i ) {
		
		xave = xave + cor3[i][0];
		if ( cor3[i][0] < xmin ) {
			xmin = cor3[i][0];
		}
		if ( cor3[i][0] > xmax ) {
			xmax = cor3[i][0];
		}
		
		yave = yave + cor3[i][1];
		if ( cor3[i][1] < ymin ) {
			ymin = cor3[i][1];
		}
		if ( cor3[i][1] > ymax ) {
			ymax = cor3[i][1];
		}
		
		zave = zave + cor3[i][2];
		if ( cor3[i][2] < zmin ) {
			zmin = cor3[i][2];
		}
		if ( cor3[i][2] > zmax ) {
			zmax = cor3[i][2];
		}
	}
	
	xave = xave / cor3_num;
	yave = yave / cor3_num;
	zave = zave / cor3_num;
	
	printf ( "\n" );
	printf ( "COR3_RANGE - Data range:\n" );
	printf ( "\n" );
	printf ( "   Minimum   Average   Maximum  Range\n" );
	printf ( "\n" );
	printf ( "X  %f %f %f %f\n", xmin, xave, xmax, xmax-xmin );
	printf ( "Y  %f %f %f %f\n", ymin, yave, ymax, ymax-ymin );
	printf ( "Z  %f %f %f %f\n", zmin, zave, zmax, zmax-zmin );
	
}
/******************************************************************************/

void IVCONV::data_check ()

/******************************************************************************/

/*
Purpose:

  DATA_CHECK checks the input data.
  
	Modified:
	
	  18 May 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	int iface;
	int nfix;
	
	if ( color_num > COLOR_MAX ) {
		printf ( "\n" );
		printf ( "DATA_CHECK - Warning!\n" );
		printf ( "  The input data requires %d colors.\n", color_num );
		printf ( "  There was only room for %d\n", COLOR_MAX );
		color_num = COLOR_MAX;
	}
	
/*
	if ( cor3_num > COR3_MAX ) {
		printf ( "\n" );
		printf ( "DATA_CHECK - Warning!\n" );
		printf ( "  The input data requires %d points.\n", cor3_num );
		printf ( "  There was only room for %d\n", COR3_MAX );
		cor3_num = COR3_MAX;
	}
*/	
/*
	if ( face_num > FACE_MAX ) {
		printf ( "\n" );
		printf ( "DATA_CHECK - Warning!\n" );
		printf ( "  The input data requires %d faces.\n", face_num );
		printf ( "  There was only room for %d\n", FACE_MAX );
		face_num = FACE_MAX;
	}
	
	if ( line_num > LINES_MAX ) {
		printf ( "\n" );
		printf ( "DATA_CHECK - Warning!\n" );
		printf ( "  The input data requires %d line items.\n", line_num );
		printf ( "  There was only room for %d.\n", LINES_MAX );
		line_num = LINES_MAX;
	}
*/
	nfix = 0;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		if ( face_order[iface] > ORDER_MAX ) {
			face_order[iface] = ORDER_MAX;
			nfix = nfix + 1;
		}
		
	}
	
	if ( nfix > 0 ) {
		printf ( "\n" );
		printf ( "DATA_CHECK - Warning!\n" );
		printf ( "  Corrected %d faces using more than %d vertices per face.\n",
			nfix, ORDER_MAX );
	}
	
	for ( int i = 0; i < material_num; ++i ) {
		if ( strcmp ( material[i].name, "" ) == 0 ) {
			strcpy ( material[i].name, default_material_name );
		}
	}
	
	for ( int i = 0; i < texture_num; ++i ) {
		if ( strcmp ( texture_name[i], "" ) == 0 ) {
			strcpy ( texture_name[i], "Texture_0000" );
		}
	}
	
	printf ( "\n" );
	printf ( "DATA_CHECK - Data checked.\n" );
	
	return;
}

void IVCONV::materials_fixups()
{

	// If there are no materials at all, define one.
	// PhG : no, it will be defined only when exporting in a format that supports materials

	if ( material_num < 1 ) {
		material_num = 1;
		material[0]=Material(0.7f,0.7f,0.7f);
		strcpy ( material[0].name, default_material_name );
	}

	//If a node has not been assigned a material, set it to material 0.
	for ( long icor3 = 0; icor3 < cor3_num; icor3++ ) {
		if ( cor3_material[icor3] < 0 || cor3_material[icor3] > material_num - 1 ) {
			cor3_material[icor3] = 0;
		}
	}
	/*
	If a vertex has not been assigned a material, set it to material 0.
	*/
	for ( long iface = 0; iface < face_num; iface++ ) {
		for ( long ivert = 0; ivert < face_order[iface]; ivert++ ) {
			if ( vertex_material[ivert][iface] < 0 || vertex_material[ivert][iface] > material_num - 1 ) {
				vertex_material[ivert][iface] = 0;
			}
		}
	}
	/*
	If a face has not been assigned a material, set it to material 0.
	*/
	for ( int iface = 0; iface < face_num; iface++ ) {
		if ( face_material[iface] < 0 || face_material[iface] > material_num - 1 ) {
			face_material[iface] = 0;
		}
	}
	/*
	If a line item has not been assigned a material, set it to material 0.
	*/
	for ( long iline = 0; iline < line_num; iline++ ) {
		if ( line_dex[iline] == -1 ) {
			line_material[iline] = -1;
		}
		else if ( line_material[iline] < 0 || line_material[iline] > material_num - 1 ) {
			line_material[iline] = 0;
		}
	}
};

void IVCONV::data_init ()


/*
Purpose:

  DATA_INIT initializes the internal graphics data.
  
	Modified:
	
	  04 July 2000
	  
		Author:
		
		  John Burkardt
		  */
{
	
	color_num = 0;
	cor3_num = 0;
	face_num = 0;
	group_num = 0;
	line_num = 0;
	material_num = 0;
	object_num = 0;
	texture_num = 0;
	
	char object_name[500];

	strcpy ( object_name, "IVCON" );
	
	origin=vec3();
	pivot=vec3();
	
	/*
	for ( int j = 0; j < COLOR_MAX; ++j ) {
		rgbcolor[j][0] = 0.299;
		rgbcolor[j][1] = 0.587;
		rgbcolor[j][2] = 0.114;
	}
	*/
	strcpy ( texture_binding, "DEFAULT" );
	
	/*
	for ( j = 0; j < TEXTURE_MAX; ++j ) {
		strcpy ( texture_name[j], "Texture_0000" );
	}
	*/
	tmat_init ( transform_matrix );

	if ( debug ) {
		printf ( "\n" );
		printf ( "DATA_INIT: Graphics data initialized.\n" );
	}
	
	return;
}
/******************************************************************************/

bool IVCONV::data_read ()

/******************************************************************************/

/*
Purpose:

  DATA_READ reads a file into internal graphics data.
  
	Modified:
	
	  26 September 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	FILE *filein;
	char *filein_type;
	bool   ok;
	/* 
	Retrieve the input file type. 
	*/
	filein_type = file_ext ( filein_name );
	
	if ( filein_type == NULL ) {
		printf ( "\n" );
		printf ( "DATA_READ - Fatal error!\n" );
		printf ( "  Could not determine the type of '%s'.\n", filein_name );
		return ERROR;
	}
	else if ( debug ) {
		printf ( "\n" );
		printf ( "DATA_READ: Input file has type %s.\n", filein_type );  
	}
	/*
	Initialize some data.
	*/
	max_order2 = 0;
	bad_num = 0;
	bytes_num = 0;
	comment_num = 0;
	dup_num = 0;
	text_num = 0;
	/* 
	Open the file. 
	*/
	if ( leqi ( filein_type, "3DS" ) ||
		leqi ( filein_type, "STLB" ) ||
		leqi ( filein_type, "TRIB" ) ) {
		filein = fopen ( filein_name, "rb" );
	}
	else {
		filein = fopen ( filein_name, "r" );
	}
	
	if ( filein == NULL ) {
		printf ( "\n" );
		printf ( "DATA_READ - Fatal error!\n" );
		printf ( "  Could not open the input file '%s'!\n", filein_name );
		return ERROR;
	}
	/*
	Read the information in the file. 
	*/
	if ( leqi ( filein_type, "3DS" )) {
		
		ok = tds_read ( filein );
		/*
		Cleanup: distribute the node textures to the vertices.
		*/
		if (ok) {
			
			for ( long iface = 0; iface < face_num; iface++ ) {
				for ( long ivert = 0; ivert < face_order[iface]; ivert++ ) {
					long icor3 = face[ivert][iface];
					vertex_tex_uv[iface][ivert][0] = cor3_tex_uv[icor3][0];
					vertex_tex_uv[iface][ivert][1] = cor3_tex_uv[icor3][1];
				}
			}
			
		}
		
	}
	else if ( leqi ( filein_type, "ASC" )  ) {
		
		ok = asc_read ( filein );
		
	}
	else if ( leqi ( filein_type, "ASE" )) {
		
		ok = ase_read ( filein );
		
		if (ok) {
			
			node_to_vertex_material ( );
			
			vertex_to_face_material ( );
			
		}
		
	}
	else if ( leqi ( filein_type, "BYU" )  ) {
		
		ok = byu_read ( filein );
		
	}
	else if ( leqi ( filein_type, "DXF" )  ) {
		
		ok = dxf_read ( filein );
		
	}
	else if ( leqi ( filein_type, "GMOD" )  ) {
		
		ok = gmod_read ( filein );
		
	}
	else if ( leqi ( filein_type, "HRC" )  ) {
		
		ok = hrc_read ( filein );
		
	}
	else if ( leqi ( filein_type, "IV" )  ) {
		
		ok = iv_read ( filein );
		
	}
		
	else if ( leqi ( filein_type, "WRL" )  ) {
		
		ok = iv_read ( filein );	// reads also vrml
		
	}

	else if ( leqi ( filein_type, "OBJ" )  ) {
		
		ok = obj_read ( filein );
		
	}
	else if ( leqi ( filein_type, "SMF" )  ) {
		
		ok = smf_read ( filein );
		
	}
	else if ( leqi ( filein_type, "STLA")  ) {
		
		ok = stla_read ( filein );
		
	}
	else if ( leqi ( filein_type, "STL" )  ||
		  leqi ( filein_type, "STLB")  )  {
		
		ok = stlb_read ( filein );
		
	}
	else if ( 
		leqi ( filein_type, "TRI" )  ||
		leqi ( filein_type, "TRIA")  ) {
		
		ok = tria_read ( filein );
		
	}
	else if ( leqi ( filein_type, "TRIB")  ) {
		
		ok = trib_read ( filein );
		
	}
	else if ( leqi ( filein_type, "VLA" )  ) {
		
		ok = vla_read ( filein );
		
	}
	else {
		printf ( "\n" );
		printf ( "DATA_READ - Fatal error!\n" );
		printf ( "  Unacceptable input file type.\n" );
		return ERROR;
	}
	
	fclose ( filein );
	
	if ( debug ) {
		printf ( "DATA_READ: Finished reading the data file.\n" );
	}
	/*
	Catch errors reported by the various reading routines.
	*/
	if (!ok) return false;
	/*
	Restore the transformation matrix.
	*/
	tmat_init ( transform_matrix );
	/*
	Report on what we read.
	*/
	long ntemp = face_num;
	max_order2 = ivec_max ( ntemp, face_order );
	
	data_report ( );
	/*
	Warn about any errors that occurred during reading.
	*/
	if (!ok) {
		printf ( "\n" );
		printf ( "DATA_READ - Fatal error!\n" );
		printf ( "  An error occurred while reading the input file.\n" );
		return false;
	}
	/*
	Check the data.
	You MUST wait until after this check before doing other computations,
	since COR3_NUM and other variables could be much larger than the legal
	maximums, until corrected by DATA_CHECK.
	*/
	data_check ( );
	
	// materials_fixups();	// PhG done only before exporting in formats that support materials

	/*
	Delete edges of zero length.
	*/
	edge_null_delete ( );
	/*
	Compute the area of each face.
	*/
	face_area_set ( );
	/*
	Delete faces with zero area.
	*/
	face_null_delete ( );
	/*
	Recompute zero face-vertex normals from vertex positions.
	*/
	vertex_normal_set ( );
	/*
	Compute the node normals from the vertex normals.
	*/
	cor3_normal_set ( );
	/*
	Recompute zero face normals by averaging face-vertex normals.
	*/
	face_normal_ave ( );
	/*
	Report on the nodal coordinate range.
	*/
	cor3_range ( );
	
	return SUCCESS;
}
/**********************************************************************/

void IVCONV::data_report ()

/**********************************************************************/

/*
Purpose:

  DATA_REPORT gives a summary of the contents of the data file.
  
	Modified:
	
	  24 May 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	printf ( "\n" );
	printf ( "DATA_REPORT - The input file contains:\n" );
	printf ( "\n" );
	printf ( "  Bad data items             %i\n", bad_num );
	printf ( "  Text lines                 %i\n", text_num );
	printf ( "  Text bytes (binary data)   %i\n", bytes_num );
	printf ( "  Colors                     %i\n", color_num );
	printf ( "  Comments                   %i\n", comment_num );
	printf ( "  Duplicate points           %i\n", dup_num );
	printf ( "  Faces                      %li\n", face_num );
	printf ( "  Groups                     %i\n", group_num );
	printf ( "  Vertices per face, maximum %i\n", max_order2 );
	printf ( "  Line items                 %li\n", line_num );
	printf ( "  Points                     %li\n", cor3_num );
	printf ( "  Objects                    %i\n", object_num );
	
	return;
}
/******************************************************************************/

bool IVCONV::data_write ()

/******************************************************************************/

/*
Purpose:

  DATA_WRITE writes the internal graphics data to a file.
  
	Modified:
	
	  22 May 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	FILE *fileout;
	char *fileout_type;
	int   line_num_save;
	int   result;
	
	result = SUCCESS;
	/* 
	Retrieve the output file type. 
	*/
	fileout_type = file_ext ( fileout_name );
	
	if ( fileout_type == NULL ) {
		printf ( "\n" );
		printf ( "DATA_WRITE - Fatal error!\n" );
		printf ( "  Could not determine the output file type.\n" );
		return ERROR;
	}
	/* 
	Open the output file. 
	*/
	if ( leqi ( fileout_type, "3DS" )  ||
		leqi ( fileout_type, "STLB" )  ||
		leqi ( fileout_type, "TRIB" ) ) {
		fileout = fopen ( fileout_name, "wb" );
	}
	else {
		fileout = fopen ( fileout_name, "w" );
	}
	
	if ( fileout == NULL ) {
		printf ( "\n" );
		printf ( "DATA_WRITE - Fatal error!\n" );
		printf ( "  Could not open the output file!\n" );
		return ERROR;
	}
	/* 
	Write the output file. 
	*/
	if ( leqi ( fileout_type, "3DS" )  ) {
		
		result = tds_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "ASE" )  ) {
		
		result = ase_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "BYU" )  ) {
		
		result = byu_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "DXF" )  ) {
		
		result = dxf_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "GMOD" )  ) {
		
		result = gmod_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "HRC" )  ) {
		
		result = hrc_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "IV" )  ) {
		
		result = iv_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "OBJ" )  ) {
		
		result = obj_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "POV" )  ) {
		
		result = pov_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "SMF" )  ) {
		
		result = smf_write ( fileout );
		
	}
	else if ( 
		leqi ( fileout_type, "STL" )  ||
		leqi ( fileout_type, "STLA" )  ) {
		
		result = stla_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "STLB" )  ) {
		
		result = stlb_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "TEC" )  ) {
		
		result = tec_write ( fileout );
		
	}
	else if ( 
		leqi ( fileout_type, "TRI" )  ||
		leqi ( fileout_type, "TRIA" )  ) {
		
		result = tria_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "TRIB" )  ) {
		
		result = trib_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "TXT" )  ) {
		
		result = txt_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "UCD" )  ) {
		
		result = ucd_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "VLA" )  ) {
		
		line_num_save = line_num;
		
		if ( face_num > 0 ) {
			
			printf ( "\n" );
			printf ( "DATA_WRITE - Note:\n" );
			printf ( "  Face information will temporarily be converted to\n" );
			printf ( "  line information for output to a VLA file.\n" );
			
			face_to_line (true);	// pruning on
		}
			
	result = vla_write ( fileout );
	
	line_num = line_num_save;
	
	}
	else if ( leqi ( fileout_type, "WRL" )  ) {
		
		result = wrl_write ( fileout );
		
	}
	else if ( leqi ( fileout_type, "XGL" )  ) {
		
		result = xgl_write ( fileout );
		
	}
	else {
		
		result = ERROR;
		printf ( "\n" );
		printf ( "DATA_WRITE - Fatal error!\n" );
		printf ( "  Unacceptable output file type \"%s\".\n", fileout_type );
		
	}
	/*  
	Close the output file. 
	*/
	fclose ( fileout );
	
	if ( result == ERROR ) {
		return ERROR;
	}
	else {
		return SUCCESS;
	}
}
/******************************************************************************/


/**********************************************************************/

void IVCONV::edge_null_delete ()

/**********************************************************************/

/*
Purpose:

  EDGE_NULL_DELETE deletes face edges with zero length.
  
	Modified:
	
	  16 July 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	float distsq;
	int face2[ORDER_MAX];
	int face_order2;
	int iface;
	int inode;
	int ivert;
	int j;
	int jnode;
	int jvert;
	int edge_num;
	int edge_num_del;
	float vertex_normal2[3][ORDER_MAX];
	float x;
	float y;
	float z;
	
	edge_num = 0;
	edge_num_del = 0;
	/*
	Consider each face.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
	/*
	Consider each pair of consecutive vertices.
		*/
		face_order2 = 0;
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			edge_num = edge_num + 1;
			
			jvert = ivert + 1;
			if ( jvert >= face_order[iface] ) {
				jvert = 0;
			}
			
			inode = face[ivert][iface];
			jnode = face[jvert][iface];
			
			
			x = cor3[inode][0] - cor3[jnode][0];
			y = cor3[inode][1] - cor3[jnode][1];
			z = cor3[inode][2] - cor3[jnode][2];
			
			distsq = x * x + y * y + z * z;
			
			if ( distsq != 0.0 ) {
				face2[face_order2] = face[ivert][iface];
				vertex_normal2[0][face_order2] = vertex_normal[ivert][iface][0];
				vertex_normal2[1][face_order2] = vertex_normal[ivert][iface][1];
				vertex_normal2[2][face_order2] = vertex_normal[ivert][iface][2];
				face_order2 = face_order2 + 1;
			}
			else {
				edge_num_del = edge_num_del + 1;
			}
			
		}
		
		face_order[iface] = face_order2;
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			face[ivert][iface] = face2[ivert];
			for ( j = 0; j < 3; ++j ) {
				vertex_normal[ivert][iface][j] = vertex_normal2[j][ivert];
			}
		}
		
	}
	
	printf ( "\n" );
	printf ( "EDGE_NULL_DELETE:\n" );
	printf ( "  There are a total of %d edges.\n", edge_num );
	printf ( "  Of these, %d were of zero length, and deleted.\n", edge_num_del );
	
	return;
}
/**********************************************************************/

void IVCONV::face_area_set ()

/**********************************************************************/

/*
Purpose:

  FACE_AREA_SET computes the area of the faces.
  
	Formula:
	
	  The area is the sum of the areas of the triangles formed by
	  node N with consecutive pairs of nodes.
	  
		Reference:
		
		  Adrian Bowyer and John Woodwark,
		  A Programmer's Geometry,
		  Butterworths, 1983.
		  
			Modified:
			
			  17 July 1999
			  
				Author:
				
				  John Burkardt
				  */
{
	float alpha;
	float area_max;
	float area_min;
	float area_tri;
	float base;
	float dot;
	float height;
	int i;
	int i1;
	int i2;
	int i3;
	int iface;
	int face_num_del;
	float tol;
	float x;
	float x1;
	float x2;
	float x3;
	float y;
	float y1;
	float y2;
	float y3;
	float z;
	float z1;
	float z2;
	float z3;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		face_area[iface] = 0.0;
		
		for ( i = 0; i < face_order[iface]-2; ++i ) {
			
			i1 = face[i][iface];
			i2 = face[i+1][iface];
			i3 = face[i+2][iface];
			
			x1 = cor3[i1][0];
			y1 = cor3[i1][1];
			z1 = cor3[i1][2];
			
			x2 = cor3[i2][0];
			y2 = cor3[i2][1];
			z2 = cor3[i2][2];
			
			x3 = cor3[i3][0];
			y3 = cor3[i3][1];
			z3 = cor3[i3][2];
			/*
			Find the projection of (P3-P1) onto (P2-P1).
			*/
			dot =
				( x2 - x1 ) * ( x3 - x1 ) +
				( y2 - y1 ) * ( y3 - y1 ) +
				( z2 - z1 ) * ( z3 - z1 );
			
			base = sqrt (
				( x2 - x1 ) * ( x2 - x1 )
				+ ( y2 - y1 ) * ( y2 - y1 )
				+ ( z2 - z1 ) * ( z2 - z1 ) );
				/*
				The height of the triangle is the length of (P3-P1) after its
				projection onto (P2-P1) has been subtracted.
			*/
			if ( base == 0.0 ) {
				height = 0.0;
			}
			else {
				
				alpha = dot / ( base * base );
				
				x = x3 - x1 - alpha * ( x2 - x1 );
				y = y3 - y1 - alpha * ( y2 - y1 );
				z = z3 - z1 - alpha * ( z2 - z1 );
				
				height = sqrt ( x * x + y * y + z * z );
				
			}
			
			area_tri = 0.5 * base * height;
			
			face_area[iface] = face_area[iface] + area_tri;
			
		}
		
	}
	
	area_min = face_area[0];
	area_max = face_area[0];
	
	for ( iface = 1; iface < face_num; iface++ ) {
		if ( area_min > face_area[iface] ) {
			area_min = face_area[iface];
		}
		if ( area_max < face_area[iface] ) {
			area_max = face_area[iface];
		}
	}
	
	printf ( "\n" );
	printf ( "FACE_AREA_SET:\n" );
	printf ( "  Minimum face area is %f\n", area_min );
	printf ( "  Maximum face area is %f\n", area_max );
	
	tol = area_max / 10000.0;
	
	if ( area_min < tol ) {
		
		face_num_del = 0;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			if ( face_area[iface] < tol ) {
				face_order[iface] = 0;
				face_num_del = face_num_del + 1;
			}
		}
		
		printf ( "  Marked %d tiny faces for deletion.\n", face_num_del );
		
	}
	
	return;
}
/******************************************************************************/

void IVCONV::face_normal_ave ()

/******************************************************************************/

/*
Purpose:

  FACE_NORMAL_AVE sets face normals as average of face vertex normals.
  
	Modified:
	
	  09 October 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int   i;
	int   iface;
	int   ivert;
	int   nfix;
	float norm;
	float x;
	float y;
	float z;
	
	if ( face_num <= 0 ) {
		return;
	}
	
	nfix = 0;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
	/*
	Check the norm of the current normal vector.
		*/
		x = face_normal[iface][0];
		y = face_normal[iface][1];
		z = face_normal[iface][2];
		norm = ( float ) sqrt ( x * x + y * y + z * z );
		
		if ( norm == 0.0 ) {
			
			nfix = nfix + 1;
			
			for ( i = 0; i < 3; ++i ) {
				face_normal[iface][i] = 0.0;
			}
			
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				for ( i = 0; i < 3; ++i ) {
					face_normal[iface][i] = face_normal[iface][i] +
						vertex_normal[ivert][iface][i];
				}
			}
			
			x = face_normal[iface][0];
			y = face_normal[iface][1];
			z = face_normal[iface][2];
			norm = ( float ) sqrt ( x * x + y * y + z * z );
			
			if ( norm == 0.0 ) {
				for ( i = 0; i < 3; ++i ) {
					face_normal[iface][i] = ( float ) ( 1.0 / sqrt ( 3.0 ) );
				}
			}
			else {
				for ( i = 0; i < 3; ++i ) {
					face_normal[iface][i] = face_normal[iface][i] / norm;
				}
			}
		}
	}
	
	if ( nfix > 0 ) {
		printf ( "\n" );
		printf ( "FACE_NORMAL_AVE: Recomputed %d face normals\n", nfix );
		printf ( "  by averaging face vertex normals.\n" );
	}
	return;
}
/**********************************************************************/

void IVCONV::face_null_delete ()

/**********************************************************************/

/*
Purpose:

  FACE_NULL_DELETE deletes faces of order less than 3.
  
	Comments:
	
	  Thanks to Susan M. Fisher, University of North Carolina,
	  Department of Computer Science, for pointing out a coding error
	  in FACE_NULL_DELETE that was overwriting all the data!
	  
		Modified:
		
		  30 November 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int face_num2=0;
	/*
	FACE_NUM2 is the number of faces we'll keep.
	*/
	/*
	Check every face.
	*/
	for ( int iface = 0; iface < face_num; iface++ ) {
	/*
	Keep it only if it has order 3 or more.
		*/
		if ( face_order[iface] >= 3 ) {
		/*
		We don't have to slide data down in the array until
		NUMFACE2 and IFACE get out of synch, that is, after
		we've discarded at least one face.
			*/
			if ( face_num2 != iface ) {
				
				face_area[face_num2] = face_area[iface];
				face_material[face_num2] = face_material[iface];
				face_order[face_num2] = face_order[iface];
/*	printf ( "NUM FACE ORDER: %i\n",face_order.size());
	printf ( "face num: %i\n",face_num);
	printf ( "face num 2: %i\n",face_num2);
	printf ( "iface: %i\n",iface);*/
				for ( int ivert = 0; ivert < ORDER_MAX; ivert++ ) {
/*	printf ( "%i/%i\n",ivert, face.size());
	printf ( "%i/%i\n",face_num2,face[ivert].size());
	printf ( "%i/%i\n",iface,face[ivert].size());*/
					if (iface < face[ivert].size() ) 
						face[ivert][face_num2] = face[ivert][iface];
					if (iface < vertex_material[ivert].size()) 
						vertex_material[ivert][face_num2] = vertex_material[ivert][iface];
					if (iface < vertex_normal[ivert].size()) {
						for ( int j = 0; j < 3; ++j ) {
							vertex_normal[ivert][face_num2][j] = vertex_normal[ivert][iface][j];
						}
					}
				}
				
			}
			/*
			Update the count only after we've used the un-incremented value
			as a pointer.
			*/
			++face_num2;
		}
		
	}
	
	printf ( "\n" );
	printf ( "FACE_NULL_DELETE\n" );
	printf ( "  There are a total of %li faces.\n", face_num );
	printf ( "  Of these, %i passed the order test.\n", face_num2 );
	
	face_num = face_num2;
	
	return;
}
/******************************************************************************/
bool IVCONV::face_print ( int iface )

/******************************************************************************/

/*
Purpose:

  FACE_PRINT prints out information about a face.
  
	Modified:
	
	  31 August 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int ivert;
	int j;
	int k;
	
	if ( iface < 0 || iface > face_num-1 ) {
		printf ( "\n" );
		printf ( "FACE_PRINT - Fatal error!\n" );
		printf ( "  Face indices must be between 1 and %li\n", face_num );
		printf ( "  But your requested value was %d\n", iface );
		return false;
	}
	
	printf ( "\n" );
	printf ( "FACE_PRINT\n" );
	printf ( "  Information about face %d\n", iface );
	printf ( "\n" );
	printf ( "  Number of vertices is %d\n", face_order[iface] );
	printf ( "\n" );
	printf ( "  Vertex list:\n" );
	printf ( "    Vertex #, Node #, Material #, X, Y, Z:\n" );
	printf ( "\n" );
	for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
		j = face[ivert][iface];
		k = vertex_material[ivert][iface];
		printf ( " %d %d %d %f %f %f\n", ivert, j, k, cor3[j][0], cor3[j][1], 
			cor3[j][2] );
	}
	
	printf ( "\n" );
	printf ( "  Face normal vector:\n" );
	printf ( "\n" );
	printf ( " %f %f %f\n", face_normal[iface][0], face_normal[iface][1],
		face_normal[iface][2] );
	
	printf ( "\n" );
	printf ( "  Vertex face normals:\n" );
	printf ( "\n" );
	for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
		printf ( " %d %f %f %f\n", ivert, vertex_normal[ivert][iface][0],
			vertex_normal[ivert][iface][1], vertex_normal[ivert][iface][2] );
	}
	
	return true;
	
}
/**********************************************************************/

bool IVCONV::face_reverse_order ()

/**********************************************************************/

/*
Purpose:

  FACE_REVERSE_ORDER reverses the order of the nodes in each face.
  
	Discussion:
	
	  Reversing the order of the nodes requires that the normal vectors
	  be reversed as well, so this routine will automatically reverse
	  the normals associated with nodes, vertices and faces.
	  
		Modified:
		
		  28 June 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int i;
	int iface;
	int itemp;
	int ivert;
	int j;
	int m;
	float temp;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		m = face_order[iface];
		
		for ( ivert = 0; ivert < ( m / 2 ); ivert++ ) {
			
			itemp = face[ivert][iface];
			face[ivert][iface] = face[m-1-ivert][iface];
			face[m-1-ivert][iface] = itemp;
			
			itemp = vertex_material[ivert][iface];
			vertex_material[ivert][iface] = vertex_material[m-1-ivert][iface];
			vertex_material[m-1-ivert][iface] = itemp;
			
			for ( j = 0; j < 3; ++j ) {
				temp = vertex_normal[ivert][iface][j];
				vertex_normal[ivert][iface][j] = vertex_normal[m-1-ivert][iface][j];
				vertex_normal[m-1-ivert][iface][j] = temp;
			}
			
			for ( j = 0; j < 2; ++j ) {
				temp = vertex_tex_uv[iface][ivert][j];
				vertex_tex_uv[iface][ivert][j] = vertex_tex_uv[iface][m-1-ivert][j];
				vertex_tex_uv[iface][m-1-ivert][j] = temp;
			}
			
		}
		
	}
	
	for ( i = 0; i < cor3_num; ++i ) {
		for ( j = 0; j < 3; ++j ) {
			cor3_normal[i][j] = - cor3_normal[i][j];
		}
	}
	
	for ( i = 0; i < face_num; ++i ) {
		for ( j = 0; j < 3; ++j ) {
			face_normal[i][j] = - face_normal[i][j];
		}
	}
	
	printf ( "\n" );
	printf ( "FACE_REVERSE_ORDER\n" );
	printf ( "  Each list of nodes defining a face\n" );
	printf ( "  has been reversed; related information,\n" );
	printf ( "  including normal vectors, was also updated.\n" );
	
	return true;
}

/******************************************************************************/

bool IVCONV::face_subset ()

/******************************************************************************/

/*
Purpose:

  FACE_SUBSET selects a subset of the current faces as the new object.
  
	Warning:
	
	  The original graphic object is overwritten by the new one.
	  
		Modified:
		
		  12 October 1998
		  
			Author:
			
			  John Burkardt
			  
				*/
{
	int i;
	int iface;
	int iface1;
	int iface2;
	int inc;
	int ivert;
	int j;
	int k;
	int cor3_num2;
	
	line_num = 0;
	/*
	Get the first and last faces to save, IFACE1 and IFACE2.
	*/
	printf ( "\n" );
	printf ( "Enter lowest face number to save between 0 and %li:\n", face_num-1 );
	scanf ( "%d", &iface1 );
	if ( iface1 < 0 || iface1 > face_num - 1 ) {
		printf ( "Illegal choice!\n" );
		return ERROR;
	}
	
	printf ( "\n" );
	printf ( "Enter highest face number to save between %i and %li:\n", 
		iface1, face_num-1 );
	scanf ( "%d", &iface2 );
	if ( iface2 < iface1 || iface2 > face_num - 1 ) {
		printf ( "Illegal choice!\n" );
		return ERROR;
	}
	
	inc = iface1;
	/*
	"Slide" the data for the saved faces down the face arrays.
	*/
	for ( iface = 0; iface < iface2 + 1 - iface1; iface++ ) {
		face_order[iface] = face_order[iface+inc];
		for ( ivert = 0; ivert < ORDER_MAX; ivert++ ) {
			face[ivert][iface] = face[ivert][iface+inc];
			vertex_material[ivert][iface] = vertex_material[ivert][iface+inc];
			for ( i = 0; i < 3; ++i ) {
				vertex_normal[ivert][iface][i] =
					vertex_normal[ivert][iface+inc][i];
				vertex_rgb[ivert][iface][i] = vertex_rgb[ivert][iface+inc][i];
			}
		}
		for ( i = 0; i < 3; ++i ) {
			face_normal[iface][i] = face_normal[iface+inc][i];
		}
	}
	/*
	Now reset the number of faces.
	*/
	face_num = iface2 + 1 - iface1;
	/*
	Now, for each point I, set LIST(I) = J if point I is the J-th
	point we are going to save, and 0 otherwise.  Then J will be
	the new label of point I.
	*/
	for ( i = 0; i < cor3_num; ++i ) {
		list[i] = -1;
	}
	
	cor3_num2 = 0;
	
	for ( iface = 0; iface < face_num; iface++ ){
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ){
			j = face[ivert][iface];
			if ( list[j] == -1 ) {
				cor3_num2 = cor3_num2 + 1;
				list[j] = cor3_num2;
			}
		}
	}
	/*
	Now make the nonzero list entries rise in order, so that
	we can compress the COR3 data in a minute.
	*/
	cor3_num2 = 0;
	
	for ( i = 0; i < cor3_num; ++i ) {
		if ( list[i] != -1 ) {
			list[i] = cor3_num2;
			cor3_num2 = cor3_num2 + 1;
		}
	}
	/*
	Relabel the FACE array with the new node indices.
	*/
	for ( iface = 0; iface < face_num; iface++ ){
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ){
			j = face[ivert][iface];
			face[ivert][iface] = list[j];
		}
	}
	/*
	Rebuild the COR3 array by sliding data down.
	*/
	for ( i = 0; i < cor3_num; ++i ){
		k = list[i];
		if ( k != -1 ) {
			for ( j = 0; j < 3; ++j ) {
				cor3[k][j] = cor3[i][j];
			}
		}
	}
	
	cor3_num = cor3_num2;
	
	return true;
}
/**********************************************************************/

bool IVCONV::face_to_line (bool line_prune)

/**********************************************************************/

/*
Purpose:

FACE_TO_LINE converts face information to line information.

Discussion:

In some cases, the graphic information represented by polygonal faces
must be converted to a representation based solely on line segments.
This is particularly true if a VLA file is being written.

Modified:

26 May 1999

Author:

John Burkardt
*/
{
	int icor3;
	int iface;
	int ivert;
	int jcor3;
	int jvert;
	/*
	Case 0:
	No line pruning.
	*/
	if (!line_prune) {
		
		for ( iface = 0; iface < face_num; iface++ ) {
			
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				
				icor3 = face[ivert][iface];
				
				++line_num;
				if ( line_num <= LINES_MAX ) {
					line_dex[line_num] = icor3;
					line_material[line_num] = vertex_material[ivert][iface];
				}
			}
			
			ivert = 0;
			icor3 = face[ivert][iface];
			
			++line_num;
			if ( line_num <= LINES_MAX ) {
				line_dex[line_num] = icor3;
				line_material[line_num] = vertex_material[ivert][iface];
			}
			
			++line_num;
			if ( line_num <= LINES_MAX ) {
				line_dex[line_num] = -1;
				line_material[line_num] = -1;
			}
		}
		
	}
	/*
	Case 2:
	Simple-minded line pruning.
	Only draw line (I,J) if I < J.
	*/
	else {
		
		for ( iface = 0; iface < face_num; iface++ ) {
			
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				
				icor3 = face[ivert][iface];
				
				if ( ivert + 1 < face_order[iface] ) {
					jvert = ivert + 1;
				}
				else {
					jvert = 0;
				}
				
				jcor3 = face[jvert][iface];
				
				if ( icor3 < jcor3 ) {
					
					if ( line_num + 3 < LINES_MAX ) {
						
						++line_num;
						line_dex[line_num] = icor3;
						line_material[line_num] = vertex_material[ivert][iface];
						
						++line_num;
						line_dex[line_num] = jcor3;
						line_material[line_num] = vertex_material[jvert][iface];
						
						++line_num;
						line_dex[line_num] = -1;
						line_material[line_num] = -1;
						
					}
				}
			}
		}
		
	}
	face_num = 0;

	if ( line_num > LINES_MAX ) {
		
		printf ( "\n" );
		printf ( "INTERACT - Note:\n" );
		printf ( "  Some face information was lost.\n" );
		printf ( "  The maximum number of lines is %li,\n", LINES_MAX );
		printf ( "  but we would need at least %li.\n", line_num );
		
		line_num = LINES_MAX;
		return false;
	}

	return true;
}
/**********************************************************************/

void IVCONV::face_to_vertex_material ()

/**********************************************************************/

/*
Purpose:

  FACE_TO_VERTEX_MAT extends face material definitions to vertices.
  
	Discussion:
	
	  Assuming material indices are defined for all the faces, this
	  routine assigns to each vertex of a face the material of that face.
	  
		Modified:
		
		  22 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int iface;
	int ivert;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			vertex_material[ivert][iface] = face_material[iface];
		}
	}
	
	return;
}

void IVCONV::init_program_data ()

/******************************************************************************/

/*
Purpose:

INIT_PROGRAM_DATA initializes the internal program data.

Modified:

26 May 1999

Author:

John Burkardt
*/
{
	byte_swap = false;
	debug = 0;
	color_num = 0;
	cor3_num = 0;
	face_num = 0;
	line_num = 0;
	
	if ( debug ) {
		printf ( "\n" );
		printf ( "INIT_PROGRAM_DATA: Program data initialized.\n" );
	}
	
	return;
	
}

/******************************************************************************/

void IVCONV::node_to_vertex_material ()

/**********************************************************************/

/*
Purpose:

NODE_TO_VERTEX_MAT extends node material definitions to vertices.

Discussion:

A NODE is a point in space.
A VERTEX is a node as used in a particular face.
One node may be used as a vertex in several faces, or none.

Modified:

22 May 1999

Author:

John Burkardt
*/
{
	int iface;
	int ivert;
	int node;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			node = face[ivert][iface];
			vertex_material[ivert][iface] = cor3_material[node];
		}
	}
	
	return;
}
/******************************************************************************/

/**********************************************************************/




void IVCONV::vertex_normal_set ()

/******************************************************************************/

/*
Purpose:

  VERTEX_NORMAL_SET recomputes the face vertex normal vectors.
  
	Modified:
	
	  12 October 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int   i;
	int   i0;
	int   i1;
	int   i2;
	int   iface;
	int   ivert;
	int   jp1;
	int   jp2;
	int   nfix;
	float norm;
	float temp;
	float x0;
	float x1;
	float x2;
	float xc;
	float y0;
	float y1;
	float y2;
	float yc;
	float z0;
	float z1;
	float z2;
	float zc;
	
	if ( face_num <= 0 ) {
		return;
	}
	
	nfix = 0;
	/*
	Consider each face.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			norm = 0.0;
			for ( i = 0; i < 3; ++i ) {
				temp = vertex_normal[ivert][iface][i];
				norm = norm + temp * temp;
			}
			norm = ( float ) sqrt ( norm );
			
			if ( norm == 0.0 ) {
				
				nfix = nfix + 1;
				
				i0 = face[ivert][iface];
				x0 = cor3[i0][0];
				y0 = cor3[i0][1];
				z0 = cor3[i0][2];
				
				jp1 = ivert + 1;
				if ( jp1 >= face_order[iface] ) {
					jp1 = jp1 - face_order[iface];
				}
				i1 = face[jp1][iface];
				x1 = cor3[i1][0];
				y1 = cor3[i1][1];
				z1 = cor3[i1][2];
				
				jp2 = ivert + 2;
				if ( jp2 >= face_order[iface] ) {
					jp2 = jp2 - face_order[iface];
				}
				i2 = face[jp2][iface];
				x2 = cor3[i2][0];
				y2 = cor3[i2][1];
				z2 = cor3[i2][2];
				
				xc = ( y1 - y0 ) * ( z2 - z0 ) - ( z1 - z0 ) * ( y2 - y0 );
				yc = ( z1 - z0 ) * ( x2 - x0 ) - ( x1 - x0 ) * ( z2 - z0 );
				zc = ( x1 - x0 ) * ( y2 - y0 ) - ( y1 - y0 ) * ( x2 - x0 );
				
				norm = ( float ) sqrt ( xc * xc + yc * yc + zc * zc );
				
				if ( norm == 0.0 ) {
					xc = ( float ) 1.0 / sqrt ( 3.0 );
					yc = ( float ) 1.0 / sqrt ( 3.0 );
					zc = ( float ) 1.0 / sqrt ( 3.0 );
				}
				else {
					xc = xc / norm;
					yc = yc / norm;
					zc = zc / norm;
				}
				
				vertex_normal[ivert][iface][0] = xc;
				vertex_normal[ivert][iface][1] = yc;
				vertex_normal[ivert][iface][2] = zc;
				
			}
		}
	}
	
	if ( nfix > 0 ) {
		printf ( "\n" );
		printf ( "VERTEX_NORMAL_SET: Recomputed %d face vertex normals.\n", nfix );
	}
	
	return;
}
/**********************************************************************/

void IVCONV::vertex_to_face_material ()

/**********************************************************************/

/*
Purpose:

  VERTEX_TO_FACE_MATERIAL extends vertex material definitions to faces.
  
	Discussion:
	
	  Assuming material indices are defined for all the vertices, this
	  routine assigns to each face the material associated with its
	  first vertex.
	  
		Modified:
		
		  22 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int iface;
	int ivert;
	
	ivert = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		face_material[iface] = vertex_material[ivert][iface];
	}
	
	return;
}
/**********************************************************************/

void IVCONV::vertex_to_node_material ()

/**********************************************************************/

/*
Purpose:

  VERTEX_TO_NODE_MATERIAL extends vertex material definitions to nodes.
  
	Discussion:
	
	  A NODE is a point in space.
	  A VERTEX is a node as used in a particular face.
	  One node may be used as a vertex in several faces, or none.
	  This routine simply runs through all the vertices, and assigns
	  the material of the vertex to the corresponding node.  If a
	  node appears as a vertex several times, then the node will
	  end up having the material of the vertex that occurs "last".
	  
		Modified:
		
		  22 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int iface;
	int ivert;
	int node;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			node = face[ivert][iface];
			cor3_material[node] = vertex_material[ivert][iface];
		}
	}
	
	return;
}

/******************************************************************************/
bool IVCONV::reverse_normals()
{
	for ( long icor3 = 0; icor3 < cor3_num; icor3++ ) {
		for ( int i = 0; i < 3; ++i ) {
			cor3_normal[icor3][i] = - cor3_normal[icor3][i];
		}
	}
	
	for ( long iface = 0; iface < face_num; iface++ ) {
		for ( int i = 0; i < 3; ++i ) {
			face_normal[iface][i] = - face_normal[iface][i];
		}
	}
	
	for ( int iface = 0; iface < face_num; iface++ ) {
		for ( long ivert = 0; ivert < face_order[iface]; ivert++ ) {
			for ( int i = 0; i < 3; ++i ) {
				vertex_normal[ivert][iface][i] = 
					- vertex_normal[ivert][iface][i];
			}
		}
	}
	return true;
}

bool IVCONV::reverse_faces()
{
	for ( int iface = 0; iface < face_num; iface++ ) {
		
		int m = face_order[iface];
		
		for ( int ivert = 0; ivert < m/2; ivert++ ) {
			
			int jvert = m - ivert - 1;
			
			int itemp = face[ivert][iface];
			face[ivert][iface] = face[jvert][iface];
			face[jvert][iface] = itemp;
			
			itemp = vertex_material[ivert][iface];
			vertex_material[ivert][iface] = vertex_material[jvert][iface];
			vertex_material[jvert][iface] = itemp;
			
			for ( int i = 0; i < 3; ++i ) {
				float temp = vertex_normal[ivert][iface][i];
				vertex_normal[ivert][iface][i] = 
					vertex_normal[jvert][iface][i];
				vertex_normal[jvert][iface][i] = temp;
			}
		}
	}
	return true;
}

bool IVCONV::recompute_normals()
{
	for ( int iface = 0; iface < face_num; iface++ ) {
		for ( int i = 0; i < 3; ++i ) {
			face_normal[iface][i] = 0.0;
		}
	}
	
	for ( int iface = 0; iface < face_num; iface++ ) {
		for ( int ivert = 0; ivert < face_order[iface]; ivert++ ) {
			for ( int i = 0; i < 3; ++i ) {
				vertex_normal[ivert][iface][i] = 0.0;
			}
		}
	}
	
	vertex_normal_set ( );
	
	cor3_normal_set ( );
	
	face_normal_ave ( );
	return true;
}

bool IVCONV::scale(float x, float y, float z)
{
	for ( int j = 0; j < cor3_num; ++j ) {
		cor3[j][0] = x * cor3[j][0];
		cor3[j][1] = y * cor3[j][1];
		cor3[j][2] = z * cor3[j][2];
	}
	
	for ( int iface = 0; iface < face_num; iface++ ) {
		for ( int i = 0; i < 3; ++i ) {
			face_normal[iface][i] = 0.0;
		}
	}
	
	for ( int iface = 0; iface < face_num; iface++ ) {
		for ( int ivert = 0; ivert < face_order[iface]; ivert++ ) {
			for ( int i = 0; i < 3; ++i ) {
				vertex_normal[ivert][iface][i] = 0.0;
			}
		}
	}
	
	vertex_normal_set ( );
	
	cor3_normal_set ( );
	
	face_normal_ave ( );
	return true;
}

