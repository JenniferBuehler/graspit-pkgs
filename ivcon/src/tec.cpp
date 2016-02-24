#include "ivconv.h"

/**********************************************************************/

bool IVCONV::tec_write ( FILE *fileout )

/**********************************************************************/

/*
Purpose:

TEC_WRITE writes graphics information to a TECPLOT file.

Discussion:

The file format used is appropriate for 3D finite element surface 
zone data.  Polygons are decomposed into triangles where necessary.

Example:

TITLE = "cube.tec created by IVCON."
VARIABLES = "X", "Y", "Z", "R", "G", "B"
ZONE T="TRIANGLES", N=8, E=12, F=FEPOINT, ET=TRIANGLE
0.0 0.0 0.0 0.0 0.0 0.0
1.0 0.0 0.0 1.0 0.0 0.0
1.0 1.0 0.0 1.0 1.0 0.0
0.0 1.0 0.0 0.0 1.0 0.0
0.0 0.0 1.0 0.0 0.0 1.0
1.0 0.0 1.0 1.0 0.0 1.0
1.0 1.0 1.0 1.0 1.0 1.0
0.0 1.0 1.0 0.0 1.0 1.0
1 4 2
2 4 3
1 5 8
1 2 5
2 6 5
2 3 6
3 7 6
3 4 7
4 8 7
4 1 8
5 6 8
6 7 8

Modified:

09 June 1999

Author:

John Burkardt
*/
{
	float b;
	int face2[3];
	float g;
	int icor3;
	int iface;
	int imat;
	int j;
	int face_num2;
	int text_num;
	float r;
	/*
	Determine the number of triangular faces.
	*/
	face_num2 = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( j = 0; j < face_order[iface] - 2; ++j ) {
			face_num2 = face_num2 + 1;
		}
	}
	
	text_num = 0;
	
	fprintf ( fileout, "\"%s created by IVCON.\"\n", fileout_name );
	fprintf ( fileout, "VARIABLES = \"X\", \"Y\", \"Z\", \"R\", \"G\", \"B\"\n" );
	fprintf ( fileout, 
		"ZONE T=\"TRIANGLES\", N=%d, E=%d, F=FEPOINT, ET=TRIANGLE\n",
		cor3_num, face_num2 );
	
	text_num = text_num + 3;
	/*
	Write out X, Y, Z, R, G, B per node.
	*/
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		imat = cor3_material[icor3];
		r = material[imat].ambient[0];
		g = material[imat].ambient[1];
		b = material[imat].ambient[2];
		fprintf ( fileout, "%f %f %f %f %f %f\n", cor3[icor3][0], cor3[icor3][1], cor3[icor3][2], r, g, b );
		++text_num;
	}
	/*
	Do the next face.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
	/*
	Break the face up into triangles, anchored at node 1.
		*/
		for ( j = 0; j < face_order[iface] - 2; ++j ) {
			
			face2[0] = face[  0][iface] + 1;
			face2[1] = face[j+1][iface] + 1;
			face2[2] = face[j+2][iface] + 1;
			
			fprintf ( fileout, "%d %d %d\n", face2[0], face2[1], face2[2] );
			++text_num;
			
		}
		
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "TEC_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}

/*********************************************************************/

