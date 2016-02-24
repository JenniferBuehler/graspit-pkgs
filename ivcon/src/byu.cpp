#include "ivconv.h"


/**********************************************************************/

bool IVCONV::byu_read ( FILE *filein )

/**********************************************************************/

/*
Purpose:

BYU_READ reads graphics data from a Movie.BYU surface geometry file.

Discussion:

A Movie.BYU surface geometry file contains 4 groups of data.

The first group of data is a single line, containing 4 integers,
each one left justified in 8 columns.  The integers are:

PART_NUM, VERTEX_NUM, POLY_NUM, EDGE_NUM,

that is, the number of parts or objects, the number of vertices or nodes,
the number of polygons or faces, and the number of edges.

The second group of data is a single line, containing 2 integers,
each one left justified in 8 columnes.  The integers are:

POLY1, POLY2,

the starting and ending polygon numbers.  Presumably, this means
that the polygons are labeled POLY1, POLY1+1, ..., POLY2, comprising
a total of POLY_NUM polygons.

The third group is the X, Y and Z coordinates of all the vertices.
These may be written using a FORTRAN format of 6E12.5, which
crams two sets of (X,Y,Z) data onto each line, with each real value
written in an exponential format with 5 places after the decimal.
However, it is generally possible to write the XYZ coordinate data
for each vertex on a separate line.

The fourth group defines the polygons in terms of the vertex indices.
For each polygon, the vertices that make up the polygon are listed in
counterclockwise order.  The last vertex listed is given with a negative
sign to indicate the end of the list.  All the vertices for all the
polygons are listed one after the other, using a format that puts
up to 10 left-justified integers on a line, with each integer occupying
8 spaces.

This code will certainly read a BYU file created by BYU_WRITE, but
it will not handle more general files.  In particular, an object
can have several parts, the coordinate data can be grouped so
that there are 2 sets of (x,y,z) data per line, and so on.

Example:

1       8       6      24
1       6
0.00000E+00 0.00000E+00 0.00000E+00
1.00000E+00 0.00000E+00 0.00000E+00
1.00000E+00 2.00000E+00 0.00000E+00
0.00000E+00 2.00000E+00 0.00000E+00
0.00000E+00 0.00000E+00 1.00000E+00
1.00000E+00 0.00000E+00 1.00000E+00
1.00000E+00 2.00000E+00 1.00000E+00
0.00000E+00 2.00000E+00 1.00000E+00
4       3       2      -1
5       6       7      -8
1       5       8      -4
4       8       7      -3
3       7       6      -2
2       6       5      -1

Modified:

24 May 2001

Author:

John Burkardt

*/
{
	int cor3_num_new;
	int count;
	int edge_num;
	int face_num_new;
	int iface;
	int ival;
	int ivert;
	int j;
	char *next;
	int part_num;
	int poly1;
	int poly2;
	int text_num;
	int width;
	float x;
	float y;
	float z;
	
	text_num = 0;
	char input[LINE_MAX_LEN];
	if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) {
		return false;
	}
	++text_num;
	
	sscanf ( input, "%d %d %d %d", &part_num, &cor3_num_new, &face_num_new,
		&edge_num );
	
	if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) {
		return false;
	}
	++text_num;
	
	sscanf ( input, "%d %d", &poly1, &poly2 );
	
	for ( j = cor3_num; j < cor3_num + cor3_num_new; ++j ) {
		
		if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) {
			return false;
		}
		++text_num;
		
		sscanf ( input, "%f %f %f", &x, &y, &z );
		cor3[j][0] = x;
		cor3[j][1] = y;
		cor3[j][2] = z;
	}
	
	for ( iface = face_num; iface < face_num + face_num_new; iface++ ) {
		
		if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) {
			return false;
		}
		++text_num;
		
		next = input;
		ivert = 0;
		
		for (;;) {
			
			count = sscanf ( next, "%d%n", &ival, &width );
			next = next + width;
			
			if ( count <= 0 ) {
				return false;
			}
			
			if ( ival > 0 ) {
				face[ivert][iface] = ival - 1 + cor3_num;
			}
			else {
				face[ivert][iface] = - ival - 1 - cor3_num;
				break;
			}
			
			++ivert;
			
		}
		face_order[iface] = ivert + 1;
	}
	
	cor3_num = cor3_num + cor3_num_new;
	face_num = face_num + face_num_new;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "BYU_READ - Read %d text lines.\n", text_num );
	
	return true;
}
/**********************************************************************/

bool IVCONV::byu_write ( FILE *fileout )

/**********************************************************************/

/*
Purpose:

BYU_WRITE writes out the graphics data as a Movie.BYU surface geometry file.

Discussion:

A Movie.BYU surface geometry file contains 4 groups of data.

The first group of data is a single line, containing 4 integers,
each one left justified in 8 columns.  The integers are:

PART_NUM, VERTEX_NUM, POLY_NUM, EDGE_NUM,

that is, the number of parts or objects, the number of vertices or nodes,
the number of polygons or faces, and the number of edges.

The second group of data is a single line, containing 2 integers,
each one left justified in 8 columnes.  The integers are:

POLY1, POLY2,

the starting and ending polygon numbers.  Presumably, this means
that the polygons are labeled POLY1, POLY1+1, ..., POLY2, comprising
a total of POLY_NUM polygons.

The third group is the X, Y and Z coordinates of all the vertices.
These may be written using a FORTRAN format of 6E12.5, which
crams two sets of (X,Y,Z) data onto each line, with each real value
written in an exponential format with 5 places after the decimal.
However, it is generally possible to write the XYZ coordinate data
for each vertex on a separate line.

The fourth group defines the polygons in terms of the vertex indices.
For each polygon, the vertices that make up the polygon are listed in
counterclockwise order.  The last vertex listed is given with a negative
sign to indicate the end of the list.  All the vertices for all the
polygons are listed one after the other, using a format that puts
up to 10 left-justified integers on a line, with each integer occupying
8 spaces.

Example:

1       8       6      24
1       6
0.00000E+00 0.00000E+00 0.00000E+00
1.00000E+00 0.00000E+00 0.00000E+00
1.00000E+00 2.00000E+00 0.00000E+00
0.00000E+00 2.00000E+00 0.00000E+00
0.00000E+00 0.00000E+00 1.00000E+00
1.00000E+00 0.00000E+00 1.00000E+00
1.00000E+00 2.00000E+00 1.00000E+00
0.00000E+00 2.00000E+00 1.00000E+00
4       3       2      -1
5       6       7      -8
1       5       8      -4
4       8       7      -3
3       7       6      -2
2       6       5      -1

Modified:

24 May 2001

Author:

John Burkardt
*/
{
	int edge_num;
	int iface;
	int ivert;
	int j;
	int jp;
	int part_num;
	int text_num;
	
	text_num = 0;
	
	edge_num = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		edge_num = edge_num + face_order[iface];
	}
	
	part_num = 1;
	
	fprintf ( fileout, "%d %li %li %d\n", part_num, cor3_num, face_num, edge_num );
	++text_num;
	
	fprintf ( fileout, "1 %li\n", face_num );
	++text_num;
	
	for ( j = 0; j < cor3_num; ++j ) {
		fprintf ( fileout, "%f %f %f\n", cor3[j][0], cor3[j][1], cor3[j][2] );
		++text_num;
	}
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			jp = face[ivert][iface] + 1;
			if ( ivert == face_order[iface] - 1 ) {
				jp = - jp;
			}
			fprintf ( fileout, "%d ", jp );
		}
		fprintf ( fileout, "\n" );
		++text_num;
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "BYU_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
