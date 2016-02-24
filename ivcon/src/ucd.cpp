#include "ivconv.h"

bool IVCONV::ucd_write ( FILE *fileout )

/**********************************************************************/

/*
Purpose:

UCD_WRITE writes graphics data to an AVS UCD file.

Examples:

#  cube.ucd created by IVREAD.
#
#  Material RGB to hue map:
#
#  material    R    G  B   Alpha     Hue
#
#    0   0.94  0.70  0.15  1.000   0.116
#    1   0.24  0.70  0.85  1.000   0.541
#    2   0.24  0.00  0.85  1.000   0.666
#
#  The node data is
#    node # / material # / RGBA / Hue
#
8  6  6  0  0
0  0.0  0.0  0.0
1  1.0  0.0  0.0
2  1.0  1.0  0.0
3  0.0  1.0  0.0
4  0.0  0.0  1.0
5  1.0  0.0  1.0
6  1.0  1.0  1.0
7  0.0  1.0  1.0
0  0  quad  0  1  2  3
1  0  quad  0  4  5  1
2  0  quad  1  5  6  2
3  0  quad  2  6  7  3
4  0  quad  3  7  4  0
5  0  quad  4  7  6  5
3  1 4 1
material, 0...2
RGBA, 0-1/0-1/0-1/0-1
Hue, 0-1
0  0  0.94  0.70  0.15  1.0  0.116
1  0  0.94  0.70  0.15  1.0  0.116
2  0  0.94  0.70  0.15  1.0  0.116
3  0  0.94  0.70  0.15  1.0  0.116
4  1  0.24  0.70  0.85  1.0  0.541
5  1  0.24  0.70  0.85  1.0  0.541
6  2  0.24  0.24  0.85  0.0  0.666
7  2  0.24  0.24  0.85  0.0  0.666

Modified:

22 May 1999

Author:

John Burkardt

*/
{
	float a;
	float b;
	float g;
	float h;
	int i;
	int imat;
	int j;
	int text_num;
	float r;
	
	text_num = 0;
	
	fprintf ( fileout, "#  %s created by IVREAD.\n", fileout_name );
	fprintf ( fileout, "#\n" );
	fprintf ( fileout, "#  Material RGB to Hue map:\n" );
	fprintf ( fileout, "#\n" );
	fprintf ( fileout, "#  material    R    G      B     Alpha  Hue\n" );
	fprintf ( fileout, "#\n" );
	
	text_num = text_num + 6;
	
	for ( j = 0; j < material_num; ++j ) {
		r = material[j].rgb[0];
		g = material[j].rgb[1];
		b = material[j].rgb[2];
		a = material[j].alpha;
		h = rgb_to_hue ( r, g, b );
		fprintf ( fileout, "#  %d %f %f %f %f %f\n", j, r, g, b, a, h );
		++text_num;
	}
	
	fprintf ( fileout, "#\n" );
	fprintf ( fileout, "#  The node data is\n" );
	fprintf ( fileout, "#    node # / material # / RGBA / Hue\n" );
	fprintf ( fileout, "#\n" );
	text_num = text_num + 4;
	
	fprintf ( fileout, "%li %li 6 0 0\n", cor3_num, face_num );
	++text_num;
	
	for ( j = 0; j < cor3_num; ++j ) {
		fprintf ( fileout, "%d %f %f %f\n", j, cor3[j][0], cor3[j][1],
			cor3[j][2] );
		++text_num;
	}
	/*
	NOTE:
	UCD only accepts triangles and quadrilaterals, not higher order
	polygons.  We would need to break polygons up to proceed.
	*/
	for ( j = 0; j < face_num; ++j ) {
		
		fprintf ( fileout, "%d %d", j, face_material[j] );
		
		if ( face_order[j] == 3 ) {
			fprintf ( fileout, " tri" );
		}
		else if ( face_order[j] == 4 ) {
			fprintf ( fileout, " quad" );
		}
		else {
			fprintf ( fileout, " ???" );
		}
		
		for ( i = 0; i < face_order[j]; ++i ) {
			fprintf ( fileout, "%li", face[i][j] );
		}
		fprintf ( fileout, "\n" );
		++text_num;
		
	}
	
	fprintf ( fileout, "3  1  4  1\n" );
	fprintf ( fileout, "material, 0...%d\n", material_num - 1 );
	fprintf ( fileout, "RGBA, 0-1/0-1/0-1/0-1\n" );
	fprintf ( fileout, "Hue, 0-1\n" );
	text_num = text_num + 4;
	
	for ( j = 0; j < cor3_num; ++j ) {
		imat = cor3_material[j];
		r = material[imat].rgb[0];
		g = material[imat].rgb[1];
		b = material[imat].rgb[2];
		a = material[imat].alpha;
		h = rgb_to_hue ( r, g, b );
		
		fprintf ( fileout, "%d %d %f %f %f %f %f\n", j, imat, r, g, b, a, h );
		++text_num;
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "UCD_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/******************************************************************************/

