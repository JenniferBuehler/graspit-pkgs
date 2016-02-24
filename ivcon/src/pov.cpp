#include "ivconv.h"

bool IVCONV::pov_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

POV_WRITE writes graphics information to a POV file.

Example:

// cone.pov created by IVCON.
// Original data in cone.iv

#version 3.0
#include "colors.inc"
#include "shapes.inc"
global_settings { assumed_gamma 2.2 }

camera {
right < 4/3, 0, 0>
up < 0, 1, 0 >
sky < 0, 1, 0 >
angle 20
location < 0, 0, -300 >
look_at < 0, 0, 0>
}

light_source { < 20, 50, -100 > color White }

background { color SkyBlue }

#declare RedText = texture {
pigment { color rgb < 0.8, 0.2, 0.2> }
finish { ambient 0.2 diffuse 0.5 }
}

#declare BlueText = texture {
pigment { color rgb < 0.2, 0.2, 0.8> }
finish { ambient 0.2 diffuse 0.5 }
}
mesh {
smooth_triangle {
< 0.29, -0.29, 0.0>, < 0.0, 0.0, -1.0 >,
< 38.85, 10.03, 0.0>, < 0.0, 0.0, -1.0 >,
< 40.21, -0.29, 0.0>, <  0.0, 0.0, -1.0 >
texture { RedText } }
...
smooth_triangle {
<  0.29, -0.29, 70.4142 >, < 0.0,  0.0, 1.0 >,
<  8.56,  -2.51, 70.4142 >, < 0.0,  0.0, 1.0 >,
<  8.85, -0.29, 70.4142 >, < 0.0,  0.0, 1.0 >
texture { BlueText } }
}

Modified:

08 October 1998

Author:

John Burkardt
*/
{
	int i;
	int j;
	int jj;
	int jlo;
	int k;
	int text_num;
	
	text_num = 0;
	fprintf ( fileout,  "// %s created by IVCON.\n", fileout_name );
	fprintf ( fileout,  "// Original data in %s.\n", filein_name );
	text_num = text_num + 2;
	/*
	Initial declarations.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "#version 3.0\n" );
	fprintf ( fileout, "#include \"colors.inc\"\n" );
	fprintf ( fileout, "#include \"shapes.inc\"\n" );
	fprintf ( fileout, "global_settings { assumed_gamma 2.2 }\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "camera {\n" );
	fprintf ( fileout, " right < 4/3, 0, 0>\n" );
	fprintf ( fileout, " up < 0, 1, 0 >\n" );
	fprintf ( fileout, " sky < 0, 1, 0 >\n" );
	fprintf ( fileout, " angle 20\n" );
	fprintf ( fileout, " location < 0, 0, -300 >\n" );
	fprintf ( fileout, " look_at < 0, 0, 0>\n" );
	fprintf ( fileout, "}\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "light_source { < 20, 50, -100 > color White }\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "background { color SkyBlue }\n" );
	
	text_num+=15;
	/*
	Declare RGB textures.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "#declare RedText = texture {\n" );
	fprintf ( fileout, "  pigment { color rgb < 0.8, 0.2, 0.2> }\n" );
	fprintf ( fileout, "  finish { ambient 0.2 diffuse 0.5 }\n" );
	fprintf ( fileout, "}\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "#declare GreenText = texture {\n" );
	fprintf ( fileout, "  pigment { color rgb < 0.2, 0.8, 0.2> }\n" );
	fprintf ( fileout, "  finish { ambient 0.2 diffuse 0.5 }\n" );
	fprintf ( fileout, "}\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "#declare BlueText = texture {\n" );
	fprintf ( fileout, "  pigment { color rgb < 0.2, 0.2, 0.8> }\n" );
	fprintf ( fileout, "  finish { ambient 0.2 diffuse 0.5 }\n" );
	fprintf ( fileout, "}\n" );
	/*
	Write one big object.
	*/
	fprintf ( fileout,  "mesh {\n" );
	++text_num;
	/*
	Do the next face.
	*/
	for ( i = 0; i < face_num; ++i ) {
	/*
	Break the face up into triangles, anchored at node 1.
		*/
		for ( jlo = 0; jlo < face_order[i] - 2; jlo++ ) {
			fprintf ( fileout, "  smooth_triangle {\n" );
			++text_num;
			
			for ( j = jlo; j < jlo + 3; ++j ) {
				
				if ( j == jlo ) {
					jj = 0;
				}
				else {
					jj = j;
				}
				
				k = face[jj][i];
				
				fprintf ( fileout, "<%f, %f, %f>, <%f, %f, %f>",
					cor3[k][0], cor3[k][1], cor3[k][2], 
					vertex_normal[jj][i][0], 
					vertex_normal[jj][i][1],
					vertex_normal[jj][i][2] );
				
				if ( j < jlo + 2 ) {
					fprintf ( fileout, ",\n" );
				}
				else {
					fprintf ( fileout, "\n" );
				}
				++text_num;
				
			}
			
			if (i%6 == 1 ) {
				fprintf ( fileout,  "texture { RedText } }\n" );
			}
			else if ( i%2 == 0 ) {
				fprintf ( fileout,  "texture { BlueText } }\n" );
			}
			else {
				fprintf ( fileout,  "texture { GreenText } }\n" );
			}
			++text_num;
			
		}
		
	}
	
	fprintf ( fileout,  "}\n" );
	++text_num;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "POV_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/******************************************************************************/

