#include "ivconv.h"

bool IVCONV::hrc_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

HRC_READ reads graphics information from a SoftImage HRC file.

Examples:

HRCH: Softimage 4D Creative Environment v3.00


model
{
name         "cube_10x10"
scaling      1.000 1.000 1.000
rotation     0.000 0.000 0.000
translation  0.000 0.000 0.000

mesh
{
flag    ( PROCESS )
discontinuity  60.000

vertices   8
{
[0] position  -5.000  -5.000  -5.000
[1] position  -5.000  -5.000  5.000
[2] position  -5.000  5.000  -5.000
[3] position  -5.000  5.000  5.000
[4] position  5.000  -5.000  -5.000
[5] position  5.000  -5.000  5.000
[6] position  5.000  5.000  -5.000
[7] position  5.000  5.000  5.000
}

polygons   6
{
[0] nodes  4
{
[0] vertex  0
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  1
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[2] vertex  3
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[3] vertex  2
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
}
material  0
[1] nodes  4
{
[0] vertex  1
normal  0.000  0.000  1.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  5

...etc.....

[5] nodes  4
{
[0] vertex  2
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  3
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[2] vertex  7
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[3] vertex  6
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
}
material  0
}

edges   12
{
[1] vertices  3  2
[2] vertices  2  0
[3] vertices  0  1
[4] vertices  1  3
[5] vertices  7  3
[6] vertices  1  5
[7] vertices  5  7
[8] vertices  6  7
[9] vertices  5  4
[10] vertices  4  6
[11] vertices  2  6
[12] vertices  4  0
}
}

material [0]
{
name           "kazoo"
type           PHONG
ambient        0.0  1.0  0.0
diffuse        1.0  0.0  0.0
specular       0.0  0.0  1.0
exponent      50.0
reflectivity   0.0
transparency   0.0
refracIndex    1.0
glow           0
coc            0.0
}

texture [0]
{
name          "/usr/users/foss/HOUSE/PICTURES/mellon"
glbname       "t2d1"
anim          STATIC
method        XY
repeat        1  1
scaling       1.000  1.000
offset        0.000  0.000
pixelInterp
effect        INTENSITY
blending      1.000
ambient       0.977
diffuse       1.000
specular      0.966
reflect       0.000
transp        0.000
roughness     0.000
reflMap       1.000
rotation      0.000
txtsup_rot    0.000  0.000  0.000
txtsup_trans  0.000  0.000  0.000
txtsup_scal   1.000  1.000  1.000
}
}

Modified:

25 June 1999

Author:

John Burkardt
*/
{
	float b;
	int   count;
	float g;
	int   i;
	int   icor3;
	int   ivert;
	int   iword;
	int   jval;
	int   level;
	char *next;
	int   nlbrack;
	int   nrbrack;
	int   cor3_num_old;
	float r;
	float t;
	float temp[3];
	int   width;
	char  word[LINE_MAX_LEN];
	char  word1[LINE_MAX_LEN];
	char  word2[LINE_MAX_LEN];
	char  wordm1[LINE_MAX_LEN];
	float x;
	float y;
	float z;
	
	level = 0;
	strcpy ( level_name[0], "Top" );
	nlbrack = 0;
	nrbrack = 0;
	cor3_num_old = cor3_num;
	strcpy ( word, " " );
	strcpy ( wordm1, " " );
	/*
	Read a line of text from the file.
	*/
	for ( ;; ) {
		char input[LINE_MAX_LEN];

		if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) {
			break;
		}
		
		++text_num;
		next = input;
		iword = 0;
		/*
		Read a word from the line.
		*/
		for ( ;; ) {
			
			strcpy ( wordm1, word );
			
			count = sscanf ( next, "%s%n", word2, &width );
			next += width;
			
			if ( count <= 0 ) {
				break;
			}
			
			strcpy ( word, word2 );
			
			++iword;
			
			if ( iword == 1 ) {
				strcpy ( word1, word );
			}
			/*
			The first line of the file must be the header.
			*/
			if ( text_num == 1 ) {
				
				if ( strcmp ( word1, "HRCH:" ) != 0 ) {
					printf ( "\n" );
					printf ( "HRC_READ - Fatal false!\n" );
					printf ( "  The input file has a bad header.\n" );
					return false;
				}
				else {
					++comment_num;
				}
				break;
			}
			/*
			If the word is a curly bracket, count it.
			*/
			if ( strcmp ( word, "{" ) == 0 ) {
				++nlbrack;
				level = nlbrack - nrbrack;
				strcpy ( level_name[level], wordm1 );
				if ( debug ) {
					printf ( "New level: %s\n", level_name[level] );
				}
			}
			else if ( strcmp ( word, "}" ) == 0 ) {
				++nrbrack;
				
				if ( nlbrack < nrbrack ) {
					printf ( "\n" );
					printf ( "HRC_READ - Fatal false!\n" );
					printf ( "  Extraneous right bracket on line %d.\n", text_num );
					printf ( "  Currently processing field %s\n.", level_name[level] );
					return false;
				}
			}
			/*
			CONTROLPOINTS
			*/
			if ( strcmp ( level_name[level], "controlpoints" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = -1;
						line_material[line_num] = 0;
					}
					++line_num;
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "position" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &x, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next += width;
					
					temp[0] = x;
					temp[1] = y;
					temp[2] = z;
					
					if ( cor3_num < 1000 ) {
						icor3 = rcol_find ( cor3, cor3_num, temp );
					}
					else {
						icor3 = -1;
					}
					
					if ( icor3 == -1 ) {
						
						icor3 = cor3_num;
//						if ( cor3_num < COR3_MAX ) 
						{
							cor3[cor3_num]=vec3(x,y,z);
						}
						++cor3_num;
						
					}
					else {
						++dup_num;
					}
					
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = icor3;
						line_material[line_num] = 0;
					}
					++line_num;
				}
				else {
					++bad_num;
					printf ( "CONTROLPOINTS: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			EDGES
			*/
			else if ( strcmp ( level_name[level], "edges" ) == 0 ) {
				
				if ( strcmp( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "vertices" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = jval + cor3_num_old;
						line_material[line_num] = 0;
					}
					++line_num;
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = jval + cor3_num_old;
						line_material[line_num] = 0;
					}
					++line_num;
					
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = -1;
						line_material[line_num] = -1;
					}
					++line_num;
					
				}
				else {
					++bad_num;
					printf ( "EDGES: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			MATERIAL
			*/
			else if ( strcmp ( level_name[level], "material" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					++material_num;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "ambient" ) == 0 ) {
					count = sscanf ( next, "%f%n", &r, &width ); next += width;
					count = sscanf ( next, "%f%n", &g, &width ); next += width;
					count = sscanf ( next, "%f%n", &b, &width ); next += width;
					material[material_num-1].ambient=vec3(r,g,b);				
				}
				else if ( strcmp ( word, "coc" ) == 0 ) {
				}
				else if ( strcmp ( word, "diffuse" ) == 0 ) {					
					count = sscanf ( next, "%f%n", &r, &width ); next += width;
					count = sscanf ( next, "%f%n", &g, &width ); next += width;
					count = sscanf ( next, "%f%n", &b, &width ); next += width;
					material[material_num-1].rgb=vec3(r,g,b);				
				}
				else if ( strcmp ( word, "exponent" ) == 0 ) {
				}
				else if ( strcmp ( word, "glow" ) == 0 ) {
				}
				else if ( strcmp ( word, "name" ) == 0 ) {
					count = sscanf ( next, "%s%n", word, &width );
					next += width;
					strcpy ( material[material_num-1].name, word );
				}
				else if ( strcmp ( word, "reflectivity" ) == 0 ) {
				}
				else if ( strcmp ( word, "refracindex" ) == 0 ) {
				}
				else if ( strcmp ( word, "specular" ) == 0 ) {
					count = sscanf ( next, "%f%n", &r, &width ); next += width;
					count = sscanf ( next, "%f%n", &g, &width ); next += width;
					count = sscanf ( next, "%f%n", &b, &width ); next += width;
					material[material_num-1].specular=vec3(r,g,b);				
				}
				else if ( strcmp ( word, "transparency" ) == 0 ) {
					count = sscanf ( next, "%f%n", &t, &width );
					next += width;
					material[material_num-1].alpha = 1.0 - t;
				}
				else if ( strcmp ( word, "type" ) == 0 ) {
				}
				else {
					++bad_num;
					printf ( "MATERIAL: Bad data %s\n", word );
					return false;
				}
			}
			/*
			MESH
			*/
			else if ( strcmp ( level_name[level], "mesh" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( strcmp ( word, "discontinuity" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "edges" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "flag" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "polygons" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "vertices" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "MESH: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			MODEL
			*/
			else if ( strcmp ( level_name[level], "model" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( strcmp ( word, "material" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "mesh" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "name" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "patch" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "rotation" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "scaling" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "spline" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "translation" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "MODEL: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			NODES
			*/
			else if ( strcmp ( level_name[level], "nodes" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					ivert = 0;
					face_order[face_num] = 0;
					++face_num;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "normal" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &x, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next += width;
					
					// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
					{
						vertex_normal[ivert-1][face_num-1] = vec3(x,y,z);
					}
					
				}
				else if ( strcmp ( word, "uvTexture" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &x, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next += width;
					
					// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
					{
						vertex_tex_uv[face_num-1][ivert-1] = vec2(x,y);
					}
				}
				else if ( strcmp ( word, "vertex" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
					{
						++face_order[face_num-1];
						face[ivert][face_num-1] = jval;
					}
					++ivert;
					
				}
				/*
				Right now, we don't do anything with the vertexColor information.
				*/
				else if ( strcmp ( word, "vertexColor" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
				}
				else {
					++bad_num;
					printf ( "NODES: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			PATCH
			I don't know what to do with this yet.
			*/
			else if ( strcmp ( level_name[level], "patch" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( strcmp ( word, "approx_type" ) == 0 ) {
				}
				else if ( strcmp ( word, "controlpoints" ) == 0 ) {
				}
				else if ( strcmp ( word, "curv_u" ) == 0 ) {
				}
				else if ( strcmp ( word, "curv_v" ) == 0 ) {
				}
				else if ( strcmp ( word, "recmin" ) == 0 ) {
				}
				else if ( strcmp ( word, "recmax" ) == 0 ) {
				}
				else if ( strcmp ( word, "recursion" ) == 0 ) {
				}
				else if ( strcmp ( word, "spacial" ) == 0 ) {
				}
				else if ( strcmp ( word, "taggedpoints" ) == 0 ) {
				}
				else if ( strcmp ( word, "ucurve" ) == 0 ) {
				}
				else if ( strcmp ( word, "ustep" ) == 0 ) {
				}
				else if ( strcmp ( word, "utension" ) == 0 ) {
				}
				else if ( strcmp ( word, "utype" ) == 0 ) {
				}
				else if ( strcmp ( word, "vclose" ) == 0 ) {
				}
				else if ( strcmp ( word, "vcurve" ) == 0 ) {
				}
				else if ( strcmp ( word, "viewdep" ) == 0 ) {
				}
				else if ( strcmp ( word, "vpoint" ) == 0 ) {
				}
				else if ( strcmp ( word, "vstep" ) == 0 ) {
				}
				else if ( strcmp ( word, "vtension" ) == 0 ) {
				}
				else if ( strcmp ( word, "vtype" ) == 0 ) {
				}
				else {
					++bad_num;
					printf ( "PATCH: Bad data %s\n", word );
					return false;
				}
			}
			/*
			POLYGONS
			*/
			else if ( strcmp ( level_name[level], "polygons" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "material" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &jval, &width );
					next += width;
					
					for ( ivert = 0; ivert < ORDER_MAX; ivert++ ) {
						vertex_material[ivert][face_num-1] = jval;
					}
					
				}
				else if ( strcmp ( word, "nodes" ) == 0 ) {
					count = sscanf ( next, "%s%n", word2, &width );
					next += width;
				}
				else {
					++bad_num;
					printf ( "POLYGONS: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			SPLINE
			*/
			else if ( strcmp ( level_name[level], "spline" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( strcmp ( word, "controlpoints" ) == 0 ) {
					break;
				}
				/*
				WHY DON'T YOU READ IN THE OBJECT NAME HERE?
				*/
				else if ( strcmp ( word, "name" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "nbKeys" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "step" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "tension" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "type" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "SPLINE: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			TAGGEDPOINTS
			*/
			else if ( strcmp ( level_name[level], "taggedpoints" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "tagged" ) == 0 ) {
				}
				else {
					++bad_num;
					printf ( "TAGGEDPOINTS: Bad data %s\n", word );
					return false;
				}
				
			}
			/*
			TEXTURE
			*/
			else if ( strcmp ( level_name[level], "texture" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					++texture_num;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "ambient" ) == 0 ) {
				}
				else if ( strcmp ( word, "anim" ) == 0 ) {
				}
				else if ( strcmp ( word, "blending" ) == 0 ) {
				}
				else if ( strcmp ( word, "diffuse" ) == 0 ) {
				}
				else if ( strcmp ( word, "effect" ) == 0 ) {
				}
				else if ( strcmp ( word, "glbname" ) == 0 ) {
				}
				else if ( strcmp ( word, "method" ) == 0 ) {
				}
				else if ( strcmp ( word, "name" ) == 0 ) {
					count = sscanf ( next, "%s%n", word, &width );
					next += width;
					strcpy ( texture_name[texture_num-1], word );
				}
				else if ( strcmp ( word, "offset" ) == 0 ) {
				}
				else if ( strcmp ( word, "pixelinterp" ) == 0 ) {
				}
				else if ( strcmp ( word, "reflect" ) == 0 ) {
				}
				else if ( strcmp ( word, "reflmap" ) == 0 ) {
				}
				else if ( strcmp ( word, "repeat" ) == 0 ) {
				}
				else if ( strcmp ( word, "rotation" ) == 0 ) {
				}
				else if ( strcmp ( word, "roughness" ) == 0 ) {
				}
				else if ( strcmp ( word, "scaling" ) == 0 ) {
				}
				else if ( strcmp ( word, "specular" ) == 0 ) {
				}
				else if ( strcmp ( word, "transp" ) == 0 ) {
				}
				else if ( strcmp ( word, "txtsup_rot" ) == 0 ) {
				}
				else if ( strcmp ( word, "txtsup_scal" ) == 0 ) {
				}
				else if ( strcmp ( word, "txtsup_trans" ) == 0 ) {
				}
				else {
					++bad_num;
					printf ( "TEXTURE: Bad data %s\n", word );
					return false;
				}
			}
			/*
			VERTICES
			*/
			else if ( strcmp ( level_name[level], "vertices" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
				}
				else if ( word[0] == '[' ) {
				}
				else if ( strcmp ( word, "position" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &x, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next += width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next += width;
					
//					if ( cor3_num < COR3_MAX ) 
					{
						cor3[cor3_num] = vec3(x,y,z);
					}
					++cor3_num;
				}
				else {
					++bad_num;
					printf ( "VERTICES: Bad data %s\n", word );
					return false;
				}
			}
			/*
			Any other word:
			*/
			else {
				
			}
    }
  }
  
  /*
  End of information in file.
  
	Check the "materials" defining a line.
	
	  If COORDINDEX is -1, so should be the MATERIALINDEX.
	  If COORDINDEX is not -1, then the MATERIALINDEX shouldn"t be either.
	  */
	  for ( i = 0; i < line_num; ++i ) {
		  
		  if ( line_dex[i] == -1 ) {
			  line_material[i] = -1;
		  }
		  else if ( line_material[i] == -1 ) {
			  line_material[i] = 0;
		  }
		  
	  }
	  return true;
}
/******************************************************************************/

bool IVCONV::hrc_write ( FILE* fileout )

/******************************************************************************/

/*
Purpose:

HRC_WRITE writes graphics data to an HRC SoftImage file.

Examples:

HRCH: Softimage 4D Creative Environment v3.00


model
{
name         "cube_10x10"
scaling      1.000 1.000 1.000
rotation     0.000 0.000 0.000
translation  0.000 0.000 0.000

mesh
{
flag    ( PROCESS )
discontinuity  60.000

vertices   8
{
[0] position  -5.000  -5.000  -5.000
[1] position  -5.000  -5.000  5.000
[2] position  -5.000  5.000  -5.000
[3] position  -5.000  5.000  5.000
[4] position  5.000  -5.000  -5.000
[5] position  5.000  -5.000  5.000
[6] position  5.000  5.000  -5.000
[7] position  5.000  5.000  5.000
}

polygons   6
{
[0] nodes  4
{
[0] vertex  0
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  1
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[2] vertex  3
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[3] vertex  2
normal  -1.000  0.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
}
material  0
[1] nodes  4
{
[0] vertex  1
normal  0.000  0.000  1.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  5

...etc.....

[5] nodes  4
{
[0] vertex  2
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[1] vertex  3
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[2] vertex  7
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
[3] vertex  6
normal  0.000  1.000  0.000
uvTexture  0.000  0.000
vertexColor 255 178 178 178
}
material  0
}

edges   12
{
[1] vertices  3  2
[2] vertices  2  0
[3] vertices  0  1
[4] vertices  1  3
[5] vertices  7  3
[6] vertices  1  5
[7] vertices  5  7
[8] vertices  6  7
[9] vertices  5  4
[10] vertices  4  6
[11] vertices  2  6
[12] vertices  4  0
}
}

material [0]
{
name           "kazoo"
type           PHONG
ambient        0.0  1.0  0.0
diffuse        1.0  0.0  0.0
specular       0.0  0.0  1.0
exponent      50.0
reflectivity   0.0
transparency   0.0
refracIndex    1.0
glow           0
coc            0.0
}

texture [0]
{
name          "/usr/users/foss/HOUSE/PICTURES/mellon"
glbname       "t2d1"
anim          STATIC
method        XY
repeat        1  1
scaling       1.000  1.000
offset        0.000  0.000
pixelInterp
effect        INTENSITY
blending      1.000
ambient       0.977
diffuse       1.000
specular      0.966
reflect       0.000
transp        0.000
roughness     0.000
reflMap       1.000
rotation      0.000
txtsup_rot    0.000  0.000  0.000
txtsup_trans  0.000  0.000  0.000
txtsup_scal   1.000  1.000  1.000
}
}

Modified:

25 June 1998

Author:

John Burkardt

*/
{
	int iface;
	int ivert;
	int j;
	int jhi;
	int jlo;
	int jrel;
	int k;
	int npts;
	int nseg;
	int text_num;
	
	nseg = 0;
	text_num = 0;
	
	fprintf ( fileout, "HRCH: Softimage 4D Creative Environment v3.00\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "\n" );
	text_num = text_num + 3;
	
	fprintf ( fileout, "model\n" );
	fprintf ( fileout, "{\n" );
//	fprintf ( fileout, "  name         \"%s\"\n", object_name );
	fprintf ( fileout, "  scaling      1.000 1.000 1.000\n" );
	fprintf ( fileout, "  rotation     0.000 0.000 0.000\n" );
	fprintf ( fileout, "  translation  0.000 0.000 0.000\n" );
	text_num = text_num + 6;
	
	if ( face_num > 0 ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  mesh\n" );
		fprintf ( fileout, "  {\n" );
		fprintf ( fileout, "    flag    ( PROCESS )\n" );
		fprintf ( fileout, "    discontinuity  60.000\n" );
		text_num = text_num + 5;
		/*
		Point coordinates.
		*/
		if ( cor3_num > 0 ) {
			
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "    vertices %li\n", cor3_num );
			fprintf ( fileout, "    {\n" );
			text_num = text_num + 3;
			
			for ( j = 0; j < cor3_num; ++j ) {
				
				fprintf ( fileout, "      [%d] position %f %f %f\n", j, cor3[j][0], 
					cor3[j][1], cor3[j][2] );
				++text_num;
			}
			fprintf ( fileout, "    }\n" );
			++text_num;
		}
		/*
		Faces.
		*/
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "    polygons %li\n", face_num );
		fprintf ( fileout, "    {\n" );
		text_num = text_num + 3;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			
			fprintf ( fileout, "      [%i] nodes %d\n", iface, face_order[iface] );
			fprintf ( fileout, "      {\n" );
			text_num = text_num + 2;
			
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				
				fprintf ( fileout, "        [%i] vertex %li\n", ivert, face[ivert][iface] );
				fprintf ( fileout, "            normal %f %f %f\n", 
					vertex_normal[ivert][iface][0], 
					vertex_normal[ivert][iface][1], vertex_normal[ivert][iface][2] );
				fprintf ( fileout, "            uvTexture  %f %f\n", 
					vertex_tex_uv[iface][ivert][0], vertex_tex_uv[iface][ivert][1] );
				fprintf ( fileout, "            vertexColor  255 178 178 178\n" );
				text_num = text_num + 4;
			}
			fprintf ( fileout, "      }\n" );
			fprintf ( fileout, "      material %d\n", face_material[iface] );
			text_num = text_num + 2;
		}
		fprintf ( fileout, "    }\n" );
		fprintf ( fileout, "  }\n" );
		text_num = text_num + 2;
	}
	/*
	IndexedLineSet.
	*/
	if ( line_num > 0 ) {
		
		nseg = 0;
		
		jhi = -1;
		
		for ( ;; ) {
			
			jlo = jhi + 1;
			/*
			Look for the next index JLO that is not -1.
			*/
			while ( jlo < line_num ) {
				if ( line_dex[jlo] != -1 ) {
					break;
				}
				++jlo;
			}
			
			if ( jlo >= line_num ) {
				break;
			}
			/*
			Look for the highest following index JHI that is not -1.
			*/
			jhi = jlo + 1;
			
			while ( jhi < line_num ) {
				if ( line_dex[jhi] == -1 ) {
					break;
				}
				++jhi;
			}
			
			jhi = jhi - 1;
			/*
			Our next line segment involves LINE_DEX indices JLO through JHI.
			*/     
			++nseg;
			npts = jhi + 1 - jlo;
			
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "  spline\n" );
			fprintf ( fileout, "  {\n" );
			fprintf ( fileout, "    name     \"spl%d\"\n", nseg );
			fprintf ( fileout, "    type     LINEAR\n" );
			fprintf ( fileout, "    nbKeys   %d\n", npts );
			fprintf ( fileout, "    tension  0.000\n" );
			fprintf ( fileout, "    step     1\n" );
			fprintf ( fileout, "\n" );
			text_num = text_num + 9;
			
			fprintf ( fileout, "    controlpoints\n" );
			fprintf ( fileout, "    {\n" );
			text_num = text_num + 2;
			
			for ( j = jlo; j <= jhi; ++j ) {
				jrel = j - jlo;
				k = line_dex[j];
				fprintf ( fileout, "      [%d] position %f %f %f\n", jrel,
					cor3[k][0], cor3[k][1], cor3[k][2] );
				++text_num;
			}
			
			fprintf ( fileout, "    }\n" );
			fprintf ( fileout, "  }\n" );
			text_num = text_num + 2;
		}
	}
	/*
	MATERIALS
	*/
	for ( int i = 0; i < material_num; ++i ) {
		
		fprintf ( fileout, "  material [%d]\n", i );
		fprintf ( fileout, "  {\n" );
		fprintf ( fileout, "    name           \"%s\"\n", material[i].name );
		fprintf ( fileout, "    type           PHONG\n" );
		fprintf ( fileout, "    ambient        %f %f %f\n", material[i].ambient[0],material[i].ambient[1],material[i].ambient[2]);
		fprintf ( fileout, "    diffuse        %f %f %f\n", material[i].rgb[0],material[i].rgb[1],material[i].rgb[2]);
		fprintf ( fileout, "    specular       %f %f %f\n", material[i].specular[0],material[i].specular[1],material[i].specular[2]);
		fprintf ( fileout, "    exponent      50.0\n" );
		fprintf ( fileout, "    reflectivity   0.0\n" );
		fprintf ( fileout, "    transparency   %f\n", 1.0 - material[i].alpha );
		fprintf ( fileout, "    refracIndex    1.0\n" );
		fprintf ( fileout, "    glow           0\n" );
		fprintf ( fileout, "    coc            0.0\n" );
		fprintf ( fileout, "  }\n" );
		
		text_num+=14;
		
	}
	/*
	TEXTURES
	*/
	for ( int i = 0; i < texture_num; ++i ) {
		
		fprintf ( fileout, "  texture [%d]\n", i );
		fprintf ( fileout, "  {\n" );
		fprintf ( fileout, "    name           \"%s\"\n", texture_name[i] );
		fprintf ( fileout, "    glbname        \"t2d1\"\n" );
		fprintf ( fileout, "    anim           STATIC\n" );
		fprintf ( fileout, "    method         XY\n" );
		fprintf ( fileout, "    repeat         1 1\n" );
		fprintf ( fileout, "    scaling        1.000  1.000\n" );
		fprintf ( fileout, "    offset         0.000  0.000\n" );
		fprintf ( fileout, "    pixelInterp\n" );
		fprintf ( fileout, "    effect         INTENSITY\n" );
		fprintf ( fileout, "    blending       1.000\n" );
		fprintf ( fileout, "    ambient        0.977\n" );
		fprintf ( fileout, "    diffuse        1.000\n" );
		fprintf ( fileout, "    specular       0.966\n" );
		fprintf ( fileout, "    reflect        0.000\n" );
		fprintf ( fileout, "    transp         0.000\n" );
		fprintf ( fileout, "    roughness      0.000\n" );
		fprintf ( fileout, "    reflMap        1.000\n" );
		fprintf ( fileout, "    rotation       0.000\n" );
		fprintf ( fileout, "    txtsup_rot     0.000  0.000  0.000\n" );
		fprintf ( fileout, "    txtsup_trans   0.000  0.000  0.000\n" );
		fprintf ( fileout, "    txtsup_scal    1.000  1.000  1.000\n" );
		fprintf ( fileout, "  }\n" );
		
		text_num = text_num + 25;
		
	}
	fprintf ( fileout, "}\n" );
	++text_num;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "HRC_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/******************************************************************************/
