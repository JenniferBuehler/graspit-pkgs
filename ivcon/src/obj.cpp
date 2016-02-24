#include "ivconv.h"

bool IVCONV::obj_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

OBJ_READ reads a Wavefront OBJ file.

Example:

#  magnolia.obj

mtllib ./vp.mtl

g
v -3.269770 -39.572201 0.876128
v -3.263720 -39.507999 2.160890
...
v 0.000000 -9.988540 0.000000
g stem
s 1
usemtl brownskn
f 8 9 11 10
f 12 13 15 14
...
f 788 806 774

Modified:

20 October 1998

Author:

John Burkardt
				  */
{
	int   count;
	int   i;
	int   ivert;
	char *next;
	char *next2;
	char *next3;
	int   node;
	int   vertex_normal_num;
	float r1;
	float r2;
	float r3;
	char  token[LINE_MAX_LEN];
	char  token2[LINE_MAX_LEN];
	int   width;
	/* 
	Initialize. 
	*/
	vertex_normal_num = 0;
	/* 
	Read the next line of the file into INPUT. 
	*/
	char input[LINE_MAX_LEN];
	while ( fgets ( input, LINE_MAX_LEN, filein ) != NULL ) {
		
		++text_num;
		/* 
		Advance to the first nonspace character in INPUT. 
		*/
		for ( next = input; *next != '\0' && isspace(*next); next++ ) {
		}
		/* 
		Skip blank lines and comments. 
		*/
		
		if ( *next == '\0' ) {
			continue;
		}
		
		if ( *next == '#' || *next == '$' ) {
			comment_num = comment_num + 1;
			continue;
		}
		/* 
		Extract the first word in this line. 
		*/
		sscanf ( next, "%s%n", token, &width );
		/* 
		Set NEXT to point to just after this token. 
		*/
		
		next = next + width;
		/*
		BEVEL
		Bevel interpolation.
		*/
		if ( leqi ( token, "BEVEL" )  ) {
			continue;
		}
		/*
		BMAT
		Basis matrix.
		*/
		else if ( leqi ( token, "BMAT" )  ) {
			continue;
		}
		/*
		C_INTERP
		Color interpolation.
		*/
		else if ( leqi ( token, "C_INTERP" )  ) {
			continue;
		}
		/*
		CON
		Connectivity between free form surfaces.
		*/
		else if ( leqi ( token, "CON" )  ) {
			continue;
		}
		/*
		CSTYPE
		Curve or surface type.
		*/
		else if ( leqi ( token, "CSTYPE" )  ) {
			continue;
		}
		/*
		CTECH
		Curve approximation technique.
		*/
		else if ( leqi ( token, "CTECH" )  ) {
			continue;
		}
		/*
		CURV
		Curve.
		*/
		else if ( leqi ( token, "CURV" )  ) {
			continue;
		}
		/*
		CURV2
		2D curve.
		*/
		else if ( leqi ( token, "CURV2" )  ) {
			continue;
		}
		/*
		D_INTERP
		Dissolve interpolation.
		*/
		else if ( leqi ( token, "D_INTERP" )  ) {
			continue;
		}
		/*
		DEG
		Degree.
		*/
		else if ( leqi ( token, "DEG" )  ) {
			continue;
		}
		/*
		END
		End statement.
		*/
		else if ( leqi ( token, "END" )  ) {
			continue;
		}
		/*  
		F V1 V2 V3
		or
		F V1/VT1/VN1 V2/VT2/VN2 ...
		or
		F V1//VN1 V2//VN2 ...
		
		  Face.
		  A face is defined by the vertices.
		  Optionally, slashes may be used to include the texture vertex
		  and vertex normal indices.
		  
			OBJ line node indices are 1 based rather than 0 based.
			So we have to decrement them before loading them into FACE.
		*/
		
		else if ( leqi ( token, "F" )  ) {
			
			ivert = 0;
			face_order[face_num] = 0;
			/*
			Read each item in the F definition as a token, and then
			take it apart.
			*/
			for ( ;; ) {
				
				count = sscanf ( next, "%s%n", token2, &width );
				next = next + width;
				
				if ( count != 1 ) {
					break;
				}
				
				count = sscanf ( token2, "%d%n", &node, &width );
				next2 = token2 + width;
				
				if ( count != 1 ) {
					break;
				}
				
				// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
				{
					face[ivert][face_num] = node-1;
					vertex_material[ivert][face_num] = 0;
					face_order[face_num] = face_order[face_num] + 1;
				} 
				/*
				If there's a slash, skip to the next slash, and extract the
				index of the normal vector.
				*/
				if ( *next2 == '/' ) {
					
					for ( next3 = next2 + 1; next3 < token2 + LINE_MAX_LEN; next3++ ) {
						
						if ( *next3 == '/' ) {
							next3 = next3 + 1;
							count = sscanf ( next3, "%d%n", &node, &width );
							
							node = node - 1;
							if ( 0 <= node && node < vertex_normal_num ) {
								for ( i = 0; i < 3; ++i ) {
									vertex_normal[ivert][face_num][i] = normal_temp[node][i];
								}
							}
							break;
						}
					}
				}
				ivert = ivert + 1;
			} 
			++face_num;
		}
		
		/*  
		G  
		Group name.
		*/
		
		else if ( leqi ( token, "G" )  ) {
			continue;
		}
		/*
		HOLE
		Inner trimming hole.
		*/
		else if ( leqi ( token, "HOLE" )  ) {
			continue;
		}
		/*  
		L  
		I believe OBJ line node indices are 1 based rather than 0 based.
		So we have to decrement them before loading them into LINE_DEX.
		*/
		
		else if ( leqi ( token, "L" )  ) {
			
			for ( ;; ) {
				
				count = sscanf ( next, "%d%n", &node, &width );
				next = next + width;
				
				if ( count != 1 ) {
					break;
				}
				
				if ( line_num < LINES_MAX  ) {
					line_dex[line_num] = node-1;
					line_material[line_num] = 0;
				} 
				++line_num;
				
			} 
			
			if ( line_num < LINES_MAX ) {
				line_dex[line_num] = -1;
				line_material[line_num] = -1;
			}
			++line_num;
			
		}
		
		/*
		LOD
		Level of detail.
		*/
		else if ( leqi ( token, "LOD" )  ) {
			continue;
		}
		/*
		MG
		Merging group.
		*/
		else if ( leqi ( token, "MG" )  ) {
			continue;
		}
		/*
		MTLLIB
		Material library.
		*/
		
		else if ( leqi ( token, "MTLLIB" )  ) {
			continue;
		}
		/*
		O
		Object name.
		*/
		else if ( leqi ( token, "O" )  ) {
			continue;
		}
		/*
		P
		Point.
		*/
		else if ( leqi ( token, "P" )  ) {
			continue;
		}
		/*
		PARM
		Parameter values.
		*/
		else if ( leqi ( token, "PARM" )  ) {
			continue;
		}
		/*
		S  
		Smoothing group
		*/
		else if ( leqi ( token, "S" )  ) {
			continue;
		}
		/*
		SCRV
		Special curve.
		*/
		else if ( leqi ( token, "SCRV" )  ) {
			continue;
		}
		/*
		SHADOW_OBJ
		Shadow casting.
		*/
		else if ( leqi ( token, "SHADOW_OBJ" )  ) {
			continue;
		}
		/*
		SP
		Special point.
		*/
		else if ( leqi ( token, "SP" )  ) {
			continue;
		}
		/*
		STECH
		Surface approximation technique.
		*/
		else if ( leqi ( token, "STECH" )  ) {
			continue;
		}
		/*
		STEP
		Stepsize.
		*/
		else if ( leqi ( token, "CURV" )  ) {
			continue;
		}
		/*
		SURF
		Surface.
		*/
		else if ( leqi ( token, "SURF" )  ) {
			continue;
		}
		/*
		TRACE_OBJ
		Ray tracing.
		*/
		else if ( leqi ( token, "TRACE_OBJ" )  ) {
			continue;
		}
		/*
		TRIM
		Outer trimming loop.
		*/
		else if ( leqi ( token, "TRIM" )  ) {
			continue;
		}
		/*
		USEMTL  
		Material name.
		*/
		else if ( leqi ( token, "USEMTL" )  ) {
			continue;
		}
		
		/*
		V X Y Z W
		Geometric vertex.
		W is optional, a weight for rational curves and surfaces.
		The default for W is 1.
		*/
		
		else if ( leqi ( token, "V" )  ) {
			
			sscanf ( next, "%e %e %e", &r1, &r2, &r3 );
			
//			if ( cor3_num < COR3_MAX ) 
			{
				cor3[cor3_num]=vec3(r1,r2,r3);
			}
			
			cor3_num = cor3_num + 1;
			
		}
		/*
		VN
		Vertex normals.
		*/
		
		else if ( leqi ( token, "VN" )  ) {
			
			sscanf ( next, "%e %e %e", &r1, &r2, &r3 );
			
			// if ( vertex_normal_num < ORDER_MAX * FACE_MAX ) 
			{
				normal_temp[vertex_normal_num] = vec3(r1,r2,r3);
			}
			
			vertex_normal_num = vertex_normal_num + 1;
			
		}
		/*
		VT
		Vertex texture.
		*/
		else if ( leqi ( token, "VT" )  ) {
			continue;
		}
		/*
		VP
		Parameter space vertices.
		*/
		else if ( leqi ( token, "VP" )  ) {
			continue;
		}
		/*
		Unrecognized  
		*/
		else {
			++bad_num;
		}
		
  }
  return true;
}
/******************************************************************************/

bool IVCONV::obj_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

OBJ_WRITE writes a Wavefront OBJ file.

Example:

#  magnolia.obj

mtllib ./vp.mtl

g
v -3.269770 -39.572201 0.876128
v -3.263720 -39.507999 2.160890
...
v 0.000000 -9.988540 0.000000
g stem
s 1
usemtl brownskn
f 8 9 11 10
f 12 13 15 14
...
f 788 806 774

Modified:

01 September 1998

Author:

John Burkardt
*/
{
	int   i;
	int   iface;
	int   indexvn;
	int   ivert;
	int   k;
	bool   new2;
	int   text_num;
	float w;
	/* 
	Initialize. 
	*/
	text_num = 0;
	w = 1.0;
	
	fprintf ( fileout, "# %s created by IVCON.\n", fileout_name );
	fprintf ( fileout, "# Original data in %s.\n", filein_name );
	fprintf ( fileout, "\n" );
//	fprintf ( fileout, "g %s\n", object_name );
	fprintf ( fileout, "\n" );
	
	text_num = text_num + 5;
	/* 
	V: vertex coordinates. 
	*/
	for ( i = 0; i < cor3_num; ++i ) {
		fprintf ( fileout, "v %f %f %f %f\n", 
			cor3[i][0], cor3[i][1], cor3[i][2], w );
		++text_num;
	}
	
	/* 
	VN: Vertex face normal vectors. 
	*/
	if ( face_num > 0 ) {
		fprintf ( fileout, "\n" );
		++text_num;
	}
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			fprintf ( fileout, "vn %f %f %f\n", vertex_normal[ivert][iface][0],
				vertex_normal[ivert][iface][1], vertex_normal[ivert][iface][2] );
			++text_num;
		}
	}
	/* 
	F: faces. 
	*/
	if ( face_num > 0 ) {
		fprintf ( fileout, "\n" );
		++text_num;
	}
	
	indexvn = 0;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		fprintf ( fileout, "f" );
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			indexvn = indexvn + 1;
			fprintf ( fileout, " %d//%d", face[ivert][iface]+1, indexvn );
		}
		fprintf ( fileout, "\n" );
		++text_num;
	}
	/* 
	L: lines. 
	*/
	if ( line_num > 0 ) {
		fprintf ( fileout, "\n" );
		++text_num;
	}
	
	new2 = true;
	
	for ( i = 0; i < line_num; ++i ) {
		
		k = line_dex[i];
		
		if ( k == -1 ) {
			fprintf ( fileout, "\n" );
			++text_num;
			new2 = true;
		}
		else {
			if ( new2  ) {
				fprintf ( fileout, "l" );
				new2 = false;
			}
			fprintf ( fileout, " %d", k+1 );
		}
		
	}
	
	fprintf ( fileout, "\n" );
	++text_num;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "OBJ_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/******************************************************************************/

