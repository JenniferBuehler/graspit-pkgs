#include "ivconv.h"


/******************************************************************************/

bool IVCONV::smf_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  SMF_READ reads an SMF file.
  
	Example:
	
	  #SMF2.0
	  #  cube_face.smf
	  #  This example demonstrates how an RGB color can be assigned to
	  #  each face of an object.
	  #    
	  # First, define the geometry of the cube.
	  #
	  v 0.0  0.0  0.0
	  v 1.0  0.0  0.0
	  v 0.0  1.0  0.0
	  v 1.0  1.0  0.0
	  v 0.0  0.0  1.0
	  v 1.0  0.0  1.0
	  v 0.0  1.0  1.0
	  v 1.0  1.0  1.0
	  f 1 4 2
	  f 1 3 4
	  f 5 6 8
	  f 5 8 7
	  f 1 2 6
	  f 1 6 5
	  f 2 4 8
	  f 2 8 6
	  f 4 3 7
	  f 4 7 8
	  f 3 1 5
	  f 3 5 7
	  #
	  #  Colors will be bound 1 per face.
	  #
	  bind c face
	  c 1.0  0.0  0.0
	  c 1.0  0.0  0.0
	  c 0.0  1.0  0.0
	  c 0.0  1.0  0.0
	  c 0.0  0.0  1.0
	  c 0.0  0.0  1.0
	  c 1.0  1.0  0.0
	  c 1.0  1.0  0.0
	  c 0.0  1.0  1.0
	  c 0.0  1.0  1.0
	  c 1.0  0.0  1.0
	  c 1.0  0.0  1.0
	  #
	  #  Normal vectors will be bound 1 per face.
	  #
	  bind n face
	  n  0.0   0.0  -1.0
	  n  0.0   0.0  -1.0
	  n  0.0   0.0   1.0
	  n  0.0   0.0   1.0
	  n  0.0  -1.0   0.0
	  n  0.0  -1.0   0.0
	  n  1.0   0.0   0.0
	  n  1.0   0.0   0.0
	  n  0.0   1.0   0.0
	  n  0.0   1.0   0.0
	  n -1.0   0.0   0.0
	  n -1.0   0.0   0.0
	  #
	  #  Texture coordinate pairs will be bound 1 per face.
	  #
	  bind r face
	  r  0.0   0.0
	  r  0.0   0.1
	  r  0.0   0.2
	  r  0.0   0.3
	  r  0.1   0.0
	  r  0.1   0.1
	  r  0.1   0.2
	  r  0.1   0.3
	  r  0.2   0.0
	  r  0.2   0.1
	  r  0.2   0.2
	  r  0.2   0.3
	  
		Modified:
		
		  03 July 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	float angle;
	char  axis;
	float b;
	char  cnr[LINE_MAX_LEN];
	int   count;
	float dx;
	float dy;
	int   face_count;
	float g;
	int   icor3_normal;
	int   icor3_tex_uv;
	int   iface_normal;
	int   iface_tex_uv;
	int   imat;
	int   ivert;
	int   level;
	char *next;
	int   node;
	int   node_count;
	float r;
	float r1;
	float r2;
	float r3;
	float sx;
	float sy;
	float sz;
	char  token[LINE_MAX_LEN];
	char  token2[LINE_MAX_LEN];
	char  type[LINE_MAX_LEN];
	float u;
	float v;
	int   vertex_base;
	int   vertex_correction;
	int   width;
	float x;
	float xvec[3];
	float y;
	float z;
	
	face_count = 0;
	icor3_normal = 0;
	icor3_tex_uv = 0;
	iface_normal = 0;
	iface_tex_uv = 0;
	level = 0;
	node_count = 0;
	vertex_base = 0;
	vertex_correction = 0;
	/* 
	Read the next line of the file into INPUT. 
	*/
	char input[LINE_MAX_LEN];

	while ( fgets ( input, LINE_MAX_LEN, filein ) != NULL ) {
		
		++text_num;
		
		if ( debug ) {
			printf ( "SMF_READ: DEBUG: Reading line #%d\n", text_num );
		}
		/* 
		Advance to the first nonspace character in INPUT. 
		*/
		for ( next = input; *next != '\0' && isspace(*next); next++ ) {
		}
		/* 
		Skip blank lines. 
		*/
		
		if ( *next == '\0' ) {
			continue;
		}
		/*
		Skip comment lines.
		*/
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
		next += width;
		/*
		BEGIN
		Reset the transformation matrix to identity.
		Node numbering starts at zero again.  (Really, this is level based)
		(Really should define a new transformation matrix, and concatenate.)
		(Also, might need to keep track of level.)
		*/
		if ( leqi ( token, "BEGIN" )  ) {
			
			level = level + 1;
			
			vertex_base = cor3_num;
			group_num = group_num + 1;
			tmat_init ( transform_matrix );
			
		}
		/*
		BIND [c|n|r] [vertex|face]
		Specify the binding for RGB color, Normal, or Texture.
		Options are "vertex" or "face"
		*/
		else if ( leqi ( token, "BIND" )  ) {
			
			sscanf ( next, "%s%n", cnr, &width );
			next += width;
			
			if ( debug ) {
				printf ( "CNR = %s\n", cnr );
			}
			
			sscanf ( next, "%s%n", type, &width );
			next += width;
			
			if ( debug ) {
				printf ( "TYPE = %s\n", type );
			}
			
			if ( leqi ( cnr, "C" )  ) {
				
				if ( leqi ( type, "VERTEX" )  ) {
					strcpy ( material_binding, "PER_VERTEX" );
				}
				else if ( leqi ( type, "FACE" )  ) {
					strcpy ( material_binding, "PER_FACE" );
				}
				
			}
			else if ( leqi ( cnr, "N" )  ) {
				
				if ( leqi ( type, "VERTEX" )  ) {
					strcpy ( normal_binding, "PER_VERTEX" );
				}
				else if ( leqi ( type, "FACE" )  ) {
					strcpy ( normal_binding, "PER_FACE" );
				}
				
			}
			else if ( leqi ( cnr, "R" )  ) {
				
				if ( leqi ( type, "VERTEX" )  ) {
					strcpy ( texture_binding, "PER_VERTEX" );
				}
				else if ( leqi ( type, "FACE" )  ) {
					strcpy ( texture_binding, "PER_FACE" );
				}
				
			}
			
		}
		/*
		C <r> <g> <b>
		Specify an RGB color, with R, G, B between 0.0 and 1.0.
		*/
		else if ( leqi ( token, "C" )  ) {
			
			sscanf ( next, "%f%n", &r, &width );
			next += width;
			
			sscanf ( next, "%f%n", &g, &width );
			next += width;
			
			sscanf ( next, "%f%n", &b, &width );
			next += width;
			/*
			Set up a temporary material (R,G,B,1.0).
			Add the material to the material database, or find the index of
			a matching material already in.
			Assign the material of the node or face to this index.
			*/
							
			material[material_num]=Material(r,g,b);
			
			imat = material_num;
			++material_num;
			
			if ( leqi ( material_binding, "PER_FACE" )  ) {
				
				face_count = face_count + 1;
				face_material[face_count] = imat;
				
			}
			else if ( leqi ( material_binding, "PER_VERTEX" )  ) {
				
				node_count = node_count + 1;
				cor3_material[node_count] = imat;
				
			}
			else {
				
				printf ( "\n" );
				printf ( "SMF_READ - Fatal error!\n" );
				printf ( "  Material binding undefined!\n" );
				return false;
				
			}
			
		}
		/*
		END
		Drop down a level. 
		*/
		else if ( leqi ( token, "END" )  ) {
			
			level = level - 1;
			
			if ( level < 0 ) {
				printf ( "\n" );
				printf ( "SMF_READ - Fatal error!\n" );
				printf ( "  More END statements than BEGINs!\n" );
				return false;
			}
		}
		/*  
		F V1 V2 V3
		
		  Face.
		  A face is defined by the vertices.
		  Node indices are 1 based rather than 0 based.
		  So we have to decrement them before loading them into FACE.
		  Note that vertex indices start back at 0 each time a BEGIN is entered.
		  The strategy here won't handle nested BEGIN's, just one at a time.
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
				next += width;
				
				if ( count != 1 ) {
					break;
				}
				
				count = sscanf ( token2, "%d%n", &node, &width );
				
				if ( count != 1 ) {
					break;
				}
				
				// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
				{
					face[ivert][face_num] = node - 1 + vertex_base;
					vertex_material[ivert][face_num] = 0;
					face_order[face_num] = face_order[face_num] + 1;
				} 
				ivert = ivert + 1;
			} 
			++face_num;
		}
		/*
		N <x> <y> <z>
		Specify a normal vector.
		*/
		else if ( leqi ( token, "N" )  ) {
			
			sscanf ( next, "%f%n", &x, &width );
			next += width;
			
			sscanf ( next, "%f%n", &y, &width );
			next += width;
			
			sscanf ( next, "%f%n", &z, &width );
			next += width;
			
			if ( leqi ( normal_binding, "PER_FACE" )  ) {
				
				face_normal[iface_normal][0] = x;
				face_normal[iface_normal][1] = y;
				face_normal[iface_normal][2] = z;
				
				iface_normal = iface_normal + 1;
				
			}
			else if ( leqi ( normal_binding, "PER_VERTEX" )  ) {
				
				cor3_normal[icor3_normal][0] = x;
				cor3_normal[icor3_normal][1] = y;
				cor3_normal[icor3_normal][2] = z;
				
				icor3_normal = icor3_normal + 1;
				
			}
			else {
				
				printf ( "\n" );
				printf ( "SMF_READ - Fatal error!\n" );
				printf ( "  Normal binding undefined!\n" );
				return false;
				
			}
		}
		/*
		R <u> <v>
		Specify a texture coordinate.
		*/
		else if ( leqi ( token, "R" )  ) {
			
			sscanf ( next, "%f%n", &u, &width );
			next += width;
			
			sscanf ( next, "%f%n", &v, &width );
			next += width;
			
			if ( leqi ( texture_binding, "PER_FACE" )  ) {
				
				face_tex_uv[iface_tex_uv][0] = u;
				face_tex_uv[iface_tex_uv][1] = v;
				
				icor3_tex_uv = icor3_tex_uv + 1;
				
			}
			else if ( leqi ( texture_binding, "PER_VERTEX" )  ) {
				
				cor3_tex_uv[icor3_tex_uv][0] = u;
				cor3_tex_uv[icor3_tex_uv][1] = v;
				
				icor3_tex_uv = icor3_tex_uv + 1;
			}
			else {
				printf ( "\n" );
				printf ( "SMF_READ - Fatal error!\n" );
				printf ( "  Texture binding undefined!\n" );
				return false;
			}
			
		}
		/*
		ROT [x|y|z] <theta>
		*/
		else if ( leqi ( token, "ROT" )  ) {
			
			sscanf ( next, "%c%n", &axis, &width );
			next += width;
			
			sscanf ( next, "%f%n", &angle, &width );
			next += width;
			
			tmat_rot_axis ( transform_matrix, transform_matrix, angle, axis );
			
		}
		/*
		SCALE <sx> <sy> <sz>
		*/
		else if ( leqi ( token, "SCALE" )  ) {
			
			sscanf ( next, "%f%n", &sx, &width );
			next += width;
			
			sscanf ( next, "%f%n", &sy, &width );
			next += width;
			
			sscanf ( next, "%f%n", &sz, &width );
			next += width;
			
			tmat_scale ( transform_matrix, transform_matrix, sx, sy, sz );
		}
		/*
		SET VERTEX_CORRECTION <i>
		Specify increment to add to vertex indices in file.
		*/
		else if ( leqi ( token, "SET" )  ) {
			
			sscanf ( next, "%s%n", cnr, &width );
			next += width;
			
			sscanf ( next, "%d%n", &vertex_correction, &width );
			next += width;
			
		}
		/*
		T_SCALE <dx> <dy>
		Specify a scaling to texture coordinates.
		*/
		else if ( leqi ( token, "T_SCALE" )  ) {
			
			sscanf ( next, "%f%n", &dx, &width );
			next += width;
			
			sscanf ( next, "%f%n", &dy, &width );
			next += width;
			
		}
		/*
		T_TRANS <dx> <dy>
		Specify a translation to texture coordinates.
		*/
		else if ( leqi ( token, "T_TRANS" )  ) {
			
			sscanf ( next, "%f%n", &dx, &width );
			next += width;
			
			sscanf ( next, "%f%n", &dy, &width );
			next += width;
			
		}
		/*
		TEX <filename>
		Specify a filename containing the texture.
		(ANY CHANCE THIS IS RIGHT?)
		*/
		else if ( leqi ( token, "TEX" )  ) {
			char* string=NULL;
			sscanf ( next, "%s%n", string, &width );
			
			for ( int i = 0; i < LINE_MAX_LEN; ++i ) {
				texture_name[texture_num][i] = string[i];
				if ( string[i] == '\0' ) {
					break;
				}
			}
			
			texture_num = texture_num + 1;
			
		}
		/*
		TRANS <dx> <dy> <dz>
		*/
		else if ( leqi ( token, "TRANS" )  ) {
			
			sscanf ( next, "%f%n", &x, &width );
			next += width;
			
			sscanf ( next, "%f%n", &y, &width );
			next += width;
			
			sscanf ( next, "%f%n", &z, &width );
			next += width;
			
			tmat_trans ( transform_matrix, transform_matrix, x, y, z );
		}
		/*
		V X Y Z
		Geometric vertex.
		*/
		else if ( leqi ( token, "V" )  ) {
			
			sscanf ( next, "%e %e %e", &r1, &r2, &r3 );
			
			xvec[0] = r1;
			xvec[1] = r2;
			xvec[2] = r3;
			/*
			Apply current transformation matrix.
			Right now, we can only handle one matrix, not a stack of
			matrices representing nested BEGIN/END's.
			*/
			tmat_mxp ( transform_matrix, xvec, xvec );
			
//			if ( cor3_num < COR3_MAX ) 
			{
				cor3[cor3_num]= xvec;
			}
			
			cor3_num = cor3_num + 1;
			
		}
		/*
		Unrecognized keyword.
		*/
		else {
			
			++bad_num;
			
			if ( bad_num <= 10 ) {
				printf ( "\n" );
				printf ( "SMF_READ: Bad data on line %d.\n", text_num );
			}
		}
		
  }
  /*
  Extend the material definition 
  * from the face to the vertices and nodes, or
  * from the vertices to the faces and nodes.
  */
  if ( strcmp ( material_binding, "PER_FACE" ) == 0 ) {
	  
	  face_to_vertex_material ( );
	  
	  vertex_to_node_material ( );
	  
  }
  else if ( strcmp ( material_binding, "PER_VERTEX" ) == 0 ) {
	  
	  node_to_vertex_material ( );
	  
	  vertex_to_face_material ( );
	  
  }
  
  return true;
}
/******************************************************************************/

bool IVCONV::smf_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  SMF_WRITE writes graphics information to an SMF file.
  
	Example:
	
	  #SMF2.0
	  #  cube_face.smf
	  #  This example demonstrates how an RGB color can be assigned to
	  #  each face of an object.
	  #    
	  # First, define the geometry of the cube.
	  #
	  v 0.0  0.0  0.0
	  v 1.0  0.0  0.0
	  v 0.0  1.0  0.0
	  v 1.0  1.0  0.0
	  v 0.0  0.0  1.0
	  v 1.0  0.0  1.0
	  v 0.0  1.0  1.0
	  v 1.0  1.0  1.0
	  f 1 4 2
	  f 1 3 4
	  f 5 6 8
	  f 5 8 7
	  f 1 2 6
	  f 1 6 5
	  f 2 4 8
	  f 2 8 6
	  f 4 3 7
	  f 4 7 8
	  f 3 1 5
	  f 3 5 7
	  #
	  #  Colors will be bound 1 per face.
	  #
	  bind c face
	  c 1.0  0.0  0.0
	  c 1.0  0.0  0.0
	  c 0.0  1.0  0.0
	  c 0.0  1.0  0.0
	  c 0.0  0.0  1.0
	  c 0.0  0.0  1.0
	  c 1.0  1.0  0.0
	  c 1.0  1.0  0.0
	  c 0.0  1.0  1.0
	  c 0.0  1.0  1.0
	  c 1.0  0.0  1.0
	  c 1.0  0.0  1.0
	  #
	  #  Normal vectors will be bound 1 per face.
	  #
	  bind n face
	  n  0.0   0.0  -1.0
	  n  0.0   0.0  -1.0
	  n  0.0   0.0   1.0
	  n  0.0   0.0   1.0
	  n  0.0  -1.0   0.0
	  n  0.0  -1.0   0.0
	  n  1.0   0.0   0.0
	  n  1.0   0.0   0.0
	  n  0.0   1.0   0.0
	  n  0.0   1.0   0.0
	  n -1.0   0.0   0.0
	  n -1.0   0.0   0.0
	  #
	  #  Texture coordinate pairs will be bound 1 per face.
	  #
	  bind r face
	  r  0.0   0.0
	  r  0.0   0.1
	  r  0.0   0.2
	  r  0.0   0.3
	  r  0.1   0.0
	  r  0.1   0.1
	  r  0.1   0.2
	  r  0.1   0.3
	  r  0.2   0.0
	  r  0.2   0.1
	  r  0.2   0.2
	  r  0.2   0.3
	  
		Modified:
		
		  05 July 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int   i;
	int   icor3;
	int   iface;
	int   imat;
	int   ivert;
	int   text_num;
	/* 
	Initialize. 
	*/
	text_num = 0;
	
	fprintf ( fileout, "#$SMF 2.0\n" );
	fprintf ( fileout, "#$vertices %li\n", cor3_num );
	fprintf ( fileout, "#$faces %li\n", face_num );
	fprintf ( fileout, "#\n" );
	fprintf ( fileout, "# %s created by IVCON.\n", fileout_name );
	fprintf ( fileout, "# Original data in %s.\n", filein_name );
	fprintf ( fileout, "#\n" );
	
	text_num = text_num + 7;
	/* 
	V: vertex coordinates. 
	*/
	for ( i = 0; i < cor3_num; ++i ) {
		fprintf ( fileout, "v %f %f %f\n", 
			cor3[i][0], cor3[i][1], cor3[i][2] );
		++text_num;
	}
	/* 
	F: faces. 
	*/
	if ( face_num > 0 ) {
		fprintf ( fileout, "\n" );
		++text_num;
	}
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		fprintf ( fileout, "f" );
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			fprintf ( fileout, " %li", face[ivert][iface]+1 );
		}
		fprintf ( fileout, "\n" );
		++text_num;
	}
	/*
	Material binding.
	*/
	fprintf ( fileout, "bind c vertex\n" );
	++text_num;
	/*
	Material RGB values at each node.
	*/
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		
		imat = cor3_material[icor3];
		
		fprintf ( fileout, "c %f %f %f\n", material[imat].rgb[0], material[imat].rgb[1], material[imat].rgb[2]);
		
		++text_num;
	}
	/*
	Normal binding.
	*/
	fprintf ( fileout, "bind n vertex\n" );
	++text_num;
	/*
	Normal vector at each node.
	*/
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		
		fprintf ( fileout, "n %f %f %f\n", cor3_normal[icor3][0],
			cor3_normal[icor3][1], cor3_normal[icor3][2] );
		
		++text_num;
	}
	
	if ( texture_num > 0 ) {
	/*
	Texture filename.
		*/
		fprintf ( fileout, "tex %s\n", texture_name[0] );
		++text_num;
		/*
		Texture binding.
		*/
		fprintf ( fileout, "bind r vertex\n" );
		++text_num;
		/*
		Texture coordinates at each node.
		*/
		for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
			fprintf ( fileout, "r %f %f\n", cor3_tex_uv[icor3][0], 
				cor3_tex_uv[icor3][1] );
			++text_num;
		}
		
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "SMF_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
