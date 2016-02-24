#include "ivconv.h"

/******************************************************************************/

bool IVCONV::ase_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  ASE_READ reads an AutoCAD ASE file.
  
	Modified:
	
	  22 May 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	float bval;
	int   count;
	float gval;
	int   i;
	int   iface;
	int   ivert;
	int   iword;
	int   level;
	char *next;
	int   nlbrack;
	int   nrbrack;
	int   cor3_num_old;
	int   face_num_old;
	float rval;
	float temp;
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
	cor3_num_old = cor3_num;
	face_num_old = face_num;
	nlbrack = 0;
	nrbrack = 0;
	
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
		Read the next word from the line.
		*/
		for ( ;; ) {
			
			strcpy ( wordm1, word );
			strcpy ( word, " " );
			
			count = sscanf ( next, "%s%n", word, &width );
			next = next + width;
			
			if ( count <= 0 ) {
				break;
			}
			
			++iword;
			
			if ( iword == 1 ) {
				strcpy ( word1, word );
			}
			/*
			In case the new word is a bracket, update the bracket count.
			*/
			if ( strcmp ( word, "{" ) == 0 ) {
				
				++nlbrack;
				level = nlbrack - nrbrack;
				strcpy ( level_name[level], wordm1 );
			}
			else if ( strcmp ( word, "}" ) == 0 ) {
				
				++nrbrack;
				
				if ( nlbrack < nrbrack ) {
					
					printf ( "\n" );
					printf ( "ASE_READ - Fatal error!\n" );
					printf ( "  Extraneous right bracket on line %d\n", text_num );
					printf ( "  Currently processing field:\n" );
					printf ( "%s\n", level_name[level] );
					return false;
				}
				
			}
			/*
			*3DSMAX_ASCIIEXPORT  200
			*/
			if ( strcmp ( word1, "*3DSMAX_ASCIIEXPORT" ) == 0 ) {
				break;
			}
			/*
			*COMMENT
			*/
			else if ( strcmp ( word1, "*COMMENT" ) == 0 ) {
				break;
			}
			/*
			*GEOMOBJECT
			*/
			else if ( strcmp ( level_name[level], "*GEOMOBJECT" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				/*
				Why don't you read and save this name?
				*/
				else if ( strcmp ( word, "*NODE_NAME" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*NODE_TM" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*PROP_CASTSHADOW" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*PROP_MOTIONBLUR" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*PROP_RECVSHADOW" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in GEOMOBJECT, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH
			*/
			else if ( strcmp ( level_name[level], "*MESH" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*MESH_CFACELIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_CVERTLIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_FACE_LIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_NORMALS" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_NUMCVERTEX" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_NUMCVFACES" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_NUMFACES" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_NUMTVERTEX" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_NUMTVFACES" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_NUMVERTEX" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*MESH_TFACELIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_TVERTLIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*MESH_VERTEX_LIST" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "*TIMEVALUE" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_CFACELIST
			*/
			else if ( strcmp ( level_name[level], "*MESH_CFACELIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*MESH_CFACE" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_CFACE, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_CVERTLIST
			
			  Mesh vertex indices must be incremented by COR3_NUM_OLD before being stored
			  in the internal array.
			*/
			else if ( strcmp ( level_name[level], "*MESH_CVERTLIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*MESH_VERTCOL" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &i, &width );
					next = next + width;
					
					i = i + cor3_num_old;
					
					count = sscanf ( next, "%f%n", &rval, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &gval, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &bval, &width );
					next = next + width;
					
					material[material_num]=Material(rval,gval,bval);
					
					++material_num;
					cor3_material[i] = material_num;
				}
				else {
					++bad_num;
					printf ( "\n" );
					printf ( "ASE_READ - Warning!\n" );
					printf ( "  Bad data in MESH_CVERTLIST, line %d\n", text_num );
					break;
				}
				
			}
			/*
			*MESH_FACE_LIST
			This coding assumes a face is always triangular or quadrilateral.
			*/
			else if ( strcmp ( level_name[level], "*MESH_FACE_LIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*MESH_FACE" ) == 0 ) {
					
					// if ( face_num < FACE_MAX ) 
					{
						
						face_material[face_num] = 0;
						face_order[face_num] = 0;
						
						count = sscanf ( next, "%d%n", &i, &width );
						next = next + width;
						
						count = sscanf ( next, "%s%n", word2, &width );
						next = next + width;
						count = sscanf ( next, "%s%n", word2, &width );
						next = next + width;
						
						count = sscanf ( next, "%d%n", &i, &width );
						next = next + width;
						face[0][face_num] = i + cor3_num_old;
						++face_order[face_num];
						
						count = sscanf ( next, "%s%n", word2, &width );
						next = next + width;
						
						count = sscanf ( next, "%d%n", &i, &width );
						next = next + width;
						face[1][face_num] = i + cor3_num_old;
						++face_order[face_num];
						
						count = sscanf ( next, "%s%n", word2, &width );
						next = next + width;
						
						count = sscanf ( next, "%d%n", &i, &width );
						next = next + width;
						face[2][face_num] = i + cor3_num_old;
						++face_order[face_num];
						
						count = sscanf ( next, "%s%n", word2, &width );
						next = next + width;
						
						if ( strcmp ( word2, "D:" ) == 0 ) {
							count = sscanf ( next, "%d%n", &i, &width );
							next = next + width;
							face[3][face_num] = i + cor3_num_old;
							++face_order[face_num];
						}
					}
					
					++face_num;
					
					break;
					
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_FACE_LIST, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_NORMALS
			*/
			else if ( strcmp ( level_name[level], "*MESH_NORMALS" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*MESH_FACENORMAL" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &iface, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &x, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next = next + width;
					
					iface = iface + face_num_old;
					ivert = 0;
					
					face_normal[iface][0] = x;
					face_normal[iface][1] = y;
					face_normal[iface][2] = z;
					
					break;
					
				}
				else if ( strcmp ( word, "*MESH_VERTEXNORMAL" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &i, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &x, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next = next + width;
					
					vertex_normal[ivert][iface][0] = x;
					vertex_normal[ivert][iface][1] = y;
					vertex_normal[ivert][iface][2] = z;
					++ivert;
					
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_NORMALS, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_TFACELIST
			*/
			else if ( strcmp ( level_name[level], "*MESH_TFACELIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word1, "*MESH_TFACE" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_TFACE_LIST, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_TVERTLIST
			*/
			else if ( strcmp ( level_name[level], "*MESH_TVERTLIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word1, "*MESH_TVERT" ) == 0  ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_TVERTLIST, line %d\n", text_num );
					break;
				}
			}
			/*
			*MESH_VERTEX_LIST
			*/
			else if ( strcmp ( level_name[level], "*MESH_VERTEX_LIST" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					cor3_num_old = cor3_num;
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word1, "*MESH_VERTEX" ) == 0 ) {
					
					count = sscanf ( next, "%d%n", &i, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &x, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &y, &width );
					next = next + width;
					
					count = sscanf ( next, "%f%n", &z, &width );
					next = next + width;
					
					i = i + cor3_num_old;
					if ( cor3_num < i + 1 ) {
						cor3_num = i + 1;
					}
					
//					if ( i < COR3_MAX ) 
					{
						
						cor3[i][0] =
							transform_matrix[0][0] * x 
							+ transform_matrix[0][1] * y 
							+ transform_matrix[0][2] * z 
							+ transform_matrix[0][3];
						
						cor3[i][1] =
							transform_matrix[1][0] * x 
							+ transform_matrix[1][1] * y 
							+ transform_matrix[1][2] * z 
							+ transform_matrix[1][3];
						
						cor3[i][2] =
							transform_matrix[2][0] * x 
							+ transform_matrix[2][1] * y 
							+ transform_matrix[2][2] * z 
							+ transform_matrix[2][3];
					}
					
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in MESH_VERTEX_LIST, line %d\n", text_num );
					break;
				}
			}
			/*
			*NODE_TM
			
			  Each node should start out with a default transformation matrix.
			*/
			else if ( strcmp ( level_name[level], "*NODE_TM" ) == 0 ) {
				
				if ( strcmp ( word, "{" ) == 0 ) {
					
					tmat_init ( transform_matrix );
					
					continue;
				}
				else if ( strcmp ( word, "}" ) == 0 ) {
					level = nlbrack - nrbrack;
					continue;
				}
				else if ( strcmp ( word, "*INHERIT_POS" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*INHERIT_ROT" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*INHERIT_SCL" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*NODE_NAME" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_POS" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_ROTANGLE" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_ROTAXIS" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_ROW0" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[0][0] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[1][0] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[2][0] = temp;
					
					break;
				}
				else if ( strcmp ( word, "*TM_ROW1" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[0][1] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[1][1] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[2][1] = temp;
					
					break;
				}
				else if ( strcmp ( word, "*TM_ROW2" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[0][2] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[1][2] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[2][2] = temp;
					
					break;
				}
				else if ( strcmp ( word, "*TM_ROW3" ) == 0 ) {
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[0][3] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[1][3] = temp;
					
					count = sscanf ( next, "%f%n", &temp, &width );
					next = next + width;
					transform_matrix[2][3] = temp;
					
					break;
				}
				else if ( strcmp ( word, "*TM_SCALE" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_SCALEAXIS" ) == 0 ) {
					break;
				}
				else if ( strcmp ( word, "*TM_SCALEAXISANG" ) == 0 ) {
					break;
				}
				else {
					++bad_num;
					printf ( "Bad data in NODE_TM, line %d\n", text_num );
					break;
				}
      }
	  /*
	  *SCENE
	  */
      else if ( strcmp ( level_name[level], "*SCENE" ) == 0 ) {
		  
		  if ( strcmp ( word, "{" ) == 0 ) {
			  continue;
		  }
		  else if ( strcmp ( word, "}" ) == 0 ) {
			  level = nlbrack - nrbrack;
			  continue;
		  }
		  else if ( strcmp ( word, "*SCENE_AMBIENT_STATIC" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_BACKGROUND_STATIC" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_FILENAME" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_FIRSTFRAME" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_FRAMESPEED" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_LASTFRAME" ) == 0 ) {
			  break;
		  }
		  else if ( strcmp ( word, "*SCENE_TICKSPERFRAME" ) == 0 ) {
			  break;
		  }
		  else {
			  ++bad_num;
			  printf ( "Bad data in SCENE, line %d\n", text_num );
			  break;
		  }
		  
      }
	  
    }
	/*
	End of loop reading words from the line.
	*/
  }
  /*
  End of loop reading lines from input file.
  */
  
  return true;
}
/******************************************************************************/

bool IVCONV::ase_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  ASE_WRITE writes graphics information to an AutoCAD ASE file.
  
	Modified:
	
	  30 September 1998
	  
		Author:
		
		  John Burkardt
		  
			*/
{
	int i1;
	int i2;
	int i3;
	int i4;
	int iface;
	int ivert;
	int j;
	int text_num;
	
	text_num = 0;
	/*
	Write the header.
	*/
	fprintf ( fileout, "*3DSMAX_ASCIIEXPORT 200\n" );
	fprintf ( fileout, "*COMMENT \"%s, created by IVCON.\"\n", fileout_name );
	fprintf ( fileout, "*COMMENT \"Original data in %s\"\n", filein_name );
	
	text_num = text_num + 3;
	/*
	Write the scene block.
	*/
	fprintf ( fileout, "*SCENE {\n" );
	fprintf ( fileout, "  *SCENE_FILENAME \"\"\n" );
	fprintf ( fileout, "  *SCENE_FIRSTFRAME 0\n" );
	fprintf ( fileout, "  *SCENE_LASTFRAME 100\n" );
	fprintf ( fileout, "  *SCENE_FRAMESPEED 30\n" );
	fprintf ( fileout, "  *SCENE_TICKSPERFRAME 160\n" );
	fprintf ( fileout, "  *SCENE_BACKGROUND_STATIC 0.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "  *SCENE_AMBIENT_STATIC 0.0431 0.0431 0.0431\n" );
	fprintf ( fileout, "}\n" );
	
	text_num = text_num + 9;
	/*
	Begin the big geometry block.
	*/
	fprintf ( fileout, "*GEOMOBJECT {\n" );
//	fprintf ( fileout, "  *NODE_NAME \"%s\"\n", object_name );
	
	text_num = text_num + 2;
	/*
	Sub block NODE_TM:
	*/
	fprintf ( fileout, "  *NODE_TM {\n" );
	fprintf ( fileout, "    *NODE_NAME \"Object01\"\n" );
	fprintf ( fileout, "    *INHERIT_POS 0 0 0\n" );
	fprintf ( fileout, "    *INHERIT_ROT 0 0 0\n" );
	fprintf ( fileout, "    *INHERIT_SCL 0 0 0\n" );
	fprintf ( fileout, "    *TM_ROW0 1.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_ROW1 0.0000 1.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_ROW2 0.0000 0.0000 1.0000\n" );
	fprintf ( fileout, "    *TM_ROW3 0.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_POS 0.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_ROTAXIS 0.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_ROTANGLE 0.0000\n" );
	fprintf ( fileout, "    *TM_SCALE 1.0000 1.0000 1.0000\n" );
	fprintf ( fileout, "    *TM_SCALEAXIS 0.0000 0.0000 0.0000\n" );
	fprintf ( fileout, "    *TM_SCALEAXISANG 0.0000\n" );
	fprintf ( fileout, "  }\n" );
	
	text_num+=16;
	/*
	Sub block MESH:
    Items
	*/
	fprintf ( fileout, "  *MESH {\n" );
	fprintf ( fileout, "    *TIMEVALUE 0\n" );
	fprintf ( fileout, "    *MESH_NUMVERTEX %li\n", cor3_num );
	fprintf ( fileout, "    *MESH_NUMFACES %li\n", face_num );
	
	text_num = text_num + 4;
	/*
	Sub sub block MESH_VERTEX_LIST
	*/
	fprintf ( fileout, "    *MESH_VERTEX_LIST {\n" );
	++text_num;
	
	for ( j = 0; j < cor3_num; ++j ) {
		fprintf ( fileout, "      *MESH_VERTEX %d %f %f %f\n", j, cor3[j][0],
			cor3[j][1], cor3[j][2] );
		++text_num;
	}
	
	fprintf ( fileout, "    }\n" );
	++text_num;
	/*
	Sub sub block MESH_FACE_LIST
    Items MESH_FACE
	*/
	fprintf ( fileout, "    *MESH_FACE_LIST {\n" );
	++text_num;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		i1 = face[0][iface];
		i2 = face[1][iface];
		i3 = face[2][iface];
		
		if ( face_order[iface] == 3 ) {
			fprintf ( fileout, "      *MESH_FACE %d: A: %d B: %d C: %d", iface, i1, i2, i3 ); 
			fprintf ( fileout, " AB: 1 BC: 1 CA: 1 *MESH_SMOOTHING *MESH_MTLID 1\n" );
			++text_num;
		}
		else if ( face_order[iface] == 4 ) {
			i4 = face[3][iface];
			fprintf ( fileout, "      *MESH_FACE %d: A: %d B: %d C: %d D: %d", iface, i1, i2, i3, i4 ); 
			fprintf ( fileout, " AB: 1 BC: 1 CD: 1 DA: 1 *MESH_SMOOTHING *MESH_MTLID 1\n" );
			++text_num;
		}
	}
	
	fprintf ( fileout, "    }\n" );
	++text_num;
	/*
	Item MESH_NUMTVERTEX.
	*/
	fprintf ( fileout, "    *MESH_NUMTVERTEX 0\n" );
	++text_num;
	/*
	Item NUMCVERTEX.
	*/
	fprintf ( fileout, "    *MESH_NUMCVERTEX 0\n" );
	++text_num;
	/*
	Sub block MESH_NORMALS
    Items MESH_FACENORMAL, MESH_VERTEXNORMAL (repeated)
	*/
	fprintf ( fileout, "    *MESH_NORMALS {\n" );
	++text_num;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		fprintf ( fileout, "      *MESH_FACENORMAL %d %f %f %f\n", 
			iface, face_normal[iface][0], face_normal[iface][1], face_normal[iface][2] );
		++text_num;
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			fprintf ( fileout, "      *MESH_VERTEXNORMAL %li %f %f %f\n", 
				face[ivert][iface], vertex_normal[ivert][iface][0], 
				vertex_normal[ivert][iface][1], vertex_normal[ivert][iface][2] );
			++text_num;
		}
	}
	
	fprintf ( fileout, "    }\n" );
	++text_num;
	/*
	Close the MESH object.
	*/
	fprintf ( fileout, "  }\n" );
	/*
	A few closing parameters.
	*/
	fprintf ( fileout, "  *PROP_MOTIONBLUR 0\n" );
	fprintf ( fileout, "  *PROP_CASTSHADOW 1\n" );
	fprintf ( fileout, "  *PROP_RECVSHADOW 1\n" );
	/*
	Close the GEOM object.
	*/
	fprintf ( fileout, "}\n" );
	
	text_num = text_num + 5;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "ASE_WRITE - Wrote %d text lines;\n", text_num );
	
	return true;
}
