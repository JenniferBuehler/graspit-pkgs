#include "ivconv.h"

bool IVCONV::txt_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  TXT_WRITE writes the graphics data to a text file.
  
	Modified:
	
	  25 June 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	int i;
	int iface;
	int iline;
	int imat;
	int ivert;
	int nitem;
	int text_num;
	
	text_num = 0;
	
	fprintf ( fileout, "%s created by IVCON.\n", fileout_name );
	fprintf ( fileout, "Original data in %s.\n", filein_name );
//	fprintf ( fileout, "Object name is %s.\n", object_name );
	fprintf ( fileout, "Object origin at %f %f %f.\n", origin[0], origin[1],
		origin[2] );
	fprintf ( fileout, "Object pivot at %f %f %f.\n", pivot[0], pivot[1],
		pivot[2] );
	text_num = text_num + 5;
	/*
	TRANSFORMATION MATRIX.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "Transformation matrix:\n" );
	fprintf ( fileout, "\n" );
	text_num = text_num + 3;
	
	for ( i = 0; i < 4; ++i ) {
		fprintf ( fileout, "  %f %f %f %f\n", transform_matrix[i][0],
			transform_matrix[i][1], transform_matrix[i][2], transform_matrix[i][3] );
		++text_num;
	}
	/*
	NODES.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "  %d nodes.\n", cor3_num );
	text_num = text_num + 2;
	
	if ( cor3_num > 0 ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Node coordinate data:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( i = 0; i < cor3_num; ++i ) {
			fprintf ( fileout, " %d %f %f %f\n ", i, cor3[i][0], cor3[i][1], 
				cor3[i][2] );
			++text_num;
		}
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Node normal vectors:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( i = 0; i < cor3_num; ++i ) {
			fprintf ( fileout, " %d %f %f %f\n ", i, cor3_normal[i][0], 
				cor3_normal[i][1], cor3_normal[i][2] );
			++text_num;
		}
		
		if (material_num>0)
		{
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "  Node materials:\n" );
			fprintf ( fileout, "\n" );
			text_num = text_num + 3;
			
			for ( i = 0; i < cor3_num; ++i ) {
				fprintf ( fileout, " %d %d\n ", i, cor3_material[i] );
				++text_num;
			}
			
			if ( texture_num > 0 ) {
				fprintf ( fileout, "\n" );
				fprintf ( fileout, "  Node texture coordinates:\n" );
				fprintf ( fileout, "\n" );
				text_num = text_num + 3;
				
				for ( i = 0; i < cor3_num; ++i ) {
					fprintf ( fileout, " %d %f %f\n ", i, cor3_tex_uv[i][0], 
						cor3_tex_uv[i][1] );
					++text_num;
				}
			}
		}
	}
	/*
	LINES.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "  %d line data items.\n", line_num );
	text_num = text_num + 2;
	
	if ( line_num > 0 ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Line index data:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		nitem = 0;
		
		for ( iline = 0; iline < line_num; iline++ ) {
			
			fprintf ( fileout, " %d", line_dex[iline] );
			nitem = nitem + 1;
			
			if ( iline == line_num - 1 || line_dex[iline] == -1 || nitem >= 10 ) {
				nitem = 0;
				fprintf ( fileout, "\n" );
				++text_num;
			}
			
		}
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Line materials:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		nitem = 0;
		
		for ( iline = 0; iline < line_num; iline++ ) {
			
			fprintf ( fileout, " %d", line_material[iline] );
			nitem = nitem + 1;
			
			if ( iline == line_num - 1 || line_material[iline] == -1 || nitem >= 10 ) {
				nitem = 0;
				fprintf ( fileout, "\n" );
				++text_num;
			}
		}
		
	}
	/*
	COLOR DATA
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "  %d colors.\n", color_num );
	text_num = text_num + 2;
	/*
	FACES.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "  %d faces.\n", face_num );
	text_num = text_num + 2;
	
	if ( face_num > 0 ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Face, Material, Number of vertices, Smoothing, Flags:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			fprintf ( fileout, " %d %d %d %d %d\n", iface, face_material[iface],
				face_order[iface], face_smooth[iface], face_flags[iface] );
			++text_num;
		}
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Face, Vertices\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			
			fprintf ( fileout, "%d   ", iface );
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				fprintf ( fileout, " %d", face[ivert][iface] );
			}
			
			fprintf ( fileout, "\n" );
			++text_num;
		}
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  Face normal vectors:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			fprintf ( fileout, " %d %f %f %f\n", iface, face_normal[iface][0],
				face_normal[iface][1], face_normal[iface][2] );
			++text_num;
		}
		
		if ( texture_num > 0 ) {
			
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "  Face texture coordinates:\n" );
			fprintf ( fileout, "\n" );
			text_num = text_num + 3;
			
			for ( iface = 0; iface < face_num; iface++ ) {
				fprintf ( fileout, " %d %f %f\n", iface, face_tex_uv[iface][0],
					face_tex_uv[iface][1] );
				++text_num;
			}
		}
	}
	/*
	VERTICES.
	*/
	if ( face_num > 0 ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "Vertex normal vectors:\n" );
		text_num = text_num + 2;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			fprintf ( fileout, "\n" );
			++text_num;
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				fprintf ( fileout, " %d %d %f %f %f\n", iface, ivert, 
					vertex_normal[ivert][iface][0], vertex_normal[ivert][iface][1],
					vertex_normal[ivert][iface][2] );
				++text_num;
			}
		}
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "Vertex materials:\n" );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			fprintf ( fileout, "%d", iface );
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				fprintf ( fileout, " %d", vertex_material[ivert][iface] );
			}
			fprintf ( fileout, "\n" );
			++text_num;
		}
		
		if ( texture_num > 0 ) {
			
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "Vertex UV texture coordinates:\n" );
			fprintf ( fileout, "\n" );
			text_num = text_num + 3;
			
			for ( iface = 0; iface < face_num; iface++ ) {
				for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
					fprintf ( fileout, "%d %d %f %f\n", iface, ivert,
						vertex_tex_uv[iface][ivert][0], vertex_tex_uv[iface][ivert][1] );
					++text_num;
				}
			}
		}
	}
	/*
	MATERIALS.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "%d materials.\n", material_num );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "Index      Name   R G B A\n" );
	fprintf ( fileout, "\n" );
	
	text_num = text_num + 5;
	
	for ( imat = 0; imat < material_num; imat++ ) {
		fprintf ( fileout, "%d %s %f %f %f %f\n", imat, material[imat].name,
			material[imat].ambient[0], material[imat].ambient[1], material[imat].ambient[2],
			material[imat].alpha );
		++text_num;
	}
	/*
	TEXTURES.
	*/
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "%d textures.\n", texture_num );
	text_num = text_num + 2;
	
	if ( texture_num > 0 ) {
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "Index  Name\n" );
		fprintf ( fileout, "\n" );
		for ( i = 0; i < texture_num; ++i ) {
			fprintf ( fileout, "%d %s\n", i, texture_name[i] );
		}
		text_num = text_num + 3;
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "TXT_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/**********************************************************************/

