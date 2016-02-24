/******************************************************************************/
// Author: John Burkardt 
// Created 23 September 1998
// Change: 2003/03/15 Philippe Guglielmetti correct material handling
//					  according to http://www.the-labs.com/Blender/3DS-details.html

#include "ivconv.h"

#include "ofile.h"	// file from 3D Studio Max API

int tds_write_string ( FILE *fileout, char *string )
{
	int   nchar = 0;
	for ( char* c = string; nchar < 12; c++ ) {

		fputc ( *c, fileout );
		nchar = nchar + 1;
		
		if  ( *c == 0 ) break;
	}
	return nchar;
}


int tds_write_u_short_int ( FILE *fileout, unsigned short int short_int_val )

/******************************************************************************/

/*
Modified:

  14 October 1998
  
	Author:
	
	  John Burkardt
	  */
{
	union {
		unsigned short int yint;
		char ychar[2];
	} y;
	
	y.yint = short_int_val;
	
	if ( byte_swap  ) {
		fputc ( y.ychar[1], fileout );
		fputc ( y.ychar[0], fileout );
	}
	else {
		fputc ( y.ychar[0], fileout );
		fputc ( y.ychar[1], fileout );
	}
	
	return 2;
}

unsigned long tds_read_u_long_int ( FILE *filein )
{
	union {
		unsigned long yint;
		char ychar[4];
	} y;
	
	if ( byte_swap  ) {
		y.ychar[3] = fgetc ( filein );
		y.ychar[2] = fgetc ( filein );
		y.ychar[1] = fgetc ( filein );
		y.ychar[0] = fgetc ( filein );
	}
	else {
		y.ychar[0] = fgetc ( filein );
		y.ychar[1] = fgetc ( filein );
		y.ychar[2] = fgetc ( filein );
		y.ychar[3] = fgetc ( filein );
	}
	
	return y.yint;
}

unsigned short int tds_read_u_short_int ( FILE *filein )
{
	unsigned char c1 = fgetc ( filein );
	unsigned char c2 = fgetc ( filein );
	
	return  c1 | ( c2 << 8 );
	}

void tds_pre_process ()

/******************************************************************************/

/*
Purpose:

  TDS_PRE_PROCESS divides the monolithic object into acceptably small pieces.
  
	Note:
	
	  The 3DS binary format allows an unsigned short int for the number of
	  points, and number of faces in an object.  This limits such quantities
	  to 65535.  We have at least one interesting object with more faces
	  than that.  So we need to tag faces and nodes somehow.
	  
		Modified:
		
		  14 October 1998
		  
			Author:
			
			  John Burkardt
			  */
{
	/*  static unsigned short int BIG = 60000; */
	
	return;
}

unsigned long tds_read_unknown_section ( FILE *filein )

/******************************************************************************/
{	
	unsigned long current_pointer = ftell ( filein ) - 2;
	unsigned long temp_pointer = tds_read_u_long_int ( filein );
	
	long pointer = current_pointer + temp_pointer;
	
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

//! @return number of bytes read from file
unsigned long tds_read_color (FILE *filein, unsigned short selector, float* rgb_val)
{
	switch ( selector ) {
	case COLOR_F:
		{
			if ( debug ) printf ( "COLOR_F color definition section tag of %0X\n", selector );
			for (int i = 0; i < 3; ++i ) rgb_val[i] = float_read ( filein );
			if ( debug ) printf ( "RGB_VAL = %f %f %f\n", rgb_val[0], rgb_val[1], rgb_val[2] );
			return 3 * sizeof ( float );
		}
	case COLOR_24:
		{
			if ( debug ) printf ( "COLOR_24 24 bit color definition section tag of %0X\n", selector );
			unsigned char true_c_val[3];
			for (int i = 0; i < 3; ++i ) 
			{
				true_c_val[i] = fgetc ( filein );
				rgb_val[i] = float(true_c_val[i])/255.0f;
			}
			if ( debug ) printf ( "TRUE_C_VAL = %d %d %d\n", true_c_val[0], true_c_val[1], true_c_val[2] );
			return 3;
		}
	default:
		return 0;
	}
}

//! @return number of bytes read from file
unsigned long tds_read_color_chunk ( FILE *filein, vec3& rgb)
{
	unsigned long temp_pointer = tds_read_u_long_int ( filein );
	unsigned long teller = 6;
	
	while (teller < temp_pointer) {
		unsigned short temp_int = tds_read_u_short_int ( filein );
		teller+=2+tds_read_color(filein,temp_int,&rgb[0]);
	}	
	return temp_pointer;
}


unsigned long tds_read_ambient_section ( FILE *filein )
{
	vec3 rgb_val;
	
	unsigned long current_pointer = ftell ( filein ) - 2;	
	unsigned long temp_pointer=tds_read_color_chunk(filein,rgb_val);
	long pointer = current_pointer + temp_pointer;
	fseek ( filein, pointer, SEEK_SET );
	
	return temp_pointer;
}

unsigned long tds_read_background_section ( FILE *filein )
{
	vec3 rgb_val;
	
	unsigned long current_pointer = ftell ( filein ) - 2;	
	unsigned long temp_pointer=tds_read_color_chunk(filein,rgb_val);
	long pointer = current_pointer + temp_pointer;
	fseek ( filein, pointer, SEEK_SET );
	
	return temp_pointer;
}
/******************************************************************************/

unsigned long tds_read_boolean ( unsigned char *boolean, FILE *filein )

/******************************************************************************/

{
	unsigned long current_pointer;
	long      pointer;
	unsigned long temp_pointer;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	*boolean = fgetc ( filein );
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}
/******************************************************************************/

unsigned long tds_read_camera_section ( FILE *filein )

/******************************************************************************/
{
	float               camera_eye[3];
	float               camera_focus[3];
	unsigned long   current_pointer;
	float               lens;
	long            pointer;
	float               rotation;
	unsigned long   temp_pointer;
	unsigned short int  u_short_int_val;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	camera_eye[0] = float_read ( filein );
	camera_eye[1] = float_read ( filein );
	camera_eye[2] = float_read ( filein );
	
	camera_focus[0] = float_read ( filein );
	camera_focus[1] = float_read ( filein );
	camera_focus[2] = float_read ( filein );
	
	rotation = float_read ( filein );
	lens = float_read ( filein );
	
	if ( debug ) {
		printf ( " Found camera viewpoint at XYZ = %f %f %f.\n",
			camera_eye[0], camera_eye[1], camera_eye[2] );
		printf ( "     Found camera focus coordinates at XYZ = %f %f %f.\n",
			camera_focus[0], camera_focus[1], camera_focus[2] );
		printf ( "     Rotation of camera is:  %f.\n", rotation );
		printf ( "     Lens in used camera is: %f mm.\n", lens );
	}
	
	if ( ( temp_pointer-38 ) > 0 ) {
		
		if ( debug ) {
			printf ( "          Found extra camera sections.\n" );
		}
		
		u_short_int_val = tds_read_u_short_int ( filein );
		
		if ( u_short_int_val == CAM_SEE_CONE ) {
			if ( debug ) {
				printf ( "          CAM_SEE_CONE.\n" );
			}
			tds_read_unknown_section ( filein );
		}
		
		u_short_int_val = tds_read_u_short_int ( filein );
		
		if ( u_short_int_val == CAM_RANGES ) {
			if ( debug ) {
				printf ( "          CAM_RANGES.\n" );
			}
			tds_read_unknown_section ( filein );
		}
		
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

char   temp_name[500];

int tds_read_name ( FILE *filein )

/******************************************************************************/
{
	unsigned char  letter;
	unsigned int   teller;

	teller = 0;
	letter = fgetc ( filein );
	/*
	Could be a dummy object.  
	*/
	
	if ( letter == 0 ) {
		strcpy ( temp_name, "Default name" );
		return (-1); 
	}
	
	temp_name[teller] = letter;
	teller+=1;
	
	do {
		letter = fgetc ( filein );
		temp_name[teller] = letter;
		teller+=1;
	} while ( ( letter != 0 ) && ( teller < 12 ) );
	
	temp_name[teller-1] = 0;
	
	if ( debug ) {
		printf ( "      tds_read_name found name: %s.\n", temp_name );
	}
	
	return 0;
}

unsigned long tds_read_spot_section ( FILE *filein )

/******************************************************************************/
{
	unsigned long  current_pointer;
	float              falloff;
	float              hotspot;
	long           pointer;
	float              target[4];
	unsigned long  temp_pointer;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	target[0] = float_read ( filein );
	target[1] = float_read ( filein );
	target[2] = float_read ( filein );
	hotspot = float_read ( filein );
	falloff = float_read ( filein );
	
	if ( debug ) {
		printf ( "      The target of the spot is XYZ = %f %f %f.\n",
			target[0], target[1], target[2] );
		printf ( "      The hotspot of this light is %f.\n", hotspot );
		printf ( "      The falloff of this light is %f.\n", falloff );
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}
/******************************************************************************/


unsigned long tds_read_light_section ( FILE *filein )

/******************************************************************************/
{
	unsigned char       boolean;
	unsigned long   current_pointer;
	bool end_found = false;
	float               light_coors[3];
	long            pointer;
	float               rgb_val[3];
	unsigned long   teller;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	
	light_coors[0] = float_read ( filein );
	light_coors[1] = float_read ( filein );
	light_coors[2] = float_read ( filein );
	
	teller+=3 * 4;
	
	if ( debug ) {
		printf ( "     Found light at coordinates XYZ = %f %f %f.\n",
			light_coors[0], light_coors[1], light_coors[2] );
	}
	
	while (!end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
		case COLOR_F:
		case COLOR_24:
			teller+=tds_read_color(filein,temp_int,&rgb_val[0]);
			break;
		case DL_OFF:
			if ( debug ) {
				printf ( "      DL_OFF section: %0X\n", temp_int );
			}
			teller+=tds_read_boolean ( &boolean, filein );
			if ( debug ) {
				if ( boolean  ) {
					printf ( "      Light is on\n" );
				}
				else {
					printf ( "      Light is off\n" );
				}
			}
			break;
		case DL_SPOTLIGHT:
			if ( debug  ) {
				printf ( "      DL_SPOTLIGHT section tag of %0X\n", temp_int );
			}
			teller+=tds_read_spot_section ( filein );
			break;
		case DL_OUTER_RANGE:
			if ( debug  ) {
				printf ( "      DL_OUTER_RANGE section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default:
			break;
		}
		
		if ( teller >= temp_pointer ) {
			end_found = true;
		}
		
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

unsigned long IVCONV::tds_read_obj_section ( FILE *filein )

/******************************************************************************/

/*
Comments:

  Thanks to John F Flanagan for some suggested corrections.
  
	Modified:
	
	  30 June 2001
	  */
{
	unsigned long   chunk_size;
	unsigned short int  color_index;
	unsigned long   current_pointer;
	unsigned char       end_found = false;
	int                 i;
	int                 j;
	int                 cor3_num_base;
	int                 cor3_num_inc;
	int                 face_num_inc;
	long            pointer;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   temp_pointer2;
	unsigned long   teller; 
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	cor3_num_base = cor3_num;
	
	while (!end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
			
		case 0x4000:
			if ( debug ) {
				printf ( "        NAMED_OBJECT section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		case 0x4100:
			if ( debug ) {
				printf ( "        N_TRI_OBJECT section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		case 0x4110:
			
			if ( debug ) {
				printf ( "        POINT_ARRAY section tag of %0X\n", temp_int );
			}
			
			current_pointer = ftell ( filein ) - 2;
			temp_pointer2 = tds_read_u_long_int ( filein );
			cor3_num_inc =  ( int ) tds_read_u_short_int ( filein );
			
			for ( i = cor3_num; i < cor3_num + cor3_num_inc; ++i ) {
				cor3[i][0] = float_read ( filein );
				cor3[i][1] = float_read ( filein );
				cor3[i][2] = float_read ( filein );
			}
			
			cor3_num = cor3_num + cor3_num_inc;
			teller+=temp_pointer2;
			break;
			
		case 0x4111:
			if ( debug ) {
				printf ( "        POINT_FLAG_ARRAY faces (2) section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		case FACE_ARRAY:
			
			if ( debug ) {
				printf ( "        FACE_ARRAY section tag of %0X\n", 
					temp_int );
			}
			
			temp_pointer2 = tds_read_u_long_int ( filein );
			face_num_inc = ( int ) tds_read_u_short_int ( filein );
			
			for ( i = face_num; i < face_num + face_num_inc; ++i ) {
				face[0][i] = tds_read_u_short_int ( filein ) + cor3_num_base;
				face[1][i] = tds_read_u_short_int ( filein ) + cor3_num_base;
				face[2][i] = tds_read_u_short_int ( filein ) + cor3_num_base;
				face_order[i] = 3;
				face_flags[i] = tds_read_u_short_int ( filein );
				/*
				Color is given per face, and as 24 bit RGB data packed in one word.
				Extract RGB from the word, and assign R / 255 to each vertex.
				
				  Just a guess, JVB, 30 June 2001.
				temp_int = face_flags[i] & 0x000F;
				r = ( temp_int & 0x0004 ) >> 2;
				g = ( temp_int & 0x0002 ) >> 1;
				b = ( temp_int & 0x0001 );
				
				for ( j = 0; j < 3; ++j ) {
					vertex_rgb[j][i][0] = r;
					vertex_rgb[j][i][1] = g;
					vertex_rgb[j][i][2] = b;
				}

				PhG 2003/03/15
				No, it isn't that simple : you cannot pack 24 bits in 8...
				it is a "visibility flag", whatever that means...
				*/

			}
			
			temp_int = tds_read_u_short_int ( filein );
			if ( temp_int == 0x4150 ) {
				for ( i = face_num; i < face_num + face_num_inc; ++i ) {
					face_smooth[i] = ( int ) tds_read_u_long_int ( filein ) 
						+ cor3_num_base;
				}
			}
			face_num = face_num + face_num_inc;
			teller = ftell ( filein );
			break;
			
		case MSH_MAT_GROUP:
			if ( debug ) {
				printf ( "        MSH_MAT_GROUP section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		case 0x4140:
			if ( debug ) {
				printf ( "        TEX_VERTS section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_tex_verts_section ( filein );
			break;
			
		case 0x4150:
			if ( debug ) {
				printf ( "        SMOOTH_GROUP section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		case 0x4160:
			
			if ( debug ) {
				printf ( "        MESH_MATRIX section tag of %0X\n", 
					temp_int );
			}
			
			tds_read_u_long_int ( filein );
			
			for ( j = 0; j < 4; ++j ) {
				for ( i = 0; i < 3; ++i ) {
					transform_matrix[j][i] = float_read ( filein );
				}
			}
			transform_matrix[0][3] = 0.0;
			transform_matrix[1][3] = 0.0;
			transform_matrix[2][3] = 0.0;
			transform_matrix[3][3] = 0.0;
			
			teller+=12 * sizeof ( float );
			break;
			
		case 0x4165:
			
			if ( debug ) {
				printf ( "        MESH_COLOR section tag of %0X\n", temp_int );
			}
			
			chunk_size = tds_read_u_long_int ( filein );
			
			if ( chunk_size == 7 ) {
				color_index = fgetc ( filein );
				teller+=5;
			}
			else {
				color_index = tds_read_u_short_int ( filein );
				teller+=6;
			} 
			if ( debug ) {
				printf ( "        Color index set to %d\n", color_index );
			}
			break;
			
		case 0x4170:
			if ( debug ) {
				printf ( "        MESH_TEXTURE_INFO section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
			
		default:
			if ( debug ) {
				printf ( "        JUNK section tag of %0X\n", temp_int );
			}
			break;
    }
	
    if (  teller >= temp_pointer ) {
		end_found = true;
    }
	
  }
  
  pointer = ( long ) ( current_pointer + temp_pointer );
  fseek ( filein, pointer, SEEK_SET );
  
  return ( temp_pointer );
}

char   object_name[500];

unsigned long IVCONV::tds_read_object_section ( FILE *filein )

{
	bool end_found = false;
	unsigned long   current_pointer;
	int                 int_val;
	long            pointer;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   teller;

	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	/*
	Why don't you read and save the name here?
	*/
	int_val = tds_read_name ( filein );
	
	if ( int_val == -1 ) {
		if ( debug ) {
			printf ( "      Dummy Object found\n" );
		}
	}
	else {
		strcpy ( object_name, temp_name );
	}
	
	while ( !end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
		case 0x4700:
			if ( debug ) {
				printf ( "      N_CAMERA section tag of %0X\n", temp_int );
			}
			teller+=tds_read_camera_section ( filein );
			break;
		case 0x4600:
			if ( debug ) {
				printf ( "      N_DIRECT_LIGHT section tag of %0X\n", temp_int );
			}
			teller+=tds_read_light_section ( filein );
			break;
		case 0x4100:
			if ( debug ) {
				printf ( "      OBJ_TRIMESH section tag of %0X\n", temp_int );
			}
			teller+=tds_read_obj_section ( filein );
			break;
		case 0x4010: 
			if ( debug ) {
				printf ( "      OBJ_HIDDEN section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x4012: 
			if ( debug ) {
				printf ( "      OBJ_DOESNT_CAST section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default:
			break;
		}
		
		if ( teller >= temp_pointer ) {
			end_found = true;
		}
		
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}
/******************************************************************************/


int tds_read_long_name ( FILE *filein )

/******************************************************************************/
{
	unsigned char  letter;
	unsigned int   teller;
	
	teller = 0;
	letter = fgetc ( filein );
	/*
	Could be a dummy object. 
	*/
	if ( letter == 0 ) {
		strcpy ( temp_name, "Default_name" );
		return -1; 
	}
	
	temp_name[teller] = letter;
	teller+=1;
	
	do {
		letter = fgetc ( filein );
		temp_name[teller] = letter;
		teller+=1;
	} while ( letter != 0 );
	
	temp_name[teller-1] = 0;
	
	if ( debug ) {
		printf ( "      tds_read_long_name found name: %s.\n", temp_name );
	}
	
	return teller;
}
/******************************************************************************/


unsigned long tds_read_matdef_section ( FILE *filein, char* mat_name)

/******************************************************************************/
{
	unsigned long  current_pointer = ftell ( filein ) - 2;
	unsigned long temp_pointer = tds_read_u_long_int ( filein );
	
	int teller = tds_read_long_name ( filein );
	
	if ( teller == -1 ) {
		if ( debug ) {
			printf ( "      No material name found.\n" );
		}
	}
	else {
		strcpy ( mat_name, temp_name );
		if ( debug ) {
			printf ( "      Material name %s.\n", mat_name );
		}
	}
	
	long pointer = current_pointer + temp_pointer;
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

unsigned long IVCONV::tds_read_texmap_section ( FILE *filein )

/******************************************************************************/

/*
Purpose:

TDS_READ_TEXMAP_SECTION tries to get the TEXMAP name from the TEXMAP section.

Warning:

The code has room for lots of textures.  In this routine, we behave as
though there were only one, and we stick its name in the first name slot.

Modified:

30 June 1999

Author:

John Burkardt
*/
{
	unsigned long  current_pointer;
	long           pointer;
	int                teller;
	unsigned long  temp_pointer;
	
	texture_num = texture_num + 1;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	tds_read_u_short_int ( filein );
	tds_read_u_short_int ( filein );
	tds_read_u_short_int ( filein );
	tds_read_u_short_int ( filein );
	
	/*
	This next short int should equal A300.
	*/
	tds_read_u_short_int ( filein );
	tds_read_u_long_int ( filein );
	/*
	Now read the TEXMAP file name.
	*/
	teller = tds_read_long_name ( filein );
	
	if ( teller == -1 ) {
		if ( debug ) {
			printf ( "      No TEXMAP name found.\n" );
		}
	}
	else {
		strcpy ( texture_name[0], temp_name );
		if ( debug ) {
			printf ( "      TEXMAP name %s.\n", texture_name[0] );
		}
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}


unsigned long IVCONV::tds_read_material_section ( FILE *filein )

/******************************************************************************/
{
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   teller;
	
	unsigned long current_pointer = ftell ( filein ) - 2;
	
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	
	while ( teller < temp_pointer ) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
			
		case 0xa000:
			if ( debug ) {
				printf ( "     MAT_NAME definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_matdef_section ( filein,&material[material_num].name[0]);
			break;
		case 0xa010:
			if ( debug ) {
				printf ( "     MAT_AMBIENT definition section tag of %0X\n", temp_int );
			}
			teller+=tds_read_color_chunk(filein,material[material_num].ambient);
			break;
		case 0xa020:
			if ( debug ) {
				printf ( "     MAT_DIFFUSE definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_color_chunk(filein,material[material_num].rgb);
			break;
		case 0xa030:
			if ( debug ) {
				printf ( "     MAT_SPECULAR definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_color_chunk(filein,material[material_num].specular);
			break;
		case 0xa040:
			if ( debug ) {
				printf ( "     MAT_SHININESS definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa041:
			if ( debug ) {
				printf ( "     MAT_SHIN2PCT definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa042:
			if ( debug ) {
				printf ( "     MAT_SHIN3PCT definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa050:
			if ( debug ) {
				printf ( "     MAT_TRANSPARENCY definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa052:
			if ( debug ) {
				printf ( "     MAT_XPFALL definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa053:
			if ( debug ) {
				printf ( "     MAT_REFBLUR definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa080:
			if ( debug ) {
				printf ( "     MAT_SELF_ILLUM definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa081:
			if ( debug ) {
				printf ( "     MAT_TWO_SIDE definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa082:
			if ( debug ) {
				printf ( "     MAT_DECAL definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa083:
			if ( debug ) {
				printf ( "     MAT_ADDITIVE definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa084:
			if ( debug ) {
				printf ( "     MAT_SELF_ILPCT definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa085:
			if ( debug ) {
				printf ( "     MAT_WIRE definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa086:
			if ( debug ) {
				printf ( "     MAT_SUPERSMP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa087:
			if ( debug ) {
				printf ( "     MAT_WIRESIZE definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa088:
			if ( debug ) {
				printf ( "     MAT_FACEMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa08a:
			if ( debug ) {
				printf ( "     MAT_XPFALLIN definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa08c:
			if ( debug ) {
				printf ( "     MAT_PHONGSOFT definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa08e:
			if ( debug ) {
				printf ( "     MAT_WIREABS definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa100:
			if ( debug ) {
				printf ( "     MAT_SHADING definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa200:
			if ( debug ) {
				printf ( "     MAT_TEXMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_texmap_section ( filein );
			/*
			teller+=tds_read_unknown_section ( filein );
			*/
			break;
		case 0xa204:
			if ( debug ) {
				printf ( "     MAT_SPECMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa210:
			if ( debug ) {
				printf ( "     MAT_OPACMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa220:
			if ( debug ) {
				printf ( "     MAT_REFLMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa230:
			if ( debug ) {
				printf ( "     MAT_BUMPMAP definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xa353:
			if ( debug ) {
				printf ( "     MAT_MAP_TEXBLUR definition section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default:
			if ( debug ) {
				printf ( "  Junk section tag of %0X\n", temp_int );
			}
			break;	
		}	
	}
	++material_num;

	long pointer = current_pointer + temp_pointer;

	fseek ( filein, pointer, SEEK_SET );

	return ( temp_pointer );
}

unsigned long tds_read_vp_section ( FILE *filein, int *views_read )

/******************************************************************************/
{
	unsigned int       attribs;
	unsigned long  current_pointer;
	int                i;
	int                int_val;
	long           pointer;
	unsigned int       port;
	unsigned long  temp_pointer;
	char              *viewports[11] = {
		"Bogus",
			"Top",
			"Bottom",
			"Left",
			"Right",
			"Front",
			"Back",
			"User",
			"Camera",
			"Light",
			"Disabled"
	};
	
	*views_read = *views_read + 1;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	attribs = tds_read_u_short_int ( filein );
	
	if ( attribs == 3 ) {
		if ( debug ) {
			printf ( "<Snap> active in viewport.\n" );
		}
	}
	
	if ( attribs == 5 ) {
		if ( debug ) {
			printf ( "<Grid> active in viewport.\n" );
		}
	}
	/* 
	Read 5 INTS to get to the viewport information. 
	*/
	for ( i = 1; i < 6; ++i ) {
		tds_read_u_short_int ( filein ); 
	}
	
	port = tds_read_u_short_int ( filein );
	/*
	Find camera section. 
	*/
	if ( ( port == 0xffff ) || ( port == 0 ) ) {
		
		for ( i = 0; i < 12; ++i ) {
			tds_read_u_short_int ( filein );
		}
		
		int_val = tds_read_name (filein );
		
		if ( int_val == -1 ) {
			if ( debug ) {
				printf ( "      No Camera name found\n" );
			}
		}
		
		port = 0x0008;
	}
	
	if ( debug ) {
		printf ( "Reading [%s] information with tag:%d\n", viewports[port], port );
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	
	fseek ( filein, pointer, SEEK_SET ); 
	
	return ( temp_pointer );
}

unsigned long tds_read_view_section ( FILE *filein, int *views_read )
{
	unsigned long   current_pointer;
	bool end_found = false;
	long            pointer;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   teller;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	
	while (!end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
		case 0x7012:
			if ( debug ) {
				printf ( "     VIEWPORT_DATA_3 section tag of %0X\n", temp_int );
			}
			teller+=tds_read_vp_section ( filein, views_read );
			break;
		case 0x7011:
			if ( debug ) {
				printf ( "     VIEWPORT_DATA section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x7020:
			if ( debug ) {
				printf ( "     VIEWPORT_SIZE section tag of %0X\n", temp_int );
			}
			teller+=tds_read_vp_section ( filein, views_read );
			break;
		default:
			break;
		}
		
		if ( teller >= temp_pointer ) {
			end_found = true;
		}
		
		if ( *views_read > 3 ) {
			end_found = true;
		}
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

unsigned long IVCONV::tds_read_edit_section ( FILE *filein, int *views_read )

/******************************************************************************/

/*
Modified:

  18 September 1998
  */
{
	unsigned long   chunk_length;
	unsigned long   current_pointer;
	bool end_found = false;
	long            pointer;
	unsigned long   teller;
	unsigned short int  temp_int;
	
	current_pointer = ftell ( filein ) - 2;
	chunk_length = tds_read_u_long_int ( filein );
	teller = 6;
	
	while ( !end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		if ( debug ) {
			printf ( "    TDS_READ_EDIT_SECTION processing tag %0X\n", temp_int );
		}
		
		switch ( temp_int ) {
		case 0x1100:
			if ( debug ) {
				printf ( "    BIT_MAP section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1201:
			if ( debug ) {
				printf ( "    USE_SOLID_BGND section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1300:
			if ( debug ) {
				printf ( "    V_GRADIENT section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1400:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1420:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1450:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x1500:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2200:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2201:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2210:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2300:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2302:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x3000:
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x2100:
			if ( debug ) {
				printf ( "    AMBIENT_LIGHT section tag of %0X\n", temp_int );
			}
			teller+=tds_read_ambient_section ( filein );
			break;
		case 0x1200:
			if ( debug ) {
				printf ( "    SOLID_BGND section tag of %0X\n", temp_int );
			}
			teller+=tds_read_background_section ( filein );
			break;
		case 0x0100:
			if ( debug ) {
				printf ( "    MASTER_SCALE section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x3d3e:
			if ( debug ) {
				printf ( "    MESH_VERSION section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xafff:
			if ( debug ) {
				printf ( "    MAT_ENTRY section tag of %0X\n", temp_int );
			}
			teller+=tds_read_material_section ( filein );
			break;
		case 0x4000:
			if ( debug ) {
				printf ( "    NAMED_OBJECT section tag of %0X\n", temp_int );
			}
			teller+=tds_read_object_section ( filein );
			break;
		case 0x7001:
			if ( debug ) {
				printf ( "    VIEWPORT_LAYOUT section tag of %0X\n", 
					temp_int );
			}
			teller+=tds_read_view_section ( filein, views_read );
			break;
		case 0x7012:
			if ( debug ) {
				printf ( "    VIEWPORT_DATA_3 section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x7011:
			if ( debug ) {
				printf ( "    VIEWPORT_DATA section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0x7020:
			if ( debug ) {
				printf ( "    VIEWPORT_SIZE section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default:
			if ( debug ) {
				printf ( "    Junk.\n" );
			}
			break;
    }
	
    if ( teller >= chunk_length ) {
		end_found = true;
    }
	
  }
  
  pointer = ( long ) ( current_pointer + chunk_length );
  
  fseek ( filein, pointer, SEEK_SET );
  
  return ( chunk_length );
}

unsigned long IVCONV::tds_read_keyframe_objdes_section ( FILE *filein )

/******************************************************************************/

/*
Modified:

  21 September 1998
  */
{
	unsigned long   chunk_size;
	unsigned long   current_pointer;
	unsigned char       end_found = false;
	long            pointer;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   teller;
	unsigned long   u_long_int_val;
	unsigned short int  u_short_int_val;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	
	while (!end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
		case 0xb011:
			if ( debug ) {
				printf ( "      INSTANCE_NAME section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb010:
			if ( debug ) {
				printf ( "      NODE_HDR section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb020:
			if ( debug ) {
				printf ( "      POS_TRACK_TAG section tag of %0X\n", temp_int );
			}
			chunk_size = tds_read_u_long_int ( filein );
			if ( debug ) {
				printf ( "      chunk_size = %d\n", chunk_size );
			}
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_short_int_val = tds_read_u_short_int ( filein );
			u_long_int_val = tds_read_u_long_int ( filein );
			if ( debug ) {
				printf ( "u_short_int_val = %d\n", u_short_int_val );
				printf ( "u_long_int_val = %d\n", u_long_int_val );
			}
			origin[0] = float_read ( filein );
			origin[1] = float_read ( filein );
			origin[2] = float_read ( filein );
			teller+=32;
			break;
		case 0xb013:
			if ( debug ) {
				printf ( "      PIVOT section tag of %0X\n", temp_int );
			}
			chunk_size = tds_read_u_long_int ( filein );
			pivot[0] = float_read ( filein );
			pivot[1] = float_read ( filein );
			pivot[2] = float_read ( filein );
			teller+=12;
			break;
		case 0xb014:
			if ( debug ) {
				printf ( "      BOUNDBOX section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb015:
			if ( debug ) {
				printf ( "      MORPH_SMOOTH section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb021:
			if ( debug ) {
				printf ( "      ROT_TRACK_TAG section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb022:
			if ( debug ) {
				printf ( "      SCL_TRACK_TAG section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb030:
			if ( debug ) {
				printf ( "      NODE_ID section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default: 
			break;
		}
		
		if ( teller >= temp_pointer ) {
			end_found = true;
		}
		
	}
	
	pointer = ( long ) ( current_pointer+temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}

/******************************************************************************/

unsigned long IVCONV::tds_read_keyframe_section ( FILE *filein, int *views_read )

/******************************************************************************/
{
	unsigned long   current_pointer;
	bool end_found = false;
	long            pointer;
	unsigned short int  temp_int;
	unsigned long   temp_pointer;
	unsigned long   teller;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	teller = 6;
	
	while (!end_found) {
		
		temp_int = tds_read_u_short_int ( filein );
		teller+=2;
		
		switch ( temp_int ) {
		case 0x7001:
			if ( debug ) {
				printf ( "    VIEWPORT_LAYOUT main definition section tag of %0X\n",
					temp_int );
			}
			teller+=tds_read_view_section ( filein, views_read );
			break;
		case 0xb008:
			if ( debug ) {
				printf ( "    KFSEG frames section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb002:
			if ( debug ) {
				printf ( "    OBJECT_NODE_TAG object description section tag of %0X\n",
					temp_int);
			}
			teller+=tds_read_keyframe_objdes_section ( filein );
			break;
		case 0xb009:
			if ( debug ) {
				printf ( "    KFCURTIME section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		case 0xb00a:
			if ( debug ) {
				printf ( "    KFHDR section tag of %0X\n", temp_int );
			}
			teller+=tds_read_unknown_section ( filein );
			break;
		default: 
			break;
		}
		
		if ( teller >= temp_pointer ) {
			end_found = true;
		}
		
	}
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}
/******************************************************************************/


unsigned long IVCONV::tds_read_tex_verts_section ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  TDS_READ_TEX_VERTS_SECTION reads the texture vertex data.
  
	Discussion:
	
	  The texture vertex data seems to be associated with nodes.  This routine
	  distributes that data to vertices (nodes as they make up a particular
	  face).
	  
		Modified:
		
		  02 July 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	unsigned long  current_pointer;
	int                icor3;
	long           pointer;
	unsigned long  temp_pointer;
	unsigned short int n2;
	
	current_pointer = ftell ( filein ) - 2;
	temp_pointer = tds_read_u_long_int ( filein );
	
	pointer = ( long ) ( current_pointer + temp_pointer );
	
	n2 = tds_read_u_short_int ( filein );
	
	for ( icor3 = 0; icor3 < n2; icor3++ ) {
		cor3_tex_uv[icor3][0] = float_read ( filein );
		cor3_tex_uv[icor3][1] = float_read ( filein );
	}
	
	fseek ( filein, pointer, SEEK_SET );
	
	return ( temp_pointer );
}
/******************************************************************************/

/******************************************************************************/

/******************************************************************************/

bool IVCONV::tds_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  TDS_WRITE writes graphics information to a 3D Studio Max 3DS file.
  
	Modified:
	
	  14 October 1998
	  
		Author:
		
		  John Burkardt
		  
			*/
{
	float               float_val;
	int                 i;
	int                 icor3;
	int                 iface;
	int                 j;
	long            l0002;
	long            l0100;
	long            l3d3d;
	long            l3d3e;
	long            l4000;
	long            l4100;
	long            l4110;
	long            l4120;
	long            l4150;
	long            l4160;
	long            l4d4d;
	long            lb000;
	long            lb002;
	long            lb00a;
	long            lb008;
	long            lb009;
	long            lb010;
	long            lb013;
	long            lb020;
	long            lb021;
	long            lb022;
	long            lb030;
	long            long_int_val;
	int                 name_length;
	short int           short_int_val;
	unsigned short int  u_short_int_val;
	
	tds_pre_process();

	bytes_num = 0;
	name_length = strlen ( object_name );
	
	l0002 = 10;
	
	l4150 = 2 + 4 + face_num * 4;
	l4120 = 2 + 4 + 2 + 4 * face_num * 2 + l4150;
	l4160 = 2 + 4 + 4 * 12;
	l4110 = 2 + 4 + 2 + cor3_num * 3 * 4;
	l4100 = 2 + 4 + l4110 + l4160 + l4120;
	l4000 = 2 + 4 + ( name_length + 1 ) + l4100;
	l0100 = 2 + 4 + 4;
	l3d3e = 2 + 4 + 4;
	l3d3d = 2 + 4 + l3d3e + l0100 + l4000;
	
	lb022 = 2 + 4 + 32;
	lb021 = 2 + 4 + 9 * 4;
	lb020 = 2 + 4 + 8 * 4;
	lb013 = 2 + 4 + 6 * 2;
	lb010 = 2 + 4 + ( name_length + 1 ) + 3 * 2;
	lb030 = 2 + 4 + 2;
	lb002 = 2 + 4 + lb030 + lb010 + lb013 + lb020 + lb021 + lb022;
	lb009 = 2 + 4 + 4;
	lb008 = 2 + 4 + 2 * 4;
	lb00a = 2 + 4 + 2 + 9 + 2 * 2;
	lb000 = 2 + 4 + lb00a + lb008 + lb009 + lb002;
	
	l4d4d = 2 + 4 + l0002 + l3d3d + lb000;
	/*  
	M3DMAGIC begin.
    tag, size.
	*/
	short_int_val = ( short ) 0x4d4d;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4d4d );
	/*
	M3D_VERSION begin.
    tag, size, version.
	*/
	short_int_val = ( short ) 0x0002;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l0002 );
	long_int_val = 3;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	/*  
	M3D_VERSION end.
	MDATA begin.
    tag, size.
	*/
	short_int_val = ( short ) 0x3d3d;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l3d3d );
	/*  
	MESH_VERSION begin.
    tag, size, version.
	*/
	short_int_val = ( short ) 0x3d3e;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l3d3e );
	long_int_val = 3;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	/*  
	MESH_VERSION end.  
	MASTER_SCALE begin.  
    tag, size, scale.
	*/
	short_int_val = ( short ) 0x0100;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l0100 );
	float_val = 1.0;
	bytes_num = bytes_num + float_write ( fileout, float_val );
	/*
	MASTER_SCALE end. 
	NAMED_OBJECT begin. 
    tag, size, name. 
	*/
	short_int_val = ( short ) 0x4000;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4000 );
	bytes_num = bytes_num + tds_write_string ( fileout, object_name );
	/*  
	N_TRI_OBJECT begin.  
    tag, size.
	*/
	short_int_val = ( short ) 0x4100;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4100 );
	/*
	POINT_ARRAY begin.  
    tag, size, number of points, coordinates of points.
	Warning! number of points could exceed a short!
	*/
	short_int_val = ( short ) 0x4110;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4110 );
	
	u_short_int_val = ( unsigned short ) cor3_num;
	bytes_num = bytes_num + tds_write_u_short_int ( fileout, u_short_int_val );
	
	for ( icor3 = 0; icor3 < cor3_num; icor3++ ) {
		for ( j = 0; j < 3; ++j ) {
			bytes_num = bytes_num + float_write ( fileout, cor3[icor3][j] );
		}
	}
	/*
	POINT_ARRAY end.
	MESH_MATRIX begin.  
    tag, size, 4 by 3 matrix.
	*/
	short_int_val = ( short ) 0x4160;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4160 );
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 3; ++j ) {
			float_val = transform_matrix[i][j];
			bytes_num = bytes_num + float_write ( fileout, float_val );
		}
	}
	/*
	MESH_MATRIX end.  
	FACE_ARRAY begin. 
    tag, size, number of faces, nodes per face. 
	Warning: number of faces could exceed a short!
	*/
	short_int_val = ( short ) 0x4120;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4120 );
	
	u_short_int_val = ( unsigned short ) face_num;
	bytes_num = bytes_num + tds_write_u_short_int ( fileout, u_short_int_val );
	
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( j = 0; j < 3; ++j ) {
			short_int_val = face[j][iface];
			bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
		}
		short_int_val = face_flags[iface];
		bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	}
	/*
	SMOOTH_GROUP begin.
    tag, size, group for each face.
	*/
	short_int_val = ( short ) 0x4150;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, l4150 );
	
	for ( iface = 0; iface < face_num; iface++ ) {
		long_int_val = face_smooth[iface];
		bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	}
	/*
	SMOOTH_GROUP end.
	FACE_ARRAY end.
	N_TRI_OBJECT end.
	NAMED_OBJECT end.
	MDATA end. 
	KFDATA begin.
	*/
	short_int_val = ( short ) 0xb000;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb000 );
	/*
	KFHDR begin.  
    tag, size, revision, filename, animlen.
	*/
	short_int_val = ( short ) 0xb00a;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb00a );
	short_int_val = 5;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + tds_write_string ( fileout, "MAXSCENE" );
	short_int_val = 100;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	/*
	KFHDR end.  
	KFSEG begin.  
    tag, size, start, end.
	*/
	short_int_val = ( short ) 0xb008;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb008 );
	long_int_val = 0;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	long_int_val = 100;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	/*
	KFSEG end.  
	KFCURTIME begin.
    tag, size, current_frame.
	*/
	short_int_val = ( short ) 0xb009;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb009 );
	long_int_val = 0;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	/*
	KFCURTIME end.
	OBJECT_NODE_TAG begin.
    tag, size.  
	*/
	short_int_val = ( short ) 0xb002;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb002 );
	/*
	NODE_ID begin.
    tag, size, id.
	*/
	short_int_val = ( short ) 0xb030;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb030 );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	/*
	NODE_ID end.  
	NODE_HDR begin. 
    tag, size, object_name, flag1, flag2, hierarchy.
	*/
	short_int_val = ( short ) 0xb010;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb010 );
	bytes_num = bytes_num + tds_write_string ( fileout, object_name );
	short_int_val = 16384;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = -1;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	/*
	NODE_HDR end. 
	PIVOT begin. 
    tag, size, pivot_x, pivot_y, pivot_z.
	*/
	short_int_val = ( short ) 0xb013;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb013 );
	for ( i = 0; i < 3; ++i ) {
		float_val = pivot[i];
		bytes_num = bytes_num + float_write ( fileout, float_val );
	}
	/*
	PIVOT end. 
	POS_TRACK_TAG begin.  
    tag, size, flag, i1, i2, i3, i4, i5, i6, frame, l1, pos_x, pos_y, pos_z.
	*/
	short_int_val = ( short ) 0xb020;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb020 );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 1;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	long_int_val = 0;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	for ( i = 0; i < 3; ++i ) {
		float_val = origin[i];
		bytes_num = bytes_num + float_write ( fileout, float_val );
	}
	/*
	POS_TRACK_TAG end. 
	ROT_TRACK_TAG begin. 
    tag, size, i1, i2, i3, i4, i5, i6, i7, i8, l1, rad, axis_x, axis_y, axis_z. 
	*/
	short_int_val = ( short ) 0xb021;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb021 );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 1;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	long_int_val = 0;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	float_val = 0.0;
	bytes_num = bytes_num + float_write ( fileout, float_val );
	bytes_num = bytes_num + float_write ( fileout, float_val );
	bytes_num = bytes_num + float_write ( fileout, float_val );
	bytes_num = bytes_num + float_write ( fileout, float_val );
	/*
	ROT_TRACK_TAG end. 
	SCL_TRACK_TAG begin.  
    tag, size, i1, i2, i3, i4, i5, i6, i7, i8, l1, scale_x, scale_y, scale_z.
	*/
	short_int_val = ( short ) 0xb022;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	bytes_num = bytes_num + long_int_write ( fileout, lb022 );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 1;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	short_int_val = 0;
	bytes_num = bytes_num + short_int_write ( fileout, short_int_val );
	long_int_val = 0;
	bytes_num = bytes_num + long_int_write ( fileout, long_int_val );
	float_val = 1.0;
	bytes_num = bytes_num + float_write ( fileout, float_val );
	bytes_num = bytes_num + float_write ( fileout, float_val );
	bytes_num = bytes_num + float_write ( fileout, float_val );
	/*  
	SCL_TRACK_TAG end.
	OBJECT_NODE_TAG end.
	KFDATA end.
	M3DMAGIC end. 
	*/
	
	/*
	Report.
	*/
	printf ( "TDS_WRITE wrote %d bytes.\n", bytes_num );
	
	return true;
}
/******************************************************************************/

/******************************************************************************/

bool IVCONV::tds_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  TDS_READ reads a 3D Studio MAX binary 3DS file.
  
	Modified:
	
	  20 October 1998
	  
		Author:
		
		  John Burkardt
		  */
{
	unsigned long   chunk_begin;
	unsigned long   chunk_end;
	unsigned long   chunk_length;
	unsigned long   chunk_length2;
	unsigned long   position;
	unsigned short int  temp_int;
	int                 version;
	int                 views_read;
	/* 
	Initialize.
	*/
	views_read = 0;
	
	temp_int = tds_read_u_short_int ( filein );
	
	if ( temp_int == 0x4d4d ) {
		
		if ( debug ) {
			printf ( "TDS_READ: DEBUG: Read magic number %0X.\n", temp_int );
		}
		/* 
		Move to 28 bytes from the beginning of the file. 
		*/
		position = 28;
		fseek ( filein, ( long ) position, SEEK_SET );
		version = fgetc ( filein );
		
		if ( version < 3 ) {
			printf ( "\n" );
			printf ( "TDS_READ - Fatal error!\n" );
			printf ( "  This routine can only read 3DS version 3 or later.\n" );
			printf ( "  The input file is version %d.\n" ,version );
			return false;
		}
		
		if ( debug ) {
			printf ( "TDS_READ: DEBUG: Version number is %d.\n", version );
		}
		/* 
		Move to 2 bytes from the beginning of the file. 
		Set CURRENT_POINTER to the first byte of the chunk.
		Set CHUNK_LENGTH to the number of bytes in the chunk.
		*/
		chunk_begin = 0;
		position = 2;
		fseek ( filein, ( long ) position, SEEK_SET );
		
		chunk_length = tds_read_u_long_int ( filein );
		position = 6;
		
		chunk_end = chunk_begin + chunk_length;
		
		if ( debug ) {
			printf ( "TDS_READ:\n" );
			printf ( "  Chunk begin  = %lu.\n", chunk_begin );
			printf ( "  Chunk length = %lu.\n", chunk_length );
			printf ( "  Chunk end    = %lu.\n", chunk_end );
		}
		
		while ( position + 2 < chunk_end ) {
			
			temp_int = tds_read_u_short_int ( filein );
			position = position + 2;
			
			if ( debug ) {
				printf ( "TDS_READ: Short int = %0X, position = %lu.\n", temp_int, position );
			}
			
			if ( temp_int == 0x0002 ) {
				if ( debug ) {
					printf ( "TDS_READ: Read_Initial_Section:\n" );
				}
				chunk_length2 = tds_read_u_long_int ( filein );
				position = position + 4;
				position = position - 6 + chunk_length2;
				fseek ( filein, ( long ) position, SEEK_SET );
			}
			else if ( temp_int == 0x3d3d ) {
				if ( debug ) {
					printf ( "TDS_READ: Read_Edit_Section:\n" );
				}
				position = position - 2;
				position = position + tds_read_edit_section ( filein, &views_read );
			}
			else if ( temp_int == 0xb000 ) {
				if ( debug ) {
					printf ( "TDS_READ: Read_Keyframe_Section:\n" );
				}
				
				position = position - 2;
				position = position + tds_read_keyframe_section ( filein, &views_read );
			}
			else {
				printf ( "\n" );
				printf ( "TDS_READ - Fatal error!\n" );
				printf ( "  Unexpected input, position = %lu.\n", position );
				printf ( "  TEMP_INT = %hux\n", temp_int );
				return false;
			}
		}
		position = chunk_begin + chunk_length;
		fseek ( filein, ( long ) position, SEEK_SET );
	}
	else {
		printf ( "\n" );
		printf ( "TDS_READ - Fatal error!\n" );
		printf ( "  Could not find the main section tag.\n" );
		return false;
	}
	
	return true;
}