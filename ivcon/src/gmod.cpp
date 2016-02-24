#include "ivconv.h"


const int GMOD_MAX_SECTIONS=32;
const int GMOD_UNUSED_VERTEX=65535;
const int G1_SECTION_MODEL_QUADS=18;
const int G1_SECTION_MODEL_TEXTURE_NAMES=19;
const int G1_SECTION_MODEL_VERT_ANIMATION=20;

bool gmod_arch_check ( )

/******************************************************************************/

/*
Purpose:

  GMOD_ARCH_CHECK inquires into some features of the computer architecture.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	static unsigned char one[4];
	int temp;
	
	temp = sizeof ( float );
	if ( temp != 4 ) {
		return false;
	}
	
	*(float *)one = 1.0;
	
	if (one[0] == 0 && one[1] == 0 && one[2] == 128 && one[3] == 63) {
		/* little endian IEEE floats */
		return true;
	}
	
	if (one[0] == 63 && one[1] == 128 && one[2] == 0 && one[3] == 0) {
		/* big endian IEEE floats */
		return true;
	}
	
	return false;
}

float gmod_read_float ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  GMOD_READ_FLOAT reads a float from a Golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	int endian = 1;
	unsigned char *out_pos;
	int i;
	float Val;
	
	if (*(char *)&endian == 1) {
		/* we're little-endian, which is native for GMOD floats */
		fread(&Val, sizeof(Val), 1, filein);
	}
	else {
		/* we're big-endian, flip `em */
		out_pos = (unsigned char *)&Val;
		for ( i = sizeof(Val)-1; i >= 0; i-- ) {
			*(out_pos+i) = fgetc(filein);
		}
	}
	
	return Val;
}
/******************************************************************************/

unsigned short gmod_read_w16 ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  GMOD_READ_W16 reads a 16 bit word from a Golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	unsigned char Byte1;
	unsigned char Byte2;
	
	Byte1 = fgetc ( filein );
	Byte2 = fgetc ( filein );
	
	return Byte1 | (((unsigned short)Byte2) << 8);
}
/******************************************************************************/

unsigned long gmod_read_w32 ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  GMOD_READ_W32 reads a 32 bit word from a Golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	unsigned char Byte1, Byte2, Byte3, Byte4;
	
	Byte1 = fgetc(filein);
	Byte2 = fgetc(filein);
	Byte3 = fgetc(filein);
	Byte4 = fgetc(filein);
	
	return Byte1 | 
		(((unsigned long)Byte2) << 8) | 
		(((unsigned long)Byte3) << 16) | 
		(((unsigned long)Byte4) << 24);
}

/******************************************************************************/


void gmod_write_float ( float Val, FILE *fileout )

/******************************************************************************/

/*
Purpose:

  GMOD_WRITE_FLOAT writes a float to a Golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	int endian = 1;
	unsigned char *out_pos;
	int i;
	
	if (*(char *)&endian == 1) {
		/* we're little-endian, which is native for GMOD floats */
		fwrite ( &Val, sizeof(Val), 1, fileout );
	}
	else {
		/* we're big-endian, flip `em */
		out_pos = (unsigned char *)&Val;
		for ( i = sizeof(Val)-1; i >= 0; i-- ) {
			fputc(*(out_pos+i), fileout);
		}
	}
}
/******************************************************************************/

void gmod_write_w16 ( unsigned short Val, FILE *fileout )

/******************************************************************************/

/*
Purpose:

  GMOD_WRITE_W16 writes a 16 bit word to a Golgotha GMOD file.
  
	Modified:
	
	  13 September 2000
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	unsigned char OutByte[2];
	
	OutByte[0] = (unsigned char)(Val & 0xff);
	OutByte[1] = (unsigned char)(Val >> 8);
	
	fwrite ( OutByte, sizeof(unsigned char), 2, fileout );
}
/******************************************************************************/

void gmod_write_w32 ( unsigned long Val, FILE *fileout )

/******************************************************************************/

/*
Purpose:

  GMOD_WRITE writes a 32 bit word to a Golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
{
	unsigned char OutByte[4];
	
	OutByte[0] = (unsigned char)(Val & 0xff);
	OutByte[1] = (unsigned char)((Val >> 8) & 0xff);
	OutByte[2] = (unsigned char)((Val >> 16) & 0xff);
	OutByte[3] = (unsigned char)((Val >> 24) & 0xff);
	
	fwrite ( OutByte, sizeof(unsigned char), 4, fileout );
}

/******************************************************************************/

bool IVCONV::gmod_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  GMOD_READ reads a golgotha GMOD file.
  
	Modified:
	
	  19 May 1999
	  
		Author:
		
		  Zik Saleeba (zik@zikzak.net)
		  */
		  
		  /*
		  golgotha GMOD file format:
		  
			
			  FILE HEADER
			  
				w32     magic number           f9 fa 63 1e
				w32     number of sections
				[ number of sections
				w32     section id
				w32     section offset
				]
				
				  
					TEXTURE NAME SECTION - section id = 0x13 (19)
					
					  w16     number of faces
					  [ number of faces
					  w16     texture name length
					  [ texture name length
					  w8      texture name character
					  ]
					  ]
					  
						
						  
							MODEL QUADS SECTION - section id = 0x12 (18)
							
							  w16     number of faces
							  [ number of faces
							  [ four vertices
							  w16     vertex index
							  float   xpos (0.0-1.0)
							  float   ypos (0.0-1.0)
							  ]
							  float   scale
							  w16     flags
							  float   xnormal     (normal should be normalised)
							  float   ynormal
							  float   znormal
							  ]
							  
								
								  VERTEX ARRAY SECTION - section id = 0x14 (20)
								  
									w16     number of vertices
									w16     number of animations
									w16     length of animation name
									[ length of animation name
									w8      animation name character
									]
									w16     number of frames in animation
									[ number of frames in animation
									[ number of vertices
									float   xpos
									float   ypos
									float   zpos
									float   xnormal
									float   ynormal
									float   znormal
									]
									]
									*/
{
	unsigned char MagicNumber[4];
	unsigned long NumSections;
	int SectionCount;
	unsigned long SectionID[GMOD_MAX_SECTIONS];
	unsigned long SectionOffset[GMOD_MAX_SECTIONS];
	
	unsigned short NumAnimations;
	unsigned short NumFrames;
	unsigned short FaceCount;
	unsigned short TextureCount;
	int VertexCount;
	
	float Scale;
	unsigned short Flags;
	unsigned short TextureNameLen;
	unsigned short AnimationNameLen;
	int Order;
	int MaxCor = 0;
	
	/*
	* check if we can handle this architecture
	*/
	
	if (!gmod_arch_check()) {
		printf("GMOD_READ - This architecture not supported.\n");
		return false;
	}
	
	/* 
	* read the file header 
	*/
	
	/* read the magic number */
	fread(MagicNumber, 1, 4, filein);
	if (MagicNumber[0] != 0xf9 || 
		MagicNumber[1] != 0xfa || 
		MagicNumber[2] != 0x63 || 
		MagicNumber[3] != 0x1e) {
		printf("GMOD_READ - Bad magic number on GMOD file.\n");
		return false;
	}
	
	NumSections = gmod_read_w32(filein);
	if (NumSections >= GMOD_MAX_SECTIONS) {
		printf("GMOD_READ - Too many sections (%ld) in GMOD file - please increase static limit GMOD_MAX_SECTIONS\n", NumSections);
		return false;
	}
	
	/* 
	Read the sections.
	*/
	
	for ( SectionCount = 0; SectionCount < ( int ) NumSections; SectionCount++ ) {
		SectionID[SectionCount] = gmod_read_w32(filein);
		SectionOffset[SectionCount] = gmod_read_w32(filein);
	}
	/* 
	Read each successive section.
	*/
	for ( SectionCount = 0; SectionCount < ( int ) NumSections; SectionCount++ ) {
	/* 
	Go to the start of the section.
		*/
		fseek ( filein, ( long ) SectionOffset[SectionCount], SEEK_SET );
		/* 
		What type of section is it?
		*/
		switch (SectionID[SectionCount]) {
			
		/*
		Model section.
			*/
		case G1_SECTION_MODEL_QUADS:
		/* 
		Get the number of faces.
			*/
			face_num = gmod_read_w16 ( filein );
			
			/*
			if (face_num > FACE_MAX) {
				printf("GMOD_READ - Too many faces (%d) in GMOD file - please increase static limit FACE_MAX.\n", face_num);
				return false;
			}
			*/
			/* 
			Get the information on each face.
			*/
			for ( FaceCount = 0; FaceCount < ( unsigned short ) face_num; FaceCount++ ) {
				
				Order = 0;
				for ( VertexCount = 0; VertexCount < 4; VertexCount++ ) {
					
					/* read the vertex index */
					
					face[VertexCount][FaceCount] = gmod_read_w16(filein);
					
					if (face[VertexCount][FaceCount] != GMOD_UNUSED_VERTEX) {
						Order = VertexCount+1;
						if (MaxCor < face[VertexCount][FaceCount])
							MaxCor = face[VertexCount][FaceCount];
					}
					
					/* read the texture position */
					
					vertex_tex_uv[FaceCount][VertexCount][0] = gmod_read_float(filein);
					vertex_tex_uv[FaceCount][VertexCount][1] = gmod_read_float(filein);
				}
				
				/* scale and flags */
				
				fread(&Scale, sizeof(Scale), 1, filein);
				Flags = gmod_read_w16(filein);
				
				if ( debug ) {
					printf ( "Flags = %d\n", Flags );
				}
				
				/* normal vector */
				
				face_normal[FaceCount][0] = gmod_read_float(filein);
				face_normal[FaceCount][1] = gmod_read_float(filein);
				face_normal[FaceCount][2] = gmod_read_float(filein);
				
				/* the order is the number of used vertices */
				
				face_order[FaceCount] = Order;
			}
			break;
			
			
			/*
			Texture name section.
			*/
			
		case G1_SECTION_MODEL_TEXTURE_NAMES:
			
			/* get the number of textures */
			
			texture_num = gmod_read_w16(filein);
			if (texture_num > TEXTURE_MAX) {
				printf ( "GMOD_READ - Too many texture maps (%d) in GMOD file.\n", texture_num );
				printf ( "  Increase static limit TEXTURE_MAX.\n" );
				return false;
			}
			face_num = texture_num;
			
			for (TextureCount = 0; TextureCount < ( unsigned short ) texture_num; 
			TextureCount++) {
				
				/* read the texture name */
				
				TextureNameLen = gmod_read_w16(filein);
				fread ( texture_name[TextureCount], sizeof(char), TextureNameLen, filein);
				texture_name[TextureCount][TextureNameLen] = '\0';
			}
			break;
			
			
			/*
			* vertex section
			*/
			
		case G1_SECTION_MODEL_VERT_ANIMATION:
			
			/* get the number of vertices */
			
			cor3_num = gmod_read_w16(filein);

			/* todo : reserve the correct size in arrays
			if (cor3_num > COR3_MAX) {
				printf("GMOD_READ - Too many vertices (%d) in GMOD file - please increase static limit COR3_MAX.\n", cor3_num);
				return false;
			}
			*/
			/* 
			Get the number of animations.
			*/
			
			NumAnimations = gmod_read_w16(filein);
			
			if (NumAnimations > 1) {
				printf ( "GMOD_READ - Fatal error!\n" );
				printf ( "  GMOD files can only handle one animation.\n" );
				printf ( "  This file contains %d.\n", NumAnimations );
				return false;
			}
			
			/* read the animation name */
			AnimationNameLen = gmod_read_w16(filein);
			char anim_name[LINE_MAX_LEN];
			fread ( anim_name, sizeof(char), AnimationNameLen, filein);
			anim_name[AnimationNameLen] = '\0';
			
			/* get the number of frames of animation */
			NumFrames = gmod_read_w16(filein);
			if (NumFrames > 1)
				printf("GMOD_READ - Too many frames of animation (%d) in GMOD file - will only use 1.\n", NumFrames);
			
			/* go through all the vertices, reading each one */
			for (VertexCount = 0; VertexCount < cor3_num; VertexCount++) {
				
				/* read the vertex */
				cor3[VertexCount][0] = gmod_read_float(filein);
				cor3[VertexCount][1] = gmod_read_float(filein);
				cor3[VertexCount][2] = gmod_read_float(filein);
				
				/* read the normal */
				cor3_normal[VertexCount][0] = gmod_read_float(filein);
				cor3_normal[VertexCount][1] = gmod_read_float(filein);
				cor3_normal[VertexCount][2] = gmod_read_float(filein);
			}
			break;
			
		default:
			continue;
    }
  }
  
  /* 
  Set some other stray info.
  */
  line_num = 0;
  
  /* 
  Check for sanity.
  */
  if ( MaxCor >= cor3_num ) {
	  printf ( "GMOD_READ - Maximum coordinate index (%d)\n", MaxCor );
	  printf ( "  exceeded number of coordinates (%li) in GMOD file.\n", cor3_num );
	  return false;
  }
  
  return true;
}
/******************************************************************************/
bool IVCONV::gmod_write ( FILE *fileout ) {
	
	/******************************************************************************/
	
	/*
	Purpose:
	
	  GMOD_WRITE writes a Golgotha GMOD file.
	  
		Modified:
		
		  19 May 1999
		  
			Author:
			
			  Zik Saleeba (zik@zikzak.net)
	*/
	static unsigned char MagicNumber[4] = { 0xf9, 0xfa, 0x63, 0x1e };
	unsigned long NumSections;
	unsigned long SectionHeaderPos;
	unsigned long TextureNameSectionPos;
	unsigned long ModelSectionPos;
	unsigned long VertexSectionPos;
	
	int VertexCount;
	int FaceCount;
	int TextureCount;
	unsigned long SectionCount;
	float Scale;  
	float Min[3];
	float Max[3];
	int CorNumber;
	int DimensionCount;
	float MaxWidth;
	/*
	Check if we can handle this architecture.
	*/
	
	if ( !gmod_arch_check() ) {
		printf("GMOD_WRITE - This architecture not supported.\n");
		return false;
	}
	
	/* 
	Write the file header.
	*/
	
	/* 
	Write the magic number.
	*/
	fwrite ( MagicNumber, sizeof(char), 4, fileout );
	
	/* 
	Write the number of sections.
	*/ 
	NumSections = 3;
	gmod_write_w32 ( NumSections, fileout ); 
	
	/* 
	Write a dummy section header which we'll overwrite later.
	*/
	SectionHeaderPos = ftell ( fileout );
	for (SectionCount = 0; SectionCount < NumSections; SectionCount++) {
		gmod_write_w32 ( 0, fileout );
		gmod_write_w32 ( 0, fileout );
	}  
	/*
	Texture name section.
	*/
	
	/* 
	Take note of where we are in the file.
	*/
	TextureNameSectionPos = ftell ( fileout );
	
	/* 
	Write the number of textures.
	*/
	
	gmod_write_w16 ( ( unsigned short ) face_num, fileout );
	/*
	Write the texture names.
	*/
	for ( TextureCount = 0; TextureCount < face_num; TextureCount++ ) {
		
		gmod_write_w16 ( ( unsigned short ) strlen ( texture_name[TextureCount] ), 
			fileout );
		
		fwrite ( texture_name[TextureCount], strlen ( texture_name[TextureCount] ), 
			1, fileout );
	}
	
	/*
	Model section.
	*/
	
	/* 
	Take note of where we are in the file.
	*/
	
	ModelSectionPos = ftell(fileout);
	
	/* 
	Write the number of faces.
	*/
	
	gmod_write_w16 ( ( unsigned short ) face_num, fileout );
	
	/* 
	Write the information on each face.
	*/
	
	for ( FaceCount = 0; FaceCount < face_num; FaceCount++ ) {
		
		for (VertexCount = 0; VertexCount < ((face_order[FaceCount] < 4) ? face_order[FaceCount] : 4); VertexCount++) {
			
		/* 
		Write the vertex index.
			*/
			gmod_write_w16 ( ( unsigned short ) face[VertexCount][FaceCount], fileout );
			
			/* 
			Write the texture position.
			*/
			
			gmod_write_float ( vertex_tex_uv[FaceCount][VertexCount][0], fileout );
			gmod_write_float ( vertex_tex_uv[FaceCount][VertexCount][1], fileout );
		}
		
		/* 
		Write any extra vertices which are unused.
		*/
		
		for ( ; VertexCount < 4; VertexCount++ ) {
			
		/* 
		Write the vertex index.
			*/
			gmod_write_w16 ( GMOD_UNUSED_VERTEX, fileout );
			/* 
			Write the texture position.
			*/
			gmod_write_float ( vertex_tex_uv[FaceCount][VertexCount][0], fileout );
			
			gmod_write_float ( vertex_tex_uv[FaceCount][VertexCount][1], fileout );
		}
		
		/*
		Scale and flags.
		*/
		
		/* 
		Find the bounding box.
		*/
		
		for ( DimensionCount = 0; DimensionCount < 3; DimensionCount++ ) {
			
			CorNumber = face[0][FaceCount];
			Min[DimensionCount] = cor3[CorNumber][DimensionCount];
			Max[DimensionCount] = cor3[CorNumber][DimensionCount];
			
			for (VertexCount = 1; VertexCount < ((face_order[FaceCount] < 4) ? face_order[FaceCount] : 4); VertexCount++) {
				
				CorNumber = face[VertexCount][FaceCount];
				
				if (Min[DimensionCount] > cor3[CorNumber][DimensionCount])
					Min[DimensionCount] = cor3[CorNumber][DimensionCount];
				
				if (Max[DimensionCount] < cor3[CorNumber][DimensionCount])
					Max[DimensionCount] = cor3[CorNumber][DimensionCount];
			}
		}
		
		/* 
		The scale is the "width" of the face for mipmapping - 
		I just take the maximum bounding box dimension.
		*/
		MaxWidth = Max[0] - Min[0];
		for ( DimensionCount = 1; DimensionCount < 3; DimensionCount++ ) {
			
			if ( MaxWidth < Max[DimensionCount] - Min[DimensionCount] )
				MaxWidth = Max[DimensionCount] - Min[DimensionCount];
		}
		
		Scale = MaxWidth;
		fwrite ( &Scale, sizeof(Scale), 1, fileout );
		
		/* 
		Flags are just nothing.
		*/
		gmod_write_w16 ( 0, fileout );
		/* 
		Normal vector.
		*/
		gmod_write_float ( face_normal[FaceCount][0], fileout );
		gmod_write_float ( face_normal[FaceCount][1], fileout );
		gmod_write_float ( face_normal[FaceCount][2], fileout );
	}
	
	/*
	Vertex section.
	*/
	
	/* 
	Take note of where we are in the file.
	*/
	
	VertexSectionPos = ftell ( fileout );
	
	/* 
	Write the number of vertices.
	*/
	
	gmod_write_w16 ( ( unsigned short ) cor3_num, fileout );
	
	/* 
	Write the number of animations.
	*/
	
	gmod_write_w16 ( 1, fileout );
	
	/* 
	Write the animation name. 
	*/
	
	gmod_write_w16 ( 0, fileout );
	
	/* 
	Write the number of frames of animation.
	*/
	
	gmod_write_w16 ( 1, fileout );
	
	/* 
	Go through all the vertices, writing each one.
	*/
	
	for ( VertexCount = 0; VertexCount < cor3_num; VertexCount++ ) {
		
	/* 
	Write the vertex.
		*/
		gmod_write_float ( cor3[VertexCount][0], fileout );
		gmod_write_float ( cor3[VertexCount][1], fileout );
		gmod_write_float ( cor3[VertexCount][2], fileout );
		
		/* 
		Write the normal.
		*/
		gmod_write_float ( cor3_normal[VertexCount][0], fileout );
		gmod_write_float ( cor3_normal[VertexCount][1], fileout );
		gmod_write_float ( cor3_normal[VertexCount][2], fileout );
	}
	/*
	Now rewrite the section header.
	*/
	
	/* 
	Go back to the section header.
	*/
	fseek ( fileout, ( long ) SectionHeaderPos, SEEK_SET );
	
	/* 
	Write the texture name section header.
	*/
	gmod_write_w32 ( G1_SECTION_MODEL_TEXTURE_NAMES, fileout );
	gmod_write_w32 ( TextureNameSectionPos, fileout );
	
	/* 
	Write the model section header.
	*/
	gmod_write_w32 ( G1_SECTION_MODEL_QUADS, fileout );
	gmod_write_w32 ( ModelSectionPos, fileout );
	
	/* 
	Write the vertex section header.
	*/
	gmod_write_w32 ( G1_SECTION_MODEL_VERT_ANIMATION, fileout );
	gmod_write_w32 ( VertexSectionPos, fileout );
	
	return true;
}

/******************************************************************************/
