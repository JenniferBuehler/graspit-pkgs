#include "ivconv.h"
#include <ros/ros.h>

/* 
	19 March 2003		PhG tweaked IV_READ to accept also VRML wrl files
	16 March 2003		PhG corrected materials in 3DS_READ and WRL_WRITE
						ASC support added for point cloud
	23 December 2002	Philippe Guglielmetti (PhG) divided in smaller files
						PhG started conversion to C++/STL (IVCONV class)
						PhG made some variables local instead of global
						PhG changes news() to this changes file : any need to be interactive ???
						PhG added IVCONV::recompute_normals()
						PhG added bool IVCONV::reverse_normals()
	04 July 2000		Added preliminary XGL_WRITE.
	26 September 1999	After ASE_READ, call NODE_TO_VERTEX_MAT and VERTEX_TO_FACE_MATERIAL.
	27 July 1999		Corrected TMAT_ROT_VECTOR.
	17 July 1999		Added null edge and face deletion.
						Corrected a string problem in SMF_READ.
	03 July 1999		Fixed a problem with BINDING variables in SMF_READ.
	02 July 1999		Added limited texture support in 3DS/iv.
	26 June 1999		BYU_READ added.
	25 June 1999		BYU_WRITE added.
	22 June 1999		TRIB_READ added.
	16 June 1999		TRIB_WRITE Greg Hood binary triangle output routine added.
	10 June 1999		TRIA_WRITE Greg Hood ASCII triangle output routine added.
	09 June 1999		TEC_WRITE TECPLOT output routine added.
						IV_READ and IV_WRITE use TRANSFORM_MATRIX now.
	26 May 1999			LINE_PRUNE option added for VLA_WRITE.
	24 May 1999			Added << command to append new graphics data to old.
						Stuck in first draft STLB_READ/STLB_WRITE routines.
						STLA_WRITE and STLB_WRITE automatically decompose non-triangular faces before writing.
	23 May 1999			Stuck in first draft WRL_WRITE routine.
	22 May 1999			Faces converted to lines before calling VLA_WRITE.	
						Added UCD_WRITE.
						Added MATERIAL/PATCH/TAGGEDPOINTS fields in HRC_READ.
	17 May 1999			Updated SMF_WRITE, SMF_READ to match code in IVREAD.
						Added transformation matrix routines.
	16 May 1999			Zik Saleeba improved DXF support to handle polygons.
	15 April 1999		Zik Saleeba added Golgotha GMOD file format support.
	03 December 1998	Set up simple hooks in TDS_READ_MATERIAL_SECTION.
	02 December 1998	Set up simple hooks for texture map names.
	19 November 1998	IV_WRITE uses PER_VERTEX normal binding.
	18 November 1998	Added node normals.
						Finally added the -RN option.
	17 November 1998	Added face node ordering reversal option.
	20 October 1998		Added DATA_REPORT.
	19 October 1998		SMF_READ and SMF_WRITE added.
	16 October 1998		Fixing a bug in IV_READ that chokes on ]} and other	cases where brackets aren't properly spaced.
	11 October 1998		Added face subset selection option S.
	09 October 1998		Reworking normal vector treatments.
						Synchronizing IVREAD and IVCON.		
						POV_WRITE added.
	02 October 1998		IVCON reproduces BOX.3DS and CONE.3DS exactly.
	30 September 1998	IVCON compiled on the PC.
						Interactive BYTE_SWAP option added for binary files.
	25 September 1998	OBJECT_NAME made available to store object name.
	23 September 1998	3DS binary files can be written.
	15 September 1998	3DS binary files can be read.
	01 September 1998	COR3_RANGE, FACE_NORMAL_AVE added.
						Major modifications to normal vectors.
	24 August 1998		HRC_READ added.
	21 August 1998		TXT_WRITE improved.
	20 August 1998		HRC_WRITE can output lines as linear splines.
	19 August 1998		Automatic normal computation for OBJ files.
						Added normal vector computation.
						HRC_WRITE is working.
	18 August 1998		IV_READ/IV_WRITE handle BASECOLOR RGB properly now.
						Improved treatment of face materials and normals.
	17 August 1998		ORDER_MAX increased to 35.
						FACE_PRINT routine added.
						INIT_DATA routine added.
	14 August 1998		IV_READ is working.
	13 August 1998		ASE_WRITE is working.
						IV_WRITE is working.
	12 August 1998		ASE_READ is working.
	10 August 1998		DXF_WRITE is working.
						DXF_READ is working.
	27 July 1998		Interactive mode is working.
						OBJ_READ is working.
	25 July 1998		OBJ_WRITE is working.
	24 July 1998		DATA_CHECK checks the input data.
						VLA_READ is working.
						VLA_WRITE is working.
	23 July 1998		STL_WRITE is working.
	22 July 1998		STL_READ is working.
						TXT_WRITE is working.
*/
/**********************************************************************/

IVCONV* iv;	// the converter class

int command_line ( char **argv )

{
	int   iarg;
	bool  ok;
	bool reverse_faces;
	bool reverse_normals;
	char filein_name[500];	// longer path to allow network...
	char fileout_name[500];	// longer path to allow network...

	/* 
	Initialize local data. 
	*/
	iarg = 0;
	ok = true;
	reverse_faces = false;
	reverse_normals = false;

	// Get the -RN option, -RF option, and the input file name. 
	iarg = iarg + 1;
	strcpy ( filein_name, argv[iarg] );
	
	if ( leqi ( filein_name, "-RN" )  ) {
		reverse_normals = true;
		printf ( "\n" );
		printf ( "COMMAND_LINE: Reverse_Normals option requested.\n" );
		iarg = iarg + 1;
		strcpy ( filein_name, argv[iarg] );
	}
	
	if ( leqi ( filein_name, "-RF" )  ) {
		reverse_faces = true;
		printf ( "\n" );
		printf ( "COMMAND_LINE: Reverse_Faces option requested.\n" );
		iarg = iarg + 1;
		strcpy ( filein_name, argv[iarg] );
	}
	/*
	Read the input. 
	*/
	ok = iv->read (filein_name);
	
	if (!ok) {
		printf ( "\n" );
		printf ( "COMMAND_LINE - Fatal error!\n" );
		printf ( "  Failure while reading input data.\n" );
		return false;
	}
	/*
	Reverse the normal vectors if requested.
	*/
	if ( reverse_normals  ) 
	{	
		iv->reverse_normals();
		printf ( "\n" );
		printf ( "COMMAND_LINE - Note:\n" );
		printf ( "  Reversed node, face, and vertex normals.\n" );

	}
	/*
	Reverse the faces if requested.
	*/
	if ( reverse_faces  ) {
		
		iv->face_reverse_order ( );
		
		printf ( "\n" );
		printf ( "COMMAND_LINE - Note:\n" );
		printf ( "  Reversed the face definitions.\n" );
	}
	/*
	Write the output file. 
	*/
	iarg = iarg + 1;
	strcpy ( fileout_name, argv[iarg] );
	
	ok = iv->write (fileout_name);
	
	if (!ok) {
		printf ( "\n" );
		printf ( "COMMAND_LINE - Fatal error!\n" );
		printf ( "  Failure while writing output data.\n" );
		return false;
	}
	return true;
}

void hello ()

/******************************************************************************/
/*
Purpose:

  HELLO prints an explanatory header message.
  
	Modified:
	
	  04 July 2000
	  
		Author:
		
		  John Burkardt
		  */
{
	printf ( "\n" );
	printf ( "IVCON DynaBits version 1.0 2003/03/16\n" );
	printf ( "\n" );
	printf ( ".3ds/ase       3D Studio Max (binary & ASCII);\n" );
	printf ( ".asc           Points Cloud ASCII;\n" );
	printf ( ".byu           Movie.BYU surface geometry;\n" );
	printf ( ".dxf           Autocad DXF;\n" );
	printf ( ".gmod          Golgotha model;\n" );
	printf ( ".hrc           SoftImage hierarchy;\n" );
	printf ( ".iv            SGI Open Inventor;\n" );
	printf ( ".obj           WaveFront Advanced Visualizer;\n" );
	printf ( ".pov           Persistence of Vision (output only);\n" );
	printf ( ".smf           Michael Garland's format;\n" );
	printf ( ".stl/stla/stlb StereoLithography (ASCII & binary);\n" );
	printf ( ".tec           TECPLOT (output only);\n" );
	printf ( ".tri/tria/trib Greg Hood's triangle format(ASCII & binary);\n" );
	printf ( ".txt           Text (output only);\n" );
	printf ( ".ucd           AVS UCD file(output only);\n" );
	printf ( ".vla           VLA;\n" );
	printf ( ".wrl           VRML Virtual Reality Modeling Language.\n" );
	printf ( ".xgl           XML/OpenGL format (output only);\n" );
	printf ( "\n" );
	printf ( "\n" );
	printf ( "" );
	printf ( "This software is FREE and OpenSource : visit www.dynabits.com\n" );
	
	return;
}
/******************************************************************************/

void help ()

/******************************************************************************/

/*
Purpose:

  HELP prints a list of the interactive commands.
  
	Modified:
	
	  26 May 1999
	  
		Author:
		
		  John Burkardt
		  */
{
	printf ( "\n" );
	printf ( "Commands:\n" );
	printf ( "\n" );
	printf ( "< file   Read data from input file;\n" );
	printf ( "<< file  Append data in input file to current data;\n" );
	printf ( "> file   Write output file;\n" );
	printf ( "B        Switch the binary file byte-swapping mode;\n" );
	printf ( "D        Switch the debugging mode;\n" );
	printf ( "F        Print information about one face;\n" );
	printf ( "H        Print this help list;\n" );
	printf ( "I        Info, print out recent changes;\n" );
	printf ( "LINES    Convert face information to lines;\n" );
	printf ( "N        Recompute normal vectors;\n" );
	printf ( "P        Set LINE_PRUNE option.\n" );
	printf ( "Q        Quit;\n" );
	printf ( "R        Reverse the normal vectors.\n" );
	printf ( "S        Select face subset (NOT WORKING).\n" );
	printf ( "T        Transform the data.\n" );
	printf ( "W        Reverse the face node ordering.\n" );
	
	return;
}
/******************************************************************************/


int interact ()

/******************************************************************************/

/*
Purpose:

INTERACT carries on an interactive session with the user.

Modified:

22 May 1999

Author:

John Burkardt
*/
{
	int    ok;
	int    iface;
	char  *next;
	float  x;
	float  y;
	float  z;
	bool line_prune=true;
	
	char filein_name[500];
	char fileout_name[500];

	strcpy ( filein_name, "NO_IN_NAME" );
	strcpy ( fileout_name, "NO_OUT_NAME" );
	
	/*  
	Say hello. 
	*/
	hello ();
	/*  
	Get the next user command. 
	*/
	printf ( "\n" );
	printf ( "Enter command (H for help)\n" );
	
	char   input[LINE_MAX_LEN];

	while ( fgets ( input, LINE_MAX_LEN, stdin ) != NULL ) {
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
		/*  
		Command: << FILENAME 
		Append new data to current graphics information.
		*/
		if ( *next == '<' && *(next+1) == '<' ) {
			
			next = next + 2;
			sscanf ( next, "%s", filein_name );
			
			ok = iv->read(filein_name);
			
			if (!ok) {
				printf ( "\n" );
				printf ( "INTERACT - Fatal error!\n" );
				printf ( "  DATA_READ failed to read input data.\n" );
			}
		}
		/*  
		Command: < FILENAME 
		*/
		else if ( *next == '<' ) {
			
			next = next + 1;
			sscanf ( next, "%s", filein_name );
						
			ok = iv->read (filein_name);
			
			if (!ok) {
				printf ( "\n" );
				printf ( "INTERACT - Fatal error!\n" );
				printf ( "  DATA_READ failed to read input data.\n" );
			}
		}
		/*  
		Command: > FILENAME 
		*/
		else if ( *next == '>' ) {
			
			next = next + 1;
			sscanf ( next, "%s", fileout_name ); 
			
			ok = iv->write(fileout_name);
			
			if (!ok) {
				printf ( "\n" );
				printf ( "INTERACT - Fatal error!\n" );
				printf ( "  OUTPUT_DATA failed to write output data.\n" );
			}
			
		}
		/*
		B: Switch byte swapping option.
		*/
		else if ( *next == 'B' || *next == 'b' ) {
			
			if ( byte_swap  ) {
				byte_swap = false;
				printf ( "Byte_swapping reset to FALSE.\n" );
			}
			else {
				byte_swap = true;
				printf ( "Byte_swapping reset to TRUE.\n" );
			}
			
		}
		/*
		D: Switch debug option.
		*/
		else if ( *next == 'D' || *next == 'd' ) {
			if ( debug ) {
				debug = 0;
				printf ( "Debug reset to FALSE.\n" );
			}
			else {
				debug = 1;
				printf ( "Debug reset to TRUE.\n" );
			}
		}
		/*  
		F: Check a face. 
		*/
		else if ( *next == 'f' || *next == 'F' ) {
			printf ( "\n" );
			printf ( "  Enter a face index between 0 and %d:", iv->get_faces_count()-1 );
			scanf ( "%d", &iface );
			iv->face_print ( iface );
		}
		/*  
		H: Help
		*/
		else if ( *next == 'h' || *next == 'H' ) {
			help ( );
		}
		/*
		I: Print change information. 
		*/
		else if ( *next == 'i' || *next == 'I') {
			//	news ( );	// removed by PhG, see changes.txt file
		}
		/*
		LINES: 
		Convert face information to lines.
		*/
		else if ( *next == 'l' || *next == 'L') {
			
			if ( iv->get_faces_count() > 0 ) {
				
				printf ( "\n" );
				printf ( "INTERACT - Note:\n" );
				printf ( "  Face information will be converted\n" );
				printf ( "  to line information.\n" );
				
				iv->face_to_line (line_prune);
			}
			else {
				
				printf ( "\n" );
				printf ( "INTERACT - Note:\n" );
				printf ( "  There were no faces to convert.\n" );
				
			}
			
		}
		/*
		N: Recompute normal vectors.
		*/
		else if ( *next == 'n' || *next == 'N') {
			iv->recompute_normals();
		}
		/*
		P: Line pruning optiont
		*/
		else if ( *next == 'p' || *next == 'P' ) {
			
			printf ( "\n" );
			printf ( "INTERACT - SET LINE PRUNING OPTION.\n" );
			printf ( "\n" );
			printf ( "  LINE_PRUNE = 0 means no line pruning.\n" );
			printf ( "               nonzero means line pruning.\n" );
			printf ( "\n" );
			printf ( "  Current value is LINE_PRUNE = %d.\n", line_prune );
			printf ( "\n" );
			printf ( "  Enter new value for LINE_PRUNE.\n" );
			
			if ( fgets ( input, LINE_MAX_LEN, stdin ) == NULL ) {
				printf ( "  ??? Error trying to read input.\n" );
			}
			else {
				char lpr;
				sscanf ( input, "%c", &lpr );
				if (lpr==0) line_prune=false;
				else if (lpr==1) line_prune=true;
				else printf("ERROR: LINE_PRUNE is not 0 or 1");

				printf ( "  New value is LINE_PRUNE = %d.\n", line_prune );
			}
		}
		/*
		Q: Quit
		*/
		else if ( *next == 'q' || *next == 'Q' ) {
			printf ( "\n" );
			printf ( "INTERACT - Normal end of execution.\n" );
			return true;
		}
		/*
		R: Reverse normal vectors. 
		*/
		else if ( *next == 'r' || *next == 'R' ) {
			iv->reverse_normals();
			printf ( "\n" );
			printf ( "INTERACT - Note:\n" );
			printf ( "  Reversed node, face and vertex normals.\n" );
		}
		/*  
		S: Select a few faces, discard the rest.
		*/
		else if ( *next == 's' || *next == 'S' ) {
			iv->face_subset ( );
		}
		/*  
		T: Transform the data.
		*/
		else if ( *next == 't' || *next == 'T' ) {
			
			printf ( "\n" );
			printf ( "For now, we only offer point scaling.\n" );
			printf ( "Enter X, Y, Z scale factors:\n" );
			
			scanf ( "%f %f %f", &x, &y, &z );
			iv->scale(x,y,z);
		}
		/*
		U: Renumber faces, count objects:
		*/
		else if ( *next == 'u' || *next == 'U' ) {
		}
		/*
		V: Convert polygons to triangles:
		*/
		else if ( *next == 'v' || *next == 'V' ) {
		}
		/*
		W: Reverse the face node ordering. 
		*/
		else if ( *next == 'w' || *next == 'W' ) {
			
			iv->reverse_faces();
			printf ( "\n" );
			printf ( "INTERACT - Note:\n" );
			printf ( "  Reversed face node ordering.\n" );
		}
		/*
		Command: ???  
		*/
		else {
			printf ( "\n" );
			printf ( "INTERACT: Warning!\n" );
			printf ( "  Your command was not recognized.\n" );
		}
		
		printf ( "\n" );
		printf ( "Enter command (H for help)\n" );
		
  }
  return true;
}
/******************************************************************************/


int main ( int argc, char **argv )
{
	ros::init(argc, argv, "ivcon");
	
	printf("initializing...\n");
	iv = new IVCONV();
	int result;
	// init_program_data ( );	// performed by IV constructor
	//	If there are at least two command line arguments, call COMMAND_LINE.
	//	Otherwise call INTERACT and get information from the user.
	if ( argc >= 2 ) {
		result = command_line ( argv );
	}
	else {
		result = interact (  );
	delete iv;
	}
	
	return result;
}
