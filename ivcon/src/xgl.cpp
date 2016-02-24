#include "ivconv.h"

bool IVCONV::xgl_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

XGL_WRITE writes an XGL file.

Discussion:

Two corrections to the routine were pointed out by
Mike Phillips, msphil@widowmaker.com, on 17 September 2001,
and are gratefully acknowledged.

Example:

<WORLD>

<BACKGROUND>
<BACKCOLOR> 0.1, 0.1, 0.1 </BACKCOLOR>
</BACKGROUND>

<LIGHTING>
<AMBIENT> 0.2, 0.1, 0.1 </AMBIENT>
<DIRECTIONALLIGHT>
<DIFFUSE> 0.1, 0.2, 0.1 </DIFFUSE>
<DIRECTION> 0, 0, 100 </DIRECTION>
<SPECULAR> 0.1, 0.1, 0.2 </SPECULAR>
</DIRECTIONALLIGHT>
</LIGHTING>

<MESH ID = "0">

<P ID="0"> -0.5, -0.5, 1 </P>
<P ID="1"> 0.5, -0.5, 1 </P>
<P ID="2"> 0.5, 0.5, 1 </P>
<P ID="3"> -0.5, 0.5, 1 </P>
<P ID="4"> 0.5, -0.5, 0 </P>
<P ID="5"> -0.5, -0.5, 0 </P>
<P ID="6"> -0.5, 0.5, 0 </P>
<P ID="7"> 0.5, 0.5, 0 </P>

<N ID="0"> -0.408248, -0.408248, 0.816497 </N>
<N ID="1"> 0.666667, -0.666667, 0.333333 </N>
<N ID="2"> 0.408248, 0.408248, 0.816497 </N>
<N ID="3"> -0.666667, 0.666667, 0.333333 </N>
<N ID="4"> 0.408248, -0.408248, -0.816497 </N>
<N ID="5"> -0.666667, -0.666667, -0.333333 </N>
<N ID="6"> -0.408248, 0.408248, -0.816497 </N>
<N ID="7"> 0.666667, 0.666667, -0.333333 </N>

<MAT ID="0">
<ALPHA> 0.9 </ALPHA>
<AMB> 0.1, 0.1, 0.1 </AMB>
<DIFF> 0.2, 0.1, 0.1 </DIFF>
<EMISS> 0.1, 0.2, 0.1 </EMISS>
<SHINE> 0.8 </SHINE>
<SPEC> 0.1, 0.1, 0.2 </SPEC>
</MAT>

<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 0 </PREF><NREF> 0 </NREF></FV1>
<FV2><PREF> 1 </PREF><NREF> 1 </NREF></FV2>
<FV3><PREF> 2 </PREF><NREF> 2 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 0 </PREF><NREF> 0 </NREF></FV1>
<FV2><PREF> 2 </PREF><NREF> 2 </NREF></FV2>
<FV3><PREF> 3 </PREF><NREF> 3 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 4 </PREF><NREF> 4 </NREF></FV1>
<FV2><PREF> 5 </PREF><NREF> 5 </NREF></FV2>
<FV3><PREF> 6 </PREF><NREF> 6 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 4 </PREF><NREF> 4 </NREF></FV1>
<FV2><PREF> 6 </PREF><NREF> 6 </NREF></FV2>
<FV3><PREF> 7 </PREF><NREF> 7 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 5 </PREF><NREF> 5 </NREF></FV1>
<FV2><PREF> 0 </PREF><NREF> 0 </NREF></FV2>
<FV3><PREF> 3 </PREF><NREF> 3 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 5 </PREF><NREF> 5 </NREF></FV1>
<FV2><PREF> 3 </PREF><NREF> 3 </NREF></FV2>
<FV3><PREF> 6 </PREF><NREF> 6 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 1 </PREF><NREF> 1 </NREF></FV1>
<FV2><PREF> 4 </PREF><NREF> 4 </NREF></FV2>
<FV3><PREF> 7 </PREF><NREF> 7 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 1 </PREF><NREF> 1 </NREF></FV1>
<FV2><PREF> 7 </PREF><NREF> 7 </NREF></FV2>
<FV3><PREF> 2 </PREF><NREF> 2 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 5 </PREF><NREF> 5 </NREF></FV1>
<FV2><PREF> 4 </PREF><NREF> 4 </NREF></FV2>
<FV3><PREF> 1 </PREF><NREF> 1 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 5 </PREF><NREF> 5 </NREF></FV1>
<FV2><PREF> 1 </PREF><NREF> 1 </NREF></FV2>
<FV3><PREF> 0 </PREF><NREF> 0 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 3 </PREF><NREF> 3 </NREF></FV1>
<FV2><PREF> 2 </PREF><NREF> 2 </NREF></FV2>
<FV3><PREF> 7 </PREF><NREF> 7 </NREF></FV3>
</F>
<F>
<MATREF> 0 </MATREF>
<FV1><PREF> 3 </PREF><NREF> 3 </NREF></FV1>
<FV2><PREF> 7 </PREF><NREF> 7 </NREF></FV2>
<FV3><PREF> 6 </PREF><NREF> 6 </NREF></FV3>
</F>
</MESH>

<OBJECT>
<TRANSFORM>
<FORWARD> 0, 0, 0 </FORWARD>
<POSITION> 0, 0, 0 </POSITION>
<SCALE> 1, 1, 1 </SCALE>
<UP> 1, 1, 1 </UP>
</TRANSFORM>
<MESHREF> 0 </MESHREF>
</OBJECT>

</WORLD>

Reference:

XGL specification at http://www.xglspec.org/

Modified:

17 September 2001

Author:

John Burkardt
*/
{
	int iface;
	int ivert;
	int j;
	float light_ambient_rgb[3];
	float light_diffuse_rgb[3];
	float light_direction[3];
	float light_specular_rgb[3];
	int material;
	float material_alpha;
	float material_amb_rgb[3];
	float material_diff_rgb[3];
	float material_emiss_rgb[3];
	float material_shine;
	float material_spec_rgb[3];
	int mesh;
	int mesh_num = 1;
	int object;
	float transform_forward[3];
	float transform_position[3];
	float transform_scale[3];
	float transform_up[3];
	/*
	Make up some placeholder values for now.
	*/
	light_ambient_rgb[0] = 0.2;
	light_ambient_rgb[1] = 0.1;
	light_ambient_rgb[2] = 0.1;
	
	light_diffuse_rgb[0] = 0.1;
	light_diffuse_rgb[1] = 0.2;
	light_diffuse_rgb[2] = 0.1;
	
	light_direction[0] =   0.0;
	light_direction[1] =   0.0;
	light_direction[2] = 100.0;
	
	light_specular_rgb[0] = 0.1;
	light_specular_rgb[1] = 0.1;
	light_specular_rgb[2] = 0.2;
	
	material_alpha = 0.9;
	
	material_amb_rgb[0] = 0.1;
	material_amb_rgb[1] = 0.1;
	material_amb_rgb[2] = 0.1;
	
	material_diff_rgb[0] = 0.2;
	material_diff_rgb[1] = 0.1;
	material_diff_rgb[2] = 0.1;
	
	material_emiss_rgb[0] = 0.1;
	material_emiss_rgb[1] = 0.2;
	material_emiss_rgb[2] = 0.1;
	
	material_shine = 0.8;
	
	material_spec_rgb[0] = 0.1;
	material_spec_rgb[1] = 0.1;
	material_spec_rgb[2] = 0.2;
	
	transform_forward[0] = 0.0;
	transform_forward[1] = 0.0;
	transform_forward[2] = 0.0;
	
	transform_position[0] = 0.0;
	transform_position[1] = 0.0;
	transform_position[2] = 0.0;
	
	transform_scale[0] = 1.0;
	transform_scale[1] = 1.0;
	transform_scale[2] = 1.0;
	
	transform_up[0] = 1.0;
	transform_up[1] = 1.0;
	transform_up[2] = 1.0;
	
	object_num = 1;
	
	text_num = 0;
	
	fprintf ( fileout, "<WORLD>\n" );
	fprintf ( fileout, "\n" );
	
	text_num = text_num + 2;
	
	fprintf ( fileout, "  <BACKGROUND>\n" );
	fprintf ( fileout, "    <BACKCOLOR> %f, %f, %f </BACKCOLOR>\n", 
		background_rgb[0], background_rgb[1], background_rgb[2] );
	fprintf ( fileout, "  </BACKGROUND>\n" );
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "  <LIGHTING>\n" );
	fprintf ( fileout, "    <AMBIENT> %f, %f, %f </AMBIENT>\n",
		light_ambient_rgb[0], light_ambient_rgb[1], light_ambient_rgb[2] );
	fprintf ( fileout, "    <DIRECTIONALLIGHT>\n" );
	fprintf ( fileout, "      <DIFFUSE> %f, %f, %f </DIFFUSE>\n",
		light_diffuse_rgb[0], light_diffuse_rgb[1], light_diffuse_rgb[2] );
	fprintf ( fileout, "      <DIRECTION> %f, %f, %f </DIRECTION>\n",
		light_direction[0], light_direction[1], light_direction[2] );
	fprintf ( fileout, "      <SPECULAR> %f, %f, %f </SPECULAR>\n",
		light_specular_rgb[0], light_specular_rgb[1], light_specular_rgb[2] );
	fprintf ( fileout, "    </DIRECTIONALLIGHT>\n" );
	fprintf ( fileout, "  </LIGHTING>\n" );
	
	text_num+=12;
	
	for ( mesh = 0; mesh < mesh_num; mesh++ ) {
		
		fprintf ( fileout, "\n" );
		fprintf ( fileout, "  <MESH ID = \"%d\">\n", mesh );
		fprintf ( fileout, "\n" );
		text_num = text_num + 3;
		
		for ( j = 0; j < cor3_num; ++j ) {
			fprintf ( fileout, "    <P ID=\"%d\"> %f, %f, %f </P>\n", j,
				cor3[j][0], cor3[j][1], cor3[j][2] );
			++text_num;
		}
		
		fprintf ( fileout, "\n" );
		++text_num;
		for ( j = 0; j < cor3_num; ++j ) {
			fprintf ( fileout, "    <N ID=\"%d\"> %f, %f, %f </N>\n", j,
				cor3_normal[j][0], cor3_normal[j][1], cor3_normal[j][2] );
			++text_num;
		}
		
		for ( material = 0; material < material_num; material++ ) {
			fprintf ( fileout, "\n" );
			fprintf ( fileout, "    <MAT ID=\"%d\">\n", material );
			fprintf ( fileout, "      <ALPHA> %f </ALPHA>\n", material_alpha );
			fprintf ( fileout, "      <AMB> %f, %f, %f </AMB>\n",
				material_amb_rgb[0], material_amb_rgb[1], material_amb_rgb[2] );
			fprintf ( fileout, "      <DIFF> %f, %f, %f </DIFF>\n",
				material_diff_rgb[0], material_diff_rgb[1], material_diff_rgb[2] );
			fprintf ( fileout, "      <EMISS> %f, %f, %f </EMISS>\n",
				material_emiss_rgb[0], material_emiss_rgb[1], material_emiss_rgb[2] );
			fprintf ( fileout, "      <SHINE> %f </SHINE>\n", material_shine );
			fprintf ( fileout, "      <SPEC> %f, %f, %f </SPEC>\n",
				material_spec_rgb[0], material_spec_rgb[1], material_spec_rgb[2] );
			fprintf ( fileout, "    </MAT>\n" );
			text_num = text_num + 9;
		}
		
		fprintf ( fileout, "\n" );
		++text_num;
		
		for ( iface = 0; iface < face_num; iface++ ) {
			fprintf ( fileout, "    <F>\n" );
			fprintf ( fileout, "      <MATREF> %d </MATREF>\n", face_material[iface] );
			text_num = text_num + 2;
			for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
				fprintf ( fileout, 
					"      <FV%d><PREF> %d </PREF><NREF> %d </NREF></FV%d>\n",
					ivert+1, face[ivert][iface], face[ivert][iface], ivert+1 );
				++text_num;
			}
			fprintf ( fileout, "    </F>\n" );
			++text_num;
		}
		
		fprintf ( fileout, "  </MESH>\n" );
		++text_num;
		
	}
	
	fprintf ( fileout, "\n" );
	++text_num;
	
	for ( object = 0; object < object_num; object++ ) {
		
		fprintf ( fileout, "  <OBJECT>\n" );
		fprintf ( fileout, "    <TRANSFORM>\n" );
		fprintf ( fileout, "      <FORWARD> %f, %f, %f </FORWARD>\n",
			transform_forward[0], transform_forward[1], transform_forward[2] );
		fprintf ( fileout, "      <POSITION> %f, %f, %f </POSITION>\n",
			transform_position[0], transform_position[1], transform_position[2] );
		fprintf ( fileout, "'      <SCALE> %f, %f, %f </SCALE>\n",
			transform_scale[0], transform_scale[1], transform_scale[2] );
		fprintf ( fileout, "      <UP> %f, %f, %f </UP>\n",
			transform_up[0], transform_up[1], transform_up[2] );
		fprintf ( fileout, "    </TRANSFORM>\n" );
		mesh = 0;
		fprintf ( fileout, "    <MESHREF> %d </MESHREF>\n", mesh );
		fprintf ( fileout, "  </OBJECT>\n" );
		text_num = text_num + 9;
		
	}
	
	fprintf ( fileout, "\n" );
	fprintf ( fileout, "</WORLD>\n" );
	text_num = text_num + 2;
	
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "XGL_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}

