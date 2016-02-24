#ifndef _ivcon_ivconv
#define _ivcon_ivconv


#ifdef __linux__
#include <string.h>
#endif


#include "tmat.h"	// transform matrix
#include "utils.h"	// various utilities

#include <string>
#include <assert.h>

using std::string;

const int LINE_MAX_LEN=256;
const int COLOR_MAX=1000;
// const long FACE_MAX=200000;
const int LEVEL_MAX=10;
const long LINES_MAX=100000;
const int ORDER_MAX=20;
const int TEXTURE_MAX=100;
// const long COR3_MAX=200000;

struct Material
{
	char   name[LINE_MAX_LEN];
	vec3 ambient;
	vec3 rgb;	// diffuse : the "real" material color
	vec3 specular;
	float alpha;
	Material(float r, float g, float b, float a=1):rgb(r,g,b),ambient(rgb),specular(rgb),alpha(a) {};
	Material(){rgb=vec3(.5,.5,.5); ambient=rgb; specular=rgb; alpha=1.0;};
};

class IVCONV	// c++ wrapper around ivcon.c
{
public:	// needed for library use
	IVCONV();
	bool read(const string& file);
	bool write(const string& file);

	//set this value to scale all meshes that are converted with this interface (default 1.0).
	//The static version is for setting the default, and the non-static can be changed per class.
	static float SCALE_FACTOR;
	float scale_factor;

public:	// needed for interactive use (main.cpp)
	bool face_subset ();

	bool reverse_normals();
	bool reverse_faces();

	bool face_reverse_order ();
	bool scale(float x, float y, float z);

	int get_faces_count() const {return face_num;}
	bool face_print (int iface );
	bool face_to_line (bool line_prune);

	bool recompute_normals();

protected: // "high level"
	void data_init ();
	void init_program_data ();
	bool data_write ();
	bool data_read ();

	void materials_fixups();


protected:	// converters 

	bool asc_read ( FILE *filein );
	bool asc_write ( FILE *fileout );
	bool ase_read ( FILE *filein );
	bool ase_write ( FILE *fileout );
	bool byu_read ( FILE *filein );
	bool byu_write ( FILE *fileout );
	bool dxf_read ( FILE *filein );
	bool dxf_write ( FILE *fileout );
	bool hrc_read ( FILE *filein );
	bool hrc_write ( FILE *fileout );
	bool iv_read ( FILE *filein );
	bool iv_write ( FILE *fileout );
	bool obj_read ( FILE *filein );
	bool obj_write ( FILE *fileout );
	bool pov_write ( FILE *fileout );
	bool smf_read ( FILE *filein );
	bool smf_write ( FILE *fileout );
	bool stla_read ( FILE *filein );
	bool stla_write ( FILE *fileout );
	bool stlb_read ( FILE *filein );
	bool stlb_write ( FILE *fileout );
	bool gmod_read ( FILE *fileout );
	bool gmod_write ( FILE *fileout );
	bool tds_read ( FILE *filein );
	bool tds_write ( FILE *fileout );
	bool tec_write ( FILE *fileout );
	bool tria_read ( FILE *filein );
	bool tria_write ( FILE *fileout );
	bool trib_read ( FILE *filein );
	bool trib_write ( FILE *fileout );
	bool txt_write ( FILE *fileout );
	bool ucd_write ( FILE *fileout );
	bool vla_read ( FILE *filein );
	bool vla_write ( FILE *fileout );
	bool wrl_write ( FILE *filout );
	bool xgl_write ( FILE *fileout );


protected:

	char filein_name[500];	// longer path to allow network...
	char fileout_name[500];	// longer path to allow network...

	/******************************************************************************/

	/* FUNCTION PROTOTYPES */

	/******************************************************************************/


	// other
	void               cor3_normal_set ();
	void               cor3_range ();
	void               data_check ();
	void               data_report ();
	void               edge_null_delete ();
	void               face_area_set ();
	void               face_normal_ave ();
	void               face_null_delete ();
	void               face_to_vertex_material ();
	void               node_to_vertex_material ();
	unsigned long  tds_read_boolean ( unsigned char *boolean, FILE *filein );
	unsigned long  tds_read_edit_section ( FILE *filein, int *views_read );
	unsigned long  tds_read_keyframe_section ( FILE *filein, int *views_read );
	unsigned long  tds_read_keyframe_objdes_section ( FILE *filein );
	unsigned long  tds_read_material_section ( FILE *filein );
	unsigned long  tds_read_obj_section ( FILE *filein );
	unsigned long  tds_read_object_section ( FILE *filein );
	unsigned long  tds_read_tex_verts_section ( FILE *filein );
	unsigned long  tds_read_texmap_section ( FILE *filein );
	unsigned long  tds_read_spot_section ( FILE *filein );
	unsigned long  tds_read_vp_section ( FILE *filein, int *views_read );
	void               vertex_normal_set ();
	void               vertex_to_face_material ();
	void               vertex_to_node_material ();

protected:
/*
ORDER_MAX, the maximum number of vertices per face.
TEXTURE_MAX, the maximum number of textures.
TEXTURE_NAME[TEXTURE_MAX][LINE_MAX_LEN], ...
TEXTURE_NUM, the number of textures.
TRANSFORM_MATRIX[4][4], the current transformation matrix.
*/
	vec3  background_rgb;		// the background color.
	int    bad_num;
	int    bytes_num;
	int    color_num;
	int    comment_num;

	long    cor3_num;
	array<vec3>	cor3;			// the coordinates of nodes.
	array<int>	cor3_material;	// the index of the material of each node.
	array<vec3>	cor3_normal;	// normal vectors associated with nodes.
	array<vec3>	cor3_tex_uv;	// texture coordinates associated with nodes.

	int    dup_num;

	long    face_num;
	array< array<long> > face;	// contains the index of the I-th node making up face J
	array<float>  face_area;	// the area of each face.
	array<int>	face_flags;
	array<int>	face_material;	// the material of each face.
	array<vec3>	face_normal;	// the face normal vectors.
	array<int>	face_object;
	array<int>	face_order;		// the number of vertices per face.
	array<int>	face_smooth;
	array<vec2> face_tex_uv;	// texture coordinates associated with faces.


	int    group_num;

	char   level_name[LEVEL_MAX][LINE_MAX_LEN];

	long    line_num;
	array<int>	line_dex;		// node indices, denoting polylines, each terminated by -1.
	array<int>	line_material;	// index into RGBCOLOR for line color.

	array<int>	list;

	char   material_binding[80];
	int    material_num;
	array<Material> material;

	int    max_order2;

	char   normal_binding[80];
	array<vec3>  normal_temp;

	int    object_num;

	vec3  origin;
	vec3  pivot;
	array<vec3>  rgbcolor;

	int    text_num;

	char   texture_binding[80];
	char   texture_name[TEXTURE_MAX][LINE_MAX_LEN];
	int    texture_num;
	array<vec2>	texture_temp;

	float  transform_matrix[4][4];

	array< array<int> >		vertex_material;	// the material of vertices of faces.
	array< array<vec3> >	vertex_normal;		// normals at vertices of faces. 
	array< array<vec3> >	vertex_rgb;			// colors of vertices of faces. 
	array< array<vec2> >	vertex_tex_uv;		// texture coordinates of vertices of faces.
};

// some low-level file utility routines

extern bool debug;

#endif
