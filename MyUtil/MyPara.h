#ifndef _MY_PARA_
#define _MY_PARA_

enum RENDER_MODE {RENDER_TRANSPARENT, RENDER_MATERIAL, RENDER_NPR};


typedef struct tag_RenderPara
{
	enum DRAWING_RENDER_MODE {DEFAULT_MODE, UNIT_MODE, CONN_MODE, LABEL_MODE, OBJECT_MODE, SUBPAGE_MODE};

	bool _draw_layer_image;
	bool _draw_proxy;
	bool _draw_connect_graph;
	bool _draw_hexagon;
	bool _draw_shortest_edge;
	bool _draw_wireframe;
	bool _draw_mesh;

	DRAWING_RENDER_MODE _drawing_render_mode;
	RENDER_MODE _render_mode;
	bool _draw_npr_with_light;
	float _glPlyOffset_factor;
	float _glPlyOffset_units;
	float _line_width;
	float _line_color[4];
	float _silhouette_width;
	float _sil_color[4];

	float _svg_manual_scale;
	float _svg_manual_offset[2];
	int _shader_mode;
	double m_3DView_gl_proj[16];
	void init();
}RENDER_PARA;

typedef struct tag_FeaturePara
{
	//neighbor graph
	float _nei_thresh;
	int _bin_num;
	int _patch_size;
	int _filtered_img_w;
	int _filtered_img_h;
	float _shortest_len2;
	float _parallel_thresh;
	float _pair_edge_w;
	void init();
}FeaturePara;



enum CAM_CALIBERATE_MODE {K_RT, RT};	//KRT: full needs to be calliberate; RT: K is known
typedef struct tag_PROXY_FIT_PARA
{
	float _sampling_angle;
	//for cluster segements, dot of two directions
	float _cluster_thresh;

	CAM_CALIBERATE_MODE _cam_calib_mode;

	float _fit_error_thresh;
	float _relation_weight;
	float _para_thresh;
	float _lenEqual_thresh;

	float _small_len_thresh;

	float _max_perturb_angle;

	float _default_plane_d;

	float _euler_angle_lbnd;
	float _euler_angle_ubnd;

	void init();
}PROXY_FIT_PARA;

typedef struct tag_RELATION_PARA
{
	float _para_thresh;
	float _equal_thresh;
	void init();
}RELATION_PARA;

typedef struct tag_UNITS_PARA
{
	float _para_small_path_thresh;
	float _para_same_point_thresh;
	float _para_small_path_generating_polygon;
	float _para_gen_edge_thresh;
	float _para_find_line_empty_end_thresh;
	float _para_shape_touch_thresh;
	float _para_match_accept_thresh;
	float _para_rigid_transform_point_equal_thresh;
	float _para_rigid_candicate_radius_thresh;
	float _para_sample_straight_line;
	float _para_fitting_score_thresh;
	int _para_fitting_image_size_thresh;
	int _para_fitting_image_large_size_thresh;
	float _para_fitting_score_thresh_single;
	void init();
}UNITS_PARA;

typedef struct tag_ANIM_PARA
{
	float _ratio_interval;
	void init();
}ANIM_PARA;

#endif