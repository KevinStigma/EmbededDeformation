#include "stdafx.h"
#include "MyPara.h"
#include "MyMacro.h"

void RENDER_PARA::init()
{
	_draw_layer_image = false;
	_draw_proxy = true;
	_draw_connect_graph = false;
	_draw_hexagon = false;
	_draw_wireframe = false;
	_drawing_render_mode = OBJECT_MODE;
	_svg_manual_scale = 1.f;
	_svg_manual_offset[0] = _svg_manual_offset[1] = 0.f;
	_render_mode = RENDER_NPR;
	_draw_npr_with_light = false;
	_glPlyOffset_factor = 2.0f;
	_glPlyOffset_units = 2.0f;
	_line_width = 2.0f;
	_line_color[0] = _line_color[1] = _line_color[2] =  0.3f;
	_line_color[3] = 1.f;
	_silhouette_width = 3.f;
	_sil_color[0] = _sil_color[1] = _sil_color[2] = 0.f; _sil_color[3] = 1.f;
	_draw_shortest_edge = true;
	_draw_mesh = true;
	_shader_mode=0;
	m_3DView_gl_proj[0]=16.78396;m_3DView_gl_proj[1]=0;m_3DView_gl_proj[2]=0;m_3DView_gl_proj[3]=0;
	m_3DView_gl_proj[0]=0;m_3DView_gl_proj[1]=17.9778;m_3DView_gl_proj[2]=0;m_3DView_gl_proj[3]=0;
	m_3DView_gl_proj[0]=0;m_3DView_gl_proj[1]=-0.0648535;m_3DView_gl_proj[2]=-1.00066;m_3DView_gl_proj[3]=-1;
	m_3DView_gl_proj[0]=0;m_3DView_gl_proj[1]=0;m_3DView_gl_proj[2]=-0.20006;m_3DView_gl_proj[3]=0;
}

void FeaturePara::init()
{
	_nei_thresh = 0.8f;
	_bin_num = 8;
	_patch_size = 20;
	_filtered_img_w = 1024;
	_filtered_img_h = 1024;
	_shortest_len2 = 30.f;
	_parallel_thresh = 0.97f;
	_pair_edge_w = 1.f;
}


void PROXY_FIT_PARA::init()
{
	_sampling_angle = M_PI/36.f;	//the angle will be (-_sampling_angle/2, _sampling_angle/2)

	_cluster_thresh = 0.95f;

	_cam_calib_mode = K_RT;

	_fit_error_thresh = 1000000000.f;
	_relation_weight = 1000.f;

	_para_thresh = 0.99f;
	_lenEqual_thresh = 2.f;

	_small_len_thresh = 20.f;

	_max_perturb_angle = M_PI/36.f;

	_default_plane_d = -1.f;

	_euler_angle_lbnd = -M_PI/6.f;
	_euler_angle_ubnd = M_PI/6.f;
}

void RELATION_PARA::init()
{
	_para_thresh = 0.99f;
	_equal_thresh = 0.9f;
}

void UNITS_PARA::init()
{
	_para_small_path_thresh = 2;
	_para_same_point_thresh = 1.0f;
	_para_small_path_generating_polygon = 2;
	_para_gen_edge_thresh = 0.5;
	_para_find_line_empty_end_thresh = 1.0;
	_para_shape_touch_thresh = 0.4f;
	//_para_match_accept_thresh = 0.7f;
	_para_match_accept_thresh = 0.9f;
	_para_rigid_transform_point_equal_thresh = 2.5f;
	_para_rigid_candicate_radius_thresh = 8.f;//50.0f;//10.0f;
	_para_sample_straight_line = 0.5;
	_para_fitting_score_thresh = 0.6;//0.8f;
	_para_fitting_score_thresh_single = 0.8f;
	_para_fitting_image_size_thresh = 512;
	_para_fitting_image_large_size_thresh = 640;
}


void ANIM_PARA::init()
{
	_ratio_interval = 0.05f;
}