#ifndef _MY_UTIL_H
#define _MY_UTIL_H

#include <vector>
#include "MyType.h"
#include "opencv/cxcore.h"
#include "MyConsts.h"

class CDrawing;
class CProxyPrim;
class CProxyCamera;
class CArticulatedProxies;
class TriMesh;

namespace MyUtil
{
	int tess_poly_2D(const std::vector<VEC2F> &poly, std::vector<int> &vTriangle, int close_flag=1);
	int tess_poly_2D(const std::vector<VEC2D> &poly, std::vector<int> &vTriangle, int close_flag=1);
	int tess_poly_2D_withHole(const std::vector<VEC2F> &pts, const std::vector<int> &vPolyStart, 
		const std::vector<int> &vCloseFlags,std::vector<int> &vTriangle);
	int tess_poly_3D(const std::vector<VEC3D> &poly, std::vector<int> &vTriangle, int close_flag=1);

	bool isPtInsidePolygon(const VEC2F *poly_pts, int n, const VEC2F &pt);
	bool isPolyInsidePoly(const VEC2F *small_poly, int m, const VEC2F *big_poly, int n);
	int getOuterPoly(const std::vector<VEC2F> &pts, const std::vector<int> &vPolyStart);
	bool isPtInsidePolygon(const VEC3F *poly_pts, int n, const VEC3F &pt);
	bool isPtInsidePolygon(const VEC2D *poly_pts, int n, const VEC2D &pt);
	bool isPtInsidePolygon(const VEC3D *poly_pts, int n, const VEC3D &pt);

	VEC2F proj_pt_2_seg(const VEC2F seg_ends[2], const VEC2F &pt);
	float segPtDist2(const VEC2F &pt, const VEC2F seg[2], VEC2F &nearest_pt);

	bool isRectSeparate(const float rect1[4], const float rect2[4]);//left top right bottom
	bool isRectSeparate(const double rect1[4], const double rect2[4]);//left top right bottom

	int calcSubMat(const cv::Mat &float_mat, int left, int top, int subW, int subH, 
				cv::Mat &subMat);
	int quantitizeMat(const cv::Mat &float_mat, int bin_num, cv::Mat &subMat);
	float averMat(const cv::Mat &float_mat);
	void cvMat_2_floatVec(const cv::Mat &float_mat, float *outVec);

	void addEqualVec(std::vector<int> &left, const std::vector<int> &right);
	void addEqualVec(std::vector<float> &left, const std::vector<float> &right);
	void initVec(std::vector<int> &tgt, int val=0);
	void counts2Probs(const std::vector<int> &vCounts, std::vector<float> &vProbs);
	void normalizeVec(std::vector<float> &probVec);

	void principalDir(const VEC2F *pts, int n, VEC2F &maxDir, VEC2F &minDir);

	void render_box(const VEC3D* points, bool bRenderEdge, float r, float g, float b,float a,
		float edge_r, float edge_g, float edge_b);

	SEG2F build_SEG(const VEC2F &negPt, const VEC2F &posPt);
	SEG3F build_SEG(const VEC3F &negPt, const VEC3F &posPt);
	SEG2D build_SEG(const VEC2D &negPt, const VEC2D &posPt);
	SEG3D build_SEG(const VEC3D &negPt, const VEC3D &posPt);

	LINE2D build_line(const VEC2D &negPt, const VEC2D &posPt);

	float pt_line_dist(const VEC3F &pt, const LINE3F &line);
	double pt_line_dist(const VEC3D &pt, const LINE3D &line);
	VEC3F proj_pt_2_line(const VEC3F &pt, const LINE3F &line);
	VEC3D proj_pt_2_line(const VEC3D &pt, const LINE3D &line);
	VEC3D proj_pt_2_plane(const VEC3D &pt, const VEC4D &plane);

	VEC2F project(const VEC3F &origin, const VEC3F &pt,const VEC3F &axis1, const VEC3F &axis2);
	VEC3F unproject(const VEC3F &origin, const VEC2F &proj, const VEC3F &axis1, const VEC3F &axis2);
	VEC2D project(const VEC3D &origin, const VEC3D &pt,const VEC3D &axis1, const VEC3D &axis2);
	VEC3D unproject(const VEC3D &origin, const VEC2D &proj, const VEC3D &axis1, const VEC3D &axis2);

	void gatherLines(const std::vector<SEG2F> &segs, LINE2F lines[2]);
	LINE2F perturbLine(const LINE2F &src, float delta_angle);

	void link_drawing_cam_proxy(CDrawing *pDrawing, CProxyCamera *pCam, CProxyPrim *pProxy);
	void link_drawing_proxy(CDrawing *pDrawing, CProxyPrim *pProxy);
	void real_pt_2_render_pt(const double real_pt[2], double render_pt[2], const double real_vp[4], const int vp[4]);

	void computeCorners(const VEC3D &c, const VEC3D axis[3], const double hls[3], VEC3D corners[8]);
	void calcProxyNewPara(const CProxyPrim *pProxy, const double x[PROXY_VAR_NUM],VEC3D &new_c, VEC3D new_axie[3], double new_hl[3]);
	void calcArticulatedNewPara(const CArticulatedProxies *pArt, const double x[PROXY_VAR_NUM],VEC3D &new_c, VEC3D new_axie[3], double new_hl[3]);
	void drawCircle(const VEC3D &c, const VEC3D &n, double r, bool bFill);
	void drawCircle(float x, float y, float r, bool bFill=true);
	VEC3D relLocal_2_global(const VEC3D &rel_local, const VEC3D &c, const VEC3D axes[3], const double hls[3]);

	void getAffineMat(const VEC2D &oa,const VEC2D &ob,const VEC2D &oc,
		const VEC2D &na,const VEC2D &nb,const VEC2D &nc,
		MAT2D &outA, VEC2D &outT);
	void cluster_segs_dirBased(const std::vector<SEG2D> &vSegs, std::vector<std::vector<SEG2D>> &vvSegGrps, float angle_thresh);
	void cluster_segs_dirLengthBased(const std::vector<SEG2D> &vSegs, 
		std::vector<std::vector<SEG2D>> &vvSegGrps, float angle_thresh, float len_thresh);
	void cluster_segs_dirLengthBased(const std::vector<SEG2D> &vSegs, 
		const std::vector<int> &vSegIndices,
		std::vector<std::vector<SEG2D>> &vvSegGrps, 
		std::vector<std::vector<int>> &vvIndiceGrps, float angle_thresh, float len_thresh);
	float aver_seg_grp_len(const std::vector<SEG2D> &vGrpSegs);
	LINE2D perturbLine(const LINE2D &src, float delta_angle);


	VEC3D calcDirFromVanishPt(const MAT3D &K, const VEC2D &vpt);
	VEC3D pixelOnPlane(const VEC2D &pixel, const VEC4D &plane, const CProxyCamera *pCam);
	VEC3D calcCenter(const VEC3D *c, int n);
	void proj_pts_onto_axis(const VEC3D *pts, int n, const VEC3D axes[3], VEC3D &global_c, double hls[3]);
	TriMesh *clone_mesh(const TriMesh *pMesh);
	void build_plane_UV(const VEC3D &plane_normal, VEC3D &plane_U, VEC3D &plane_V);
	VEC2D getUV(const VEC3D &plane_pt, const VEC3D &U_dir, const VEC3D &V_dir, const VEC3D &pt);
	void removeDuplicatedPts(std::vector<VEC3D> &vPts);
	double signed_dist_pt_2_plane(const VEC4D &plane, const VEC3D pt);
	void adjust_plane_pts(VEC3D *pts, int n, const VEC3D &new_plane_normal);
	VEC3D intersect_line_plane(const VEC3D &pt1, const VEC3D &pt2, const VEC4D& abcd);
	void render3DArrow(const VEC3F& world_pos,const VEC3F& axis,float length,float width);
}

#endif