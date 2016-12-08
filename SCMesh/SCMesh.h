#ifndef _SCMESH_
#define _SCMESH_

#include "Wm4Vector3.h"
#include "../TriMesh/Trimesh.h"
#include "../MyUtil/MyPara.h"
#include "../MyUtil/MyMaterial.h"
typedef Wm4::Vector3f VEC3F;
typedef Wm4::Vector3d VEC3D;
class CVertex
{
public:
	void add_edge(int edgeIdx)
	{
		m_edges.push_back(edgeIdx);
	}
	void add_face(int faceIdx)
	{
		m_faces.push_back(faceIdx);
	}
	std::vector<int> m_edges;	//index in edge vector
	std::vector<int> m_faces;	//index in face vector
};

class CEdge
{
public:
	CEdge()
	{
		m_vertex[0] = m_vertex[1] = 0;
		m_faces[0] = m_faces[1] = 0;
	}
	CEdge(int v1, int v2, int f1, int f2)
	{
		m_vertex[0] = v1;
		m_vertex[1] = v2;
		m_faces[0] = f1;
		m_faces[1] = f2;
	}
	int m_vertex[2];
	int m_faces[2];
};

class CSCMesh
{
public:
	CSCMesh();
	virtual ~CSCMesh();
public:
	bool read_file(const char *filename);
	bool write_file(const char *filename);
	void destroy();
	void initialize();
	void update();
	void face_normal();	//get the normal of each face
	void detect_crease_edge();
	void createMesh();
	void destroyTriMesh();
protected:
	void build_edge_info();
	void destroy_edge_info();
	void hyst_connect(int edgeIdx,std::vector<bool> &creaseFlag);
	//hyst connect starting from one of the edge's 2 end points
	void hyst_connect_from_oneVertex(int edgeIdx,int vIdx,std::vector<bool> &creaseFlag);
	float get_dihedral_angle(int edgeIdx);
public:	//render functions
	void render(const RENDER_PARA &para);
	//this function is used to select crease edges
	void render_base_mesh(const RENDER_PARA &para);
protected:
	void render_feature_lines(const RENDER_PARA &para);	//render contour and suggestive contour
	void render_crease_edge(const RENDER_PARA &para);
	void render_silhouette(const vector<float> &ndotv,const RENDER_PARA &para);
	void render_wire_frame(const RENDER_PARA &para);
	void begin_wireframe();
	void end_wireframe();
	// Draw triangle strips.  They are stored as length followed by values.
	void draw_tstrips();
	// Takes a scalar field and renders the zero crossings, but only where
	// test_num/test_den is greater than 0.
	void draw_isolines(const vector<float> &val,
		const vector<float> &test_num,
		const vector<float> &test_den,
		const vector<float> &ndotv,
		bool do_bfcull, bool do_hermite,
		bool do_test, float fade);
	// Draw part of a zero-crossing curve on one triangle face, but only if
	// "test_num/test_den" is positive.  v0,v1,v2 are the indices of the 3
	// vertices, "val" are the values of the scalar field whose zero
	// crossings we are finding, and "test_*" are the values we are testing
	// to make sure they are positive.  This function assumes that val0 has
	// opposite sign from val1 and val2 - the following function is the
	// general one that figures out which one actually has the different sign.
	void draw_face_isoline2(int v0, int v1, int v2,
		const vector<float> &val,
		const vector<float> &test_num,
		const vector<float> &test_den,
		bool do_hermite, bool do_test, float fade);
	// See above.  This is the driver function that figures out which of
	// v0, v1, v2 has a different sign from the others.
	void draw_face_isoline(int v0, int v1, int v2,
		const vector<float> &val,
		const vector<float> &test_num,
		const vector<float> &test_den,
		const vector<float> &ndotv,
		bool do_bfcull, bool do_hermite,
		bool do_test, float fade);
	// Draw the ridges (valleys) of the mesh
	void draw_mesh_ridges(bool do_ridge, const vector<float> &ndotv,
		bool do_bfcull, bool do_test, float thresh);
	void draw_face_ridges(int v0, int v1, int v2,
		bool do_ridge,
		const vector<float> &ndotv,
		bool do_bfcull, bool do_test, float thresh);
	void draw_segment_ridge(int v0, int v1, int v2,
		float emax0, float emax1, float emax2,
		float kmax0, float kmax1, float kmax2,
		float thresh, bool to_center);
protected:	
	// Compute a "feature size" for the mesh: computed as 1% of
	// the reciprocal of the 10-th percentile curvature
	void compute_feature_size();
protected:
	// Color the mesh by curvatures
	void compute_curv_colors();
	// Similar, but gray-scale mapping of mean curvature H
	void compute_gcurv_colors();
	// Compute per-vertex n dot l, n dot v, radial curvature, and
	// derivative of curvature for the current view
	void compute_perview(vector<float> &ndotv, vector<float> &kr,
		vector<float> &sctest_num, vector<float> &sctest_den,
		vector<float> &shtest_num, vector<float> &q1,
		vector<vec2> &t1, vector<float> &Dt1q1,const RENDER_PARA &para,
		bool extra_sin2theta = false);
	void update_view_pos();
	// Find a zero crossing using Hermite interpolation
	float find_zero_hermite(int v0, int v1, float val0, float val1,
		const vec &grad0, const vec &grad1);
	// Compute gradient of (kr * sin^2 theta) at vertex i
	vec gradkr(int i);
	void draw_mesh_ph(bool do_ridge, const vector<float> &ndotv, bool do_bfcull,
		bool do_test, float thresh);
	void draw_face_ph(int v0, int v1, int v2, bool do_ridge,
		const vector<float> &ndotv, bool do_bfcull,
		bool do_test, float thresh);
public:	//get and set interfaces
	TriMesh *getTriMesh() const{return m_pTriMesh;}
	TriMesh *getOriMesh() const {return m_pOriTriMesh;}
	void setTriMesh(TriMesh *pMesh);
protected:
	TriMesh *m_pTriMesh;
	double m_feature_size;
	//new added, for deformation
	TriMesh *m_pOriTriMesh;
public:
	//for rendering
	point m_viewpos;    // Current view position
	Color m_meshColor;	//colors for rendering the mesh
	Color m_edgeColor;	//colors for rendering wireframe	
	Color m_lineColor;	//colors for rendering lines
	vector<Color> m_curv_colors, m_gcurv_colors;	//colors for rendering curvature(color or gray)
	// Toggles for tests we perform
	bool m_test_c, m_test_sc;	//test suggestive contour and contour
	bool m_test_rv;				//test ridge and valley
	bool m_bUseHermite;	//use Hermite interpolation;
	double m_sug_thresh;	//suggestive contour threshold
	double m_rv_thresh;		//ridge/valley threshold
	double m_crease_high_thresh;	//high threshold for detecting crease edge
	double m_crease_low_thresh;	//low threshold for detecting crease edge
	double m_featureLine_width;
public:
	std::vector<CEdge*> m_pEdges;
	std::vector<int> m_creaseEdges;	//edge idx in m_pEdges
	std::vector<CVertex*> m_pVertices;
	vector<vec> m_face_normals;
	vector<double> m_face_areas;
public:
	CMaterial m_material;
};

#endif