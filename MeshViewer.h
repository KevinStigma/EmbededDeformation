#ifndef MESHVIEWER_H
#define MESHVIEWER_h
//#include "Shader/ShaderManager.h"
#include "Data_Structure.h"
#include "Shader/ShaderManager.h"
#include <Windows.h>
#include <qgl.h>
#include <GL/glut.h>
#include <GL/GLU.H>
#include <GL/GLAUX.h>
#include <QMouseEvent>
#include <QLineEdit>
#include "Deformer.h"
#include "ArcBall.h"
#include "drag_rect.h"
#include "MyUtil/MyPara.h"
#include "TriMesh/TriMesh.h"
#include "glm.h"

class CSCMesh;
class MeshViewer : public QGLWidget
{
	Q_OBJECT
public:
	MeshViewer(QWidget* parent = 0);
	~MeshViewer();
	bool importTriMesh(const std::string&file_name);
	TriMesh* readTriMesh(const std::string &file_name);
	bool importTriMeshBatch(const std::string&filename);
	void clearModel();
	void init_deform();
	void resetView();
	void setLineWeight();
	void weightDefine();

	inline void setMeshShow(bool is){mesh_show=is;}
	inline void setNormalShow(bool is){normal_show=is;}
	inline void setTriangleShow(bool is){triangle_show=is;}
	inline void setGraphNodeShow(bool is){graph_show=is;}
	inline void setDeformNodeShow(bool is){deformnode_show=is;}
	inline void setKeyStatus(KEYSTATUS key_s){key_status=key_s;}
	inline void cancel_sel_box(){cur_boxID=-1;}
	inline void setBoxShow(bool is) {box_show=is;}
	inline std::vector<BoundingBox>* getBoundingBoxes(){return &AABB;}
	inline const std::vector<BoundingBox>* getBoundingBoxes()const {return &AABB;}
	inline std::vector<TriMesh*>* getProxyMesh(){return &m_proxy_mesh;}
	inline const std::vector<TriMesh*>* getProxyMesh()const{return &m_proxy_mesh;}
	inline VecList(Vec3)* getGraphNode(){return &m_graph;}
	inline const VecList(Vec3)* getGraphNode()const{return &m_graph;}
	void deform();
	void plus_testID();
	void minus_testID();
	void plus_modifyX();
	void minus_modifyX();
	void plus_modifyY();
	void minus_modifyY();
	void generateNew_AABB();
	void delete_AABB();
	void delete_boxpair();
	void resetModel();
	void setTranslateAxis(int axis);
	void setLineEdit(QLineEdit*rotl,QLineEdit*regl,QLineEdit*conl);
	void setGraphEdge(std::vector<VecSet(int)>* ge);
	void setDeformVerts(std::vector<std::vector<Deformer::DeformVerts>>* dv);
	void setShaderMode(int mode);
	void write_mesh();
	void outputGraphInfo(const std::string& filename)const;
	void inputGraphInfo(const std::string&filename);
	void outputBoxes(const std::string&filename)const;
	void inputBoxes(const std::string&filename);
	void readRenderInfo(const std::string& filename);
protected:
	struct MyArcball
	{
		ArcBallT    ArcBall;
		Matrix4fT   Transform;
		Matrix3fT   LastRot;
		Matrix3fT   ThisRot;
		Point2fT    MousePt;  
		float		mRadius;
		POINT		mLastMousePos;
		int			buttonstate;
		MyArcball(float w=800,float h=766):ArcBall(w,h),mRadius(0.0),buttonstate(0)
		{
			Matrix3fSetIdentity(&LastRot); 
			Matrix3fSetIdentity(&ThisRot); 
			Transform.M[0]=1.0f;Transform.M[1]=0.0f;Transform.M[2]=0.0f;Transform.M[3]=0.0f;
			Transform.M[4]=0.0f;Transform.M[5]=1.0f;Transform.M[6]=0.0f;Transform.M[7]=0.0f;
			Transform.M[8]=0.0f;Transform.M[9]=0.0f;Transform.M[10]=1.0f;Transform.M[11]=0.0f;
			Transform.M[12]=0.0f;Transform.M[13]=0.0f;Transform.M[14]=0.0f;Transform.M[15]=1.0f;
		}
	};
	typedef struct mBoxPair 
	{
		std::vector<int> box_id_list;
		std::vector<Vec3> trans_dir;
		Vec3 center;
	}BoxPair;
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void initLighting();
	void initMaterial();
	void recenterMesh(TriMesh*p_triMesh);
	void reCalNormal4TriMesh(TriMesh*triMesh);
	void setMaterial(const Material& material);
	Vec2 calAixs2D(const Vec3& start,const Vec3& end);
	void adjustViewBasedProxy();
	bool checkTriLength(TriMesh* tri_mesh,double length_threshold);

	void delete_all_proxy();
	void reCenterProxy();

	virtual void mousePressEvent(QMouseEvent*);
	virtual void mouseReleaseEvent(QMouseEvent *);
	virtual void mouseMoveEvent(QMouseEvent *);

	void renderModel();
	void renderAABB();
	void renderGraphNode();
	void renderCSCMesh();
	void render_boxpair();
	void sel_del_point();
	void sel_AABB();
	void sel_Node();
	void sel_BoxPair();
	void setCSCMesh(std::vector<TriMesh*>& tri_proxy_mesh);
	void subdivideMesh(TriMesh* tri_mesh);
	void scaleMesh(TriMesh*tri_mesh,double scale_v);
	void deform_noPair(int posx,int posy,int cur_box_id);
	void deform_Pair(int posx,int posy,int cur_box_id,int bp_id);
	void output_mesh(const std::string& filename);
private:
	int testId;
	Vec3 pos;
	Vec3 target;
	Vec3 up;
	Vec3 m_init_pos;
	Vec3 mesh_center;
	bool mesh_show,triangle_show,normal_show,graph_show,deformnode_show,box_show;
	MyArcball  z_arcball;
	Light	  light_des;
	Material gray_material;
	Material yellow_material;
	
	std::vector<TriMesh*> m_proxy_mesh;
	std::vector<CSCMesh*> m_csc_mesh;
	VecList(Vec3) m_graph;
	VecList(bool) m_is_sel_provert;
	std::vector<VecSet(int)>* m_GraphEdge;
	std::vector<std::vector<Deformer::DeformVerts>>* m_deformVerts;
	CShaderManager* m_shader_manager;

	VecList(vec3) m_init_vertices;
	VecList(Vec3) m_init_graph;

	std::vector<std::set<int>>  m_sel_provert;
	std::vector<int> m_sel_node;
	std::vector<BoxPair> box_pair;

	std::vector<BoundingBox> AABB;
	int cur_boxID,cur_proxyID;
	int m_axis;
	Vec3 m_axis_dir[3];
	Deformer deformer;
	drag_rect* m_pDragRect;
	KEYSTATUS key_status;
	float modify_y;
	float modify_x;
	float normal_length;
	float translate_step;
	double scale_step;
	double bp_sphere_radius;
	bool load_success;
	double axis_length;
	double AABB_trans;
	int curNodeId;
	int curNodeProxy;
	double model_matrix[16];
	double proj_matrix[16];

	RENDER_PARA m_render_para;

	QPoint start_pt;
	QPoint cur_pt;
	QLineEdit *rotEdit,*regEdit,*conEdit;
};
#endif