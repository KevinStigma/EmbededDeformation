#include "MeshViewer.h"
#include "../SCMesh/SCMesh.h"
//#include "../SCMesh/PolyOP.h"
#include <limits> 

MeshViewer::MeshViewer(QWidget* parent) :QGLWidget(parent),mesh_center(0,0,0),mesh_show(true),normal_show(false),triangle_show(true),
	graph_show(false),deformnode_show(false),testId(0),m_pDragRect(NULL),key_status(KEY_NONE),cur_boxID(-1),cur_proxyID(-1),m_axis(-1),
	rotEdit(NULL),regEdit(NULL),conEdit(NULL),translate_step(0.1),scale_step(0.001),m_init_pos(0,0,100),normal_length(0.5),
	load_success(false),curNodeId(-1),curNodeProxy(-1),m_GraphEdge(NULL),box_show(true),m_arcball(800,600)
{
	setFixedSize(800,766);
	initMaterial();

	m_axis_dir[0]=Vec3(1,0,0);
	m_axis_dir[1]=Vec3(0,1,0);
	m_axis_dir[2]=Vec3(0,0,1);
	m_render_para.init();
	m_shader_manager=new CShaderManager;

	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
		{
			model_matrix[i*4+j]=model_matrix[i*4+j]=0;
			if(i==j)
				model_matrix[i*4+j]=model_matrix[i*4+j]=1;
		}
}

MeshViewer::~MeshViewer()
{
	delete_all_proxy();
	
	SAFE_DELETE(m_shader_manager);
}

void MeshViewer::setLineEdit(QLineEdit*rotl,QLineEdit*regl,QLineEdit*conl)
{
	if(!rotl||!regl||!conl)
		return;
	rotEdit=rotl;
	regEdit=regl;
	conEdit=conl;
}

void MeshViewer::setLineWeight()
{
	double rotw,regw,conw;
	deformer.getWeights(rotw,regw,conw);
	rotEdit->setText(QString::number(rotw,'g'));
	regEdit->setText(QString::number(regw,'g'));
	conEdit->setText(QString::number(conw,'g'));
}

void MeshViewer::setGraphEdge(std::vector<VecSet(int)>*ge)
{
	if(!ge)
		return;
	m_GraphEdge=ge;
}
void MeshViewer::setDeformVerts(std::vector<std::vector<Deformer::DeformVerts>>* dv)
{
	if(!dv)
		return;
	m_deformVerts=dv;
}

void MeshViewer::setShaderMode(int mode)
{
	std::cout<<mode<<std::endl;
	if(mode==0)
		m_render_para._shader_mode=CShaderManager::ShaderType::none;
	else if(mode==1)
		m_render_para._shader_mode=CShaderManager::ShaderType::orennayar;
	else if(mode==2)
		m_render_para._shader_mode=CShaderManager::ShaderType::glass;
	else
		m_render_para._shader_mode=CShaderManager::ShaderType::xray;
}
void MeshViewer::weightDefine()
{
	double rotw,regw,conw;
	rotw=rotEdit->text().toDouble();
	regw=regEdit->text().toDouble();
	conw=conEdit->text().toDouble();
	deformer.setWeights(rotw,regw,conw);
}

void MeshViewer::initializeGL()
{
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glClearDepth(1.0);
	glEnable(GL_DEPTH_TEST);
	initLighting();
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	 glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	 m_shader_manager->create("Shader\\");
	//glBlendFunc(GL_ONE,GL_ZERO);
}
void MeshViewer::resizeGL(int w, int h)
{
	glViewport(0, 0, (GLint)w, (GLint)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, 0.01f, 2000.0f);
}

void MeshViewer::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
    //glMultMatrixd(proj_matrix);
	//pos=Vec3(0.0,0.0,100);
	pos=m_init_pos;
	target=Vec3(0.0f,0.0f,0.0f);
	up=Vec3(0.0f,1.0f,0.0f);

	//Vec3 normal_dir=(target-pos).normalized();
	//pos+=normal_dir*m_arcball.mRadius;
	pos(2)=pos(2)*m_arcball.mRadius;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(pos(0),pos(1),pos(2),target(0),target(1),target(2),up(0),up(1),up(2));

	glTranslatef(m_arcball.translate_x,m_arcball.translate_y,0.0f);
	glMultMatrixf(m_arcball.Transform.M);

	glPointSize(2.0f);
	glPushMatrix();

	if(load_success)
	{
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); // Make round points, not square points
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);  // Antialias the lines
		//renderCSCMesh();
		renderModel();
		renderGraphNode();
		renderAABB();
		render_boxpair();
	}
	glPopMatrix();
	glDepthMask(GL_TRUE);
	if((key_status==KEY_SELECT_POINT||key_status==KEY_DELETE_POINT||key_status==KEY_SELECT_BOX||key_status==KEY_SEL_NODE||key_status==KEY_SEL_BOXPAIR)
		&&m_pDragRect)
		m_pDragRect->render();
	update();
}

void MeshViewer::mousePressEvent(QMouseEvent* event)
{
	QPoint qpoint=mapFromGlobal(QCursor::pos());
	int posx=qpoint.x();
	int posy=qpoint.y();
	if(key_status==KEY_NONE)
	{
		if(event->button()==Qt::LeftButton)
		{
			m_arcball.MousePt.s.X = posx;
			m_arcball.MousePt.s.Y = posy;

			m_arcball.LastRot = m_arcball.ThisRot;                  
			m_arcball.ArcBall.click(&m_arcball.MousePt);
			m_arcball.button_status=1;
		}
		else if(event->button()==Qt::RightButton)
			m_arcball.button_status=2;
		m_arcball.mLastMousePos.s.X=static_cast<LONG>(qpoint.x());
		m_arcball.mLastMousePos.s.Y=static_cast<LONG>(qpoint.y());
	}
	else if(key_status==KEY_SELECT_POINT||key_status==KEY_DELETE_POINT||key_status==KEY_SELECT_BOX||key_status==KEY_SEL_NODE||key_status==KEY_SEL_BOXPAIR)
	{
		SAFE_DELETE(m_pDragRect);
		m_pDragRect = new drag_rect(posx,posy);
		//std::cout<<key_status<<std::endl;
	}
	else if(key_status==KEY_DRAG_BOX)
	{
		start_pt.setX(posx);
		start_pt.setY(posy);
	}
	setMouseTracking(true);
	grabMouse();

}

void MeshViewer::mouseReleaseEvent(QMouseEvent *)
{
	QPoint qpoint=mapFromGlobal(QCursor::pos());
	int posx=qpoint.x();
	int posy=qpoint.y();
	m_arcball.button_status=0;

	if(key_status==KEY_SELECT_POINT||key_status==KEY_DELETE_POINT)
		sel_del_point();
	else if(key_status==KEY_SELECT_BOX)
		sel_AABB();
	else if(key_status==KEY_SEL_NODE)
		sel_Node();
	else if(key_status==KEY_SEL_BOXPAIR)
		sel_BoxPair();
	else if(key_status==KEY_DRAG_BOX)
	{
		//if(AABB[cur_boxID].bp_id==-1&&m_axis!=-1)
			//deform_noPair(posx,posy,cur_boxID);
		//else if(AABB[cur_boxID].bp_id!=-1)
			//deform_Pair(posx,posy,cur_boxID,AABB[cur_boxID].bp_id);
	}


	SAFE_DELETE(m_pDragRect);
	releaseMouse();
	setMouseTracking(false);
}

void MeshViewer::output_mesh(const std::string& filename)
{

}

void MeshViewer::mouseMoveEvent(QMouseEvent * event)
{
	QPoint qpoint=mapFromGlobal(QCursor::pos());
	int posx=qpoint.x();
	int posy=qpoint.y();

	if(key_status==KEY_NONE)
	{
		if(m_arcball.button_status==1)
		{
			m_arcball.MousePt.s.X = posx;
			m_arcball.MousePt.s.Y = posy;
			Quat4fT     ThisQuat;
			m_arcball.ArcBall.drag(&m_arcball.MousePt, &ThisQuat);                       
			Matrix3fSetRotationFromQuat4f(&m_arcball.ThisRot, &ThisQuat);  
			Matrix3fMulMatrix3f(&m_arcball.ThisRot, &m_arcball.LastRot);   
			Matrix4fSetRotationFromMatrix3f(&m_arcball.Transform, &m_arcball.ThisRot); 
		}
		else if(m_arcball.button_status==2)
		{
			float dx = 0.1f*static_cast<float>(qpoint.x() - m_arcball.mLastMousePos.s.X);
			float dy = 0.1f*static_cast<float>(qpoint.y() - m_arcball.mLastMousePos.s.Y);
			m_arcball.translate_x+=dx;
			m_arcball.translate_y-=dy;
		}
		m_arcball.mLastMousePos.s.X = posx;
		m_arcball.mLastMousePos.s.Y = posy;
	}
	else if(key_status==KEY_SELECT_POINT||key_status==KEY_DELETE_POINT||key_status==KEY_SELECT_BOX||key_status==KEY_SEL_NODE||key_status==KEY_SEL_BOXPAIR)
	{
		if (m_pDragRect) 
			m_pDragRect->move_to(posx, posy);
		else 
			m_pDragRect = new drag_rect((float)posx, (float)posy);
		//std::cout<<key_status<<std::endl;
	}
	else if(key_status==KEY_DRAG_BOX)
	{
		if(cur_boxID==-1)
			return;

		if(AABB[cur_boxID].bp_id==-1&&m_axis!=-1)
			deform_noPair(posx,posy,cur_boxID);
		else if(AABB[cur_boxID].bp_id!=-1)
			deform_Pair(posx,posy,cur_boxID,AABB[cur_boxID].bp_id);
	}
}

void MeshViewer::wheelEvent(QWheelEvent *event)
{
	float delta=(float)event->delta()/1500.0*scale_step*2;
	m_arcball.mRadius *=  (1-delta);
}

void MeshViewer::sel_del_point()
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble mv_data[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, mv_data);
	GLdouble proj_data[16];	
	glGetDoublev(GL_PROJECTION_MATRIX, proj_data);

	double lx,ly,lz,sx,sy,sz;
	for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
	{
		TriMesh*tri_mesh=m_proxy_mesh[pid];
		for(size_t i=0;i<tri_mesh->vertices.size();i++)
		{
			const std::vector<vec3>& vertices=tri_mesh->vertices;
			lx=vertices[i][0]; 
			ly=vertices[i][1];
			lz=vertices[i][2];
			gluProject(lx,ly,lz,mv_data,proj_data,viewport,&sx,&sy,&sz);
			sy = viewport[3] - sy;
			if ((sx-m_pDragRect->rect_[0])*(sx-m_pDragRect->rect_[2]) > 0
				|| (sy-m_pDragRect->rect_[1])*(sy-m_pDragRect->rect_[3])>0)
			{
				continue;
			}
			if(key_status==KEY_SELECT_POINT)
			{
				m_is_sel_provert[pid][i]=true;
				m_sel_provert[pid].insert(i);
				//m_sel_provert[pid].push_back()
			}
			else if(key_status==KEY_DELETE_POINT)
			{
				m_is_sel_provert[pid][i]=false;
				m_sel_provert[pid].erase(i);
			}
		}
	}
}

void MeshViewer::sel_AABB()
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble mv_data[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, mv_data);
	GLdouble proj_data[16];	
	glGetDoublev(GL_PROJECTION_MATRIX, proj_data);

	double nearest_z=1.0;
	double lx,ly,lz,sx,sy,sz;
	for(size_t i=0;i<AABB.size();i++)
	{
		Vec3 box_cent=(AABB[i].bot_pos+AABB[i].top_pos)/2.0;
		lx=box_cent(0); 
		ly=box_cent(1);
		lz=box_cent(2);
		gluProject(lx,ly,lz,mv_data,proj_data,viewport,&sx,&sy,&sz);
		sy = viewport[3] - sy;
		if ((sx-m_pDragRect->rect_[0])*(sx-m_pDragRect->rect_[2]) > 0
			|| (sy-m_pDragRect->rect_[1])*(sy-m_pDragRect->rect_[3])>0)
		{
			continue;
		}
		if (sz<nearest_z)
		{
			cur_boxID=i;
			nearest_z=sz;
		}
	}
}

void MeshViewer::sel_BoxPair()
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble mv_data[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, mv_data);
	GLdouble proj_data[16];	
	glGetDoublev(GL_PROJECTION_MATRIX, proj_data);

	double lx,ly,lz,sx,sy,sz;
	BoxPair bp;
	Vec3 center(0,0,0);
	for(size_t i=0;i<AABB.size();i++)
	{
		Vec3 box_cent=(AABB[i].bot_pos+AABB[i].top_pos)*0.5;
		lx=box_cent(0); 
		ly=box_cent(1);
		lz=box_cent(2);
		gluProject(lx,ly,lz,mv_data,proj_data,viewport,&sx,&sy,&sz);
		sy = viewport[3] - sy;
		if ((sx-m_pDragRect->rect_[0])*(sx-m_pDragRect->rect_[2]) > 0
			|| (sy-m_pDragRect->rect_[1])*(sy-m_pDragRect->rect_[3])>0)
		{
			continue;
		}
		bp.box_id_list.push_back(i);
		center+=box_cent;
	}
	if(!bp.box_id_list.size())
		return;
	center/=bp.box_id_list.size();
	bp.center=center;
	for(int i=0;i<(int)bp.box_id_list.size();i++)
	{
		int bid=bp.box_id_list[i];
		Vec3 box_cent=(AABB[bid].bot_pos+AABB[bid].top_pos)*0.5;
		Vec3 dir=(box_cent-bp.center).normalized();
		bp.trans_dir.push_back(dir);
		int bp_id=box_pair.size();
		AABB[bid].bp_id=bp_id;
	}

	std::cout<<"New box pair has selected:"<<std::endl;
	std::cout<<"box ";
	for(int i=0;i<(int)bp.box_id_list.size();i++)
		std::cout<<bp.box_id_list[i]<<" ";
	std::cout<<std::endl;
	box_pair.push_back(bp);
}


void MeshViewer::sel_Node()
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble mv_data[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, mv_data);
	GLdouble proj_data[16];	
	glGetDoublev(GL_PROJECTION_MATRIX, proj_data);

	double nearest_z=1.0;
	double lx,ly,lz,sx,sy,sz;
	for(int pid=0;pid<m_graph.size();pid++)
	{
		const auto& graph_nodes=m_graph[pid];
		for(size_t i=0;i<graph_nodes.size();i++)
		{
			Vec3 cent=graph_nodes[i];
			lx=cent(0);
			ly=cent(1);
			lz=cent(2);
			gluProject(lx,ly,lz,mv_data,proj_data,viewport,&sx,&sy,&sz);
			sy = viewport[3] - sy;
			if ((sx-m_pDragRect->rect_[0])*(sx-m_pDragRect->rect_[2]) > 0
				|| (sy-m_pDragRect->rect_[1])*(sy-m_pDragRect->rect_[3])>0)
			{
				continue;
			}
			if (sz<nearest_z)
			{
				curNodeId=i;
				curNodeProxy=pid;
				//cur_boxID=i;
				nearest_z=sz;
			}
		}
	}
}

bool MeshViewer::importTriMesh(const std::string&file_name)
{
	delete_all_proxy();

	TriMesh*tri_mesh=readTriMesh(file_name);
	if(!tri_mesh)
	{
		std::cout<<"import mesh failed!"<<std::endl;
		return false;
	}
	m_proxy_mesh.push_back(tri_mesh);

	reCenterProxy();
	m_is_sel_provert.resize(m_proxy_mesh.size());
	for(int i=0;i<(int)m_is_sel_provert.size();i++)
		m_is_sel_provert[i].resize(m_proxy_mesh[i]->vertices.size());

	m_sel_provert.resize(m_is_sel_provert.size());
	adjustViewBasedProxy();

	m_init_vertices.resize(m_proxy_mesh.size());
	for(int i=0;i<m_proxy_mesh.size();i++)
		m_init_vertices[i].assign(m_proxy_mesh[i]->vertices.begin(),m_proxy_mesh[i]->vertices.end());

	load_success=true;
	setCSCMesh(m_proxy_mesh);
	return true;
}

TriMesh* MeshViewer::readTriMesh(const std::string &file_name)
{
	TriMesh* p_tri_mesh=TriMesh::read(file_name.c_str());
	if(!p_tri_mesh)
	{
		std::cout<<"Can't read the mesh:"<<file_name<<std::endl;
		return NULL;
	}
	//subdivideMesh(p_tri_mesh);
	reCalNormal4TriMesh(p_tri_mesh);
	//scaleMesh(p_tri_mesh,100);
	return p_tri_mesh;
}

bool MeshViewer::importTriMeshBatch(const std::string&filename)
{
	delete_all_proxy();
	std::ifstream in(filename);

	std::string mesh_name;
	while(in>>mesh_name)
	{
		TriMesh*tri_mesh=readTriMesh(mesh_name);
		if(!tri_mesh)
		{
			std::cout<<"import mesh failed!"<<std::endl;
			return false;
		}
		m_proxy_mesh.push_back(tri_mesh);
		std::cout<<tri_mesh->vertices.size()<<std::endl;
	}
	in.close();
	//reCenterProxy();

	m_is_sel_provert.resize(m_proxy_mesh.size());
	for(int i=0;i<(int)m_is_sel_provert.size();i++)
		m_is_sel_provert[i].resize(m_proxy_mesh[i]->vertices.size());
	
	m_sel_provert.resize(m_is_sel_provert.size());
	
	m_init_vertices.resize(m_proxy_mesh.size());
	for(int i=0;i<m_proxy_mesh.size();i++)
		m_init_vertices[i].assign(m_proxy_mesh[i]->vertices.begin(),m_proxy_mesh[i]->vertices.end());

	adjustViewBasedProxy();
	load_success=true;
	setCSCMesh(m_proxy_mesh);
	return true;
}

bool MeshViewer::checkTriLength(TriMesh* tri_mesh,double length_threshold)
{
	for(int i=0;i<tri_mesh->faces.size();i++)
	{
		TriMesh::Face face=tri_mesh->faces[i];
		Vec3 vert[3];
		double length[3];
		for(int j=0;j<3;j++)
			vert[j]=Vec3(tri_mesh->vertices[face.v[j]][0],tri_mesh->vertices[face.v[j]][1],tri_mesh->vertices[face.v[j]][2]);
		for(int j=0;j<3;j++)
		{
			length[j]=(vert[j]-vert[(j+1)%3]).norm();
			if(length[j]>length_threshold)
				return false;
		}
	}
	return true;
}


void MeshViewer::scaleMesh(TriMesh*tri_mesh,double scale_v)
{
	if(!tri_mesh)
		return;
	for(int i=0;i<(int)tri_mesh->vertices.size();i++)
	{
		tri_mesh->vertices[i]*=scale_v;
	}
}

void MeshViewer::subdivideMesh(TriMesh* tri_mesh)
{
	if(!tri_mesh)
		return;
	double avg_edge_length=0;
	for(int i=0;i<tri_mesh->faces.size();i++)
	{
		TriMesh::Face face=tri_mesh->faces[i];
		Vec3 vert[3];
		for(int j=0;j<3;j++)
			vert[j]=Vec3(tri_mesh->vertices[face.v[j]][0],tri_mesh->vertices[face.v[j]][1],tri_mesh->vertices[face.v[j]][2]);
		avg_edge_length+=((vert[0]-vert[1]).norm()+(vert[1]-vert[2]).norm()+(vert[0]-vert[2]).norm());
	}
	avg_edge_length/=(tri_mesh->faces.size()*3);

	while(!checkTriLength(tri_mesh,avg_edge_length))
	{
		std::vector<TriMesh::Face> faces;
		int old_vn=tri_mesh->vertices.size();
		for(int i=0;i<tri_mesh->faces.size();i++)
		{
			TriMesh::Face face=tri_mesh->faces[i];
			Vec3 vert[3];
			double length[3];
			for(int j=0;j<3;j++)
				vert[j]=Vec3(tri_mesh->vertices[face.v[j]][0],tri_mesh->vertices[face.v[j]][1],tri_mesh->vertices[face.v[j]][2]);
			double max_length=0.0;
			int max_id1,max_id2;
			for(int j=0;j<3;j++)
			{
				length[j]=(vert[j]-vert[(j+1)%3]).norm();
				if(max_length<length[j])
				{
					max_length=length[j];
					max_id1=j;
					max_id2=(j+1)%3;
				}
			}

			if(max_length>avg_edge_length)
			{
				int tri_id[3];
				for(int j=0;j<3;j++)
				{
					tri_id[j]=face.v[(max_id1+j)%3];
					vert[j]=Vec3(tri_mesh->vertices[tri_id[j]][0],tri_mesh->vertices[tri_id[j]][1],tri_mesh->vertices[tri_id[j]][2]);
				}
				Vec3 new_v=(vert[0]+vert[1])/2;

				TriMesh::Face f;
				int vid=-1;
				for(int j=old_vn;j<tri_mesh->vertices.size();j++)
				{
					Vec3 add_v=Vec3(tri_mesh->vertices[j][0],tri_mesh->vertices[j][1],tri_mesh->vertices[j][2]);
					if((add_v-new_v).norm()<1e-6)
					{
						vid=j;
						break;
					}
				}
				if(vid<0)
				{
					tri_mesh->vertices.push_back(vec3(new_v(0),new_v(1),new_v(2)));
					vid=tri_mesh->vertices.size()-1;
				}
				f.v[0]=tri_id[0];
				f.v[1]=vid;
				f.v[2]=tri_id[2];
				faces.push_back(f);

				f.v[0]=vid;
				f.v[1]=tri_id[1];
				f.v[2]=tri_id[2];
				faces.push_back(f);
			}
			else
			{
				TriMesh::Face f=tri_mesh->faces[i];
				faces.push_back(f);
			}
		}
		tri_mesh->faces.assign(faces.begin(),faces.end());
	}
}


void MeshViewer::adjustViewBasedProxy()
{
	double maxX,maxY,maxZ,minX,minY,minZ;
	double INFINITY=6553500;
	maxX=maxY=maxZ=-INFINITY;
	minX=minY=minZ=INFINITY;

	for(int i=0;i<(int)m_proxy_mesh.size();i++)
	{
		for(int j=0;j<(int)m_proxy_mesh[i]->vertices.size();j++)
		{
			Vec3 vert(m_proxy_mesh[i]->vertices[j][0],m_proxy_mesh[i]->vertices[j][1],m_proxy_mesh[i]->vertices[j][2]);
			maxX=std::max<double>(vert(0),maxX);
			maxY=std::max<double>(vert(1),maxY);
			maxZ=std::max<double>(vert(2),maxZ);
			minX=std::min<double>(vert(0),minX);
			minY=std::min<double>(vert(1),minY);
			minZ=std::min<double>(vert(2),minZ);
		}
	}

	double ZL=maxZ-minZ,XL=maxX-minX,YL=maxY-minY;
	double refL=(ZL+XL+YL)/3;
	m_init_pos=Vec3(0,0,ZL*4.0);
	scale_step=refL*0.0003;
	translate_step=refL*0.05;
	normal_length=refL*0.05;
	axis_length=refL*0.12;
	AABB_trans=refL*0.004;
	bp_sphere_radius=translate_step*0.2;
}


void MeshViewer::reCenterProxy()
{
	vec3 center(0,0,0);
	int count=0;
	for(int i=0;i<(int)m_proxy_mesh.size();i++)
	{
		for(int j=0;j<(int)m_proxy_mesh[i]->vertices.size();j++)
		{
			center+=m_proxy_mesh[i]->vertices[j];
			count++;
		}
	}

	center/=count;
	std::cout<<center<<std::endl;

	for(int i=0;i<(int)m_proxy_mesh.size();i++)
	{
		for(int j=0;j<(int)m_proxy_mesh[i]->vertices.size();j++)
			m_proxy_mesh[i]->vertices[j]-=center;
	}
}

void MeshViewer::initLighting()
{
	light_des.ambient[0]=1.0f;light_des.ambient[1]=1.0f;light_des.ambient[2]=1.0f;light_des.ambient[3]=1.0f;
	light_des.diffuse[0]=1.0f;light_des.diffuse[1]=1.0f;light_des.diffuse[2]=1.0f;light_des.diffuse[3]=1.0f;
	light_des.specular[0]=1.0f;light_des.specular[1]=1.0f;light_des.specular[2]=1.0f;light_des.specular[3]=1.0f;
	light_des.position[0]=0.0f;light_des.position[1]=800.0f;light_des.position[2]=100.0f;light_des.position[3]=1.0f;

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_des.ambient);				
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_des.diffuse);				
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_des.specular);			
	glLightfv(GL_LIGHT0, GL_POSITION, light_des.position);
	glEnable(GL_LIGHT0);
}

void MeshViewer::initMaterial()
{
	gray_material.Ambient[0]=0.12108f;gray_material.Ambient[1]=0.13282f;gray_material.Ambient[2]=0.15078f;gray_material.Ambient[3]=1.0f;
	gray_material.Diffuse[0]=0.48432f;gray_material.Diffuse[1]=0.53128f;gray_material.Diffuse[2]=0.60312f;gray_material.Diffuse[3]=1.0f;
	gray_material.Specular[0]=1.0f;gray_material.Specular[1]=1.0f;gray_material.Specular[2]=1.0f;gray_material.Specular[3]=1.0f;
	gray_material.Emission[0]=0.0f;gray_material.Emission[1]=0.0f;gray_material.Emission[2]=0.0f;gray_material.Emission[3]=1.0f;

	yellow_material.Ambient[0]=0.198f;yellow_material.Ambient[1]=0.152f;yellow_material.Ambient[2]=0.0752f;yellow_material.Ambient[3]=1.0f;
	yellow_material.Diffuse[0]=0.792f;yellow_material.Diffuse[1]=0.608f;yellow_material.Diffuse[2]=0.3008f;yellow_material.Diffuse[3]=1.0f;
	yellow_material.Specular[0]=1.0f;yellow_material.Specular[1]=1.0f;yellow_material.Specular[2]=1.0f;yellow_material.Specular[3]=1.0f;
	yellow_material.Emission[0]=0.0f;yellow_material.Emission[1]=0.0f;yellow_material.Emission[2]=0.0f;yellow_material.Emission[3]=1.0f;
}

void MeshViewer::setMaterial(const Material& material)
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, material.Ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material.Diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR,material.Specular);
	glMaterialfv(GL_FRONT, GL_EMISSION, material.Emission);

	glMaterialfv(GL_BACK, GL_AMBIENT, material.Ambient);
	glMaterialfv(GL_BACK, GL_DIFFUSE, material.Diffuse);
	glMaterialfv(GL_BACK, GL_SPECULAR,material.Specular);
	glMaterialfv(GL_BACK, GL_EMISSION, material.Emission);

	glMaterialf (GL_FRONT, GL_SHININESS, 16.0f);
	glMaterialf (GL_BACK, GL_SHININESS, 16.0f);
}


void MeshViewer::reCalNormal4TriMesh(TriMesh*triMesh)
{
	if(!triMesh)
		return;

	std::vector<Vec3>face_normals(triMesh->faces.size());
	std::vector<std::vector<int>> vert_2_face(triMesh->vertices.size());
	for(size_t i=0;i<face_normals.size();i++)
	{
		Vec3 vert[3];
		for(size_t j=0;j<3;j++)
		{
			int vid=triMesh->faces[i].v[j];
			vec3 v=triMesh->vertices[vid];
			vert[j]=Vec3(v[0],v[1],v[2]);
			vert_2_face[vid].push_back(i);
		}
		Vec3 nor=((vert[1]-vert[0]).cross(vert[2]-vert[1])).normalized();
		face_normals[i]=nor;
	}
	
	triMesh->normals.resize(triMesh->vertices.size());
	for(size_t i=0;i<vert_2_face.size();i++)
	{
		Vec3 avg_normal(0,0,0);
		for(size_t j=0;j<vert_2_face[i].size();j++)
		{
			int face_id=vert_2_face[i][j];
			Vec3 normal(face_normals[face_id](0),face_normals[face_id](1),face_normals[face_id](2));
			avg_normal+=normal.normalized();
		}
		avg_normal/=vert_2_face[i].size();

		for(size_t j=0;j<3;j++)
			triMesh->normals[i][j]=avg_normal(j);
	}
}



void MeshViewer::init_deform()
{
	if(m_graph.size())
		return;
	deformer.setModel(this);
	deformer.initDeformation();
	m_init_graph.assign(m_graph.begin(),m_graph.end());
}

void MeshViewer::resetView()
{
	m_arcball.mRadius=1;
	m_arcball.translate_x=m_arcball.translate_y=0;

	for(int i=0;i<=3;i++)
		for(int j=0;j<=3;j++)
		{
			int index=i*4+j;
			if(i==j)
				m_arcball.Transform.M[index]=1;
			else
				m_arcball.Transform.M[index]=0;
		}
}

void MeshViewer::plus_testID()
{
	/*if(testId==deform_graph.size()-1)
		testId=0;
	else
		testId++;*/
}

void MeshViewer::minus_testID()
{
	/*if(testId==0)
	testId=deform_graph.size()-1;
	else
	testId--;*/
}

void MeshViewer::generateNew_AABB()
{	
	for(int pid=0;pid<(int)m_sel_provert.size();pid++)
	{
		if(!m_sel_provert[pid].size())
			continue;
		double maxX,maxY,maxZ,minX,minY,minZ;
		double INFINITY=65535;
		//maxX=maxY=maxZ=(std::numeric_limits<double>::min)();
		//minX=minY=minZ=(std::numeric_limits<double>::max)();
		maxX=maxY=maxZ=-INFINITY;
		minX=minY=minZ=INFINITY;
		std::vector<int> container;
		for(auto t=m_sel_provert[pid].begin();t!=m_sel_provert[pid].end();t++)
		{
			vec3 vert=m_proxy_mesh[pid]->vertices[*t];
			maxX=std::max<double>(vert[0],maxX);
			maxY=std::max<double>(vert[1],maxY);
			maxZ=std::max<double>(vert[2],maxZ);
			minX=std::min<double>(vert[0],minX);
			minY=std::min<double>(vert[1],minY);
			minZ=std::min<double>(vert[2],minZ);
			container.push_back(*t);
			m_is_sel_provert[pid][*t]=false;
		}
		BoundingBox bb1;
		if(container.size()!=1)
		{
			BoundingBox bb(Vec3(minX,minY,minZ),Vec3(maxX,maxY,maxZ));
			bb.proxy_id=pid;
			bb1=bb;
		}
		else
		{
			vec3 vert=m_proxy_mesh[pid]->vertices[container[0]];
			BoundingBox bb(Vec3(vert[0]-0.01,vert[1]-0.01,vert[2]-0.01),Vec3(vert[0]+0.01,vert[1]+0.01,vert[2]+0.01));
			bb.proxy_id=pid;
			bb1=bb;
		}
		bb1.vertex_list.assign(container.begin(),container.end());
		AABB.push_back(bb1);
	}

	for(int i=0;i<(int)m_sel_provert.size();i++)
		m_sel_provert[i].clear();
	for(int i=0;i<(int)m_is_sel_provert.size();i++)
	{
		for(int j=0;j<(int)m_is_sel_provert[i].size();j++)
			m_is_sel_provert[i][j]=false;
	}
}

void MeshViewer::setTranslateAxis(int axis)
{
	if(!(axis>=0&&axis<=2))
		return;
	m_axis=axis;
	if(m_axis==0)
		std::cout<<"X axis"<<std::endl;
	else if(m_axis==1)
		std::cout<<"Y axis"<<std::endl;
	else if(m_axis==2)
		std::cout<<"Z aixs"<<std::endl;
}

Vec2 MeshViewer::calAixs2D(const Vec3& start,const Vec3& end)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble mv_data[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, mv_data);
	GLdouble proj_data[16];	
	glGetDoublev(GL_PROJECTION_MATRIX, proj_data);

	double sx, sy, sz;
	gluProject(start(0),start(1),start(2), mv_data, proj_data, viewport, &sx, &sy, &sz);
	sy = viewport[3] - sy-1;//to Windows Pt
	QPoint start_Pt2D(sx,sy);

	gluProject(end(0),end(1),end(2), mv_data, proj_data, viewport, &sx, &sy, &sz);
	sy = viewport[3] -sy-1;
	QPoint end_Pt2D(sx,sy);

	Vec2 dir(end_Pt2D.x()-start_Pt2D.x(),end_Pt2D.y()-start_Pt2D.y());
	dir.normalize();
	return dir;
}

void MeshViewer::deform()
{
	deformer.deform();
}
void MeshViewer::delete_AABB()
{
	if(!AABB.size())
		return;
	AABB.pop_back();
	box_pair.clear();
	for(int i=0;i<(int)AABB.size();i++)
		AABB[i].bp_id=-1;
	cur_boxID=-1;
}
void MeshViewer::delete_boxpair()
{
	if(!box_pair.size())
		return;
	box_pair.pop_back();
	for(int i=0;i<(int)AABB.size();i++)
		AABB[i].bp_id=-1;
	std::cout<<"delete a box pair!"<<std::endl;
}

void MeshViewer::recenterMesh(TriMesh*p_triMesh)
{
	if(!p_triMesh)
		return;
	vec3 center(0,0,0);
	for(int i=0;i<p_triMesh->vertices.size();i++)
		center+=p_triMesh->vertices[i];
	center/=p_triMesh->vertices.size();
	for(int i=0;i<p_triMesh->vertices.size();i++)
		p_triMesh->vertices[i]-=center;
}

void MeshViewer::delete_all_proxy()
{
	for(int i=0;i<m_csc_mesh.size();i++)
		SAFE_DELETE(m_csc_mesh[i]);
	m_csc_mesh.clear();
}

void MeshViewer::renderModel()
{
	if(!load_success)
		return;
	if(m_proxy_mesh.size())
	{
		glDisable(GL_BLEND);
		if(mesh_show)
		{
			glEnable(GL_LIGHTING);
			setMaterial(gray_material);
			glBegin(GL_TRIANGLES);

			for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
			{
				TriMesh* tri_mesh=m_proxy_mesh[pid];
				for(size_t i=0;i<tri_mesh->faces.size();i++)
				{
					for(size_t j=0;j<3;j++)
					{
						int index=tri_mesh->faces[i].v[j];
						glNormal3f(tri_mesh->normals[index][0],tri_mesh->normals[index][1],tri_mesh->normals[index][2]);
						glVertex3f(tri_mesh->vertices[index][0],tri_mesh->vertices[index][1],tri_mesh->vertices[index][2]);
					}
				}
			}
			glEnd();
		}

		if(normal_show)
		{
			glColor3f(0.0f,1.0f,0.0f);
			glLineWidth(1.0f);
			glDisable(GL_LIGHTING);
			glBegin(GL_LINES);

			for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
			{
				TriMesh* tri_mesh=m_proxy_mesh[pid];
				for(size_t i=0;i<tri_mesh->faces.size();i++)
				{
					for(size_t j=0;j<3;j++)
					{
						int index=tri_mesh->faces[i].v[j];
						Vec3 normal(tri_mesh->normals[index][0],tri_mesh->normals[index][1],tri_mesh->normals[index][2]);
						Vec3 vert(tri_mesh->vertices[index][0],tri_mesh->vertices[index][1],tri_mesh->vertices[index][2]);
						glVertex3f(vert(0),vert(1),vert(2));
						glVertex3f(vert(0)+normal(0)*normal_length,vert(1)+normal(1)*normal_length,vert(2)+normal(2)*normal_length);
					}
				}
			}
			glEnd();
		}

		if(triangle_show)
		{
			glDisable(GL_LIGHTING);
			glColor3f(0.0f,0.0f,1.0f);
			glLineWidth(1.0f);
			glBegin(GL_LINES);
			for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
			{
				TriMesh* tri_mesh=m_proxy_mesh[pid];
				for(size_t i=0;i<tri_mesh->faces.size();i++)
				{
					INT3 ind(tri_mesh->faces[i].v[0],tri_mesh->faces[i].v[1],tri_mesh->faces[i].v[2]);
					glVertex3f(tri_mesh->vertices[ind.x][0],tri_mesh->vertices[ind.x][1],tri_mesh->vertices[ind.x][2]);
					glVertex3f(tri_mesh->vertices[ind.y][0],tri_mesh->vertices[ind.y][1],tri_mesh->vertices[ind.y][2]);
					glVertex3f(tri_mesh->vertices[ind.x][0],tri_mesh->vertices[ind.x][1],tri_mesh->vertices[ind.x][2]);
					glVertex3f(tri_mesh->vertices[ind.z][0],tri_mesh->vertices[ind.z][1],tri_mesh->vertices[ind.z][2]);
					glVertex3f(tri_mesh->vertices[ind.y][0],tri_mesh->vertices[ind.y][1],tri_mesh->vertices[ind.y][2]);
					glVertex3f(tri_mesh->vertices[ind.z][0],tri_mesh->vertices[ind.z][1],tri_mesh->vertices[ind.z][2]);
				}
			}
			glEnd();		
		}

		glDisable(GL_LIGHTING);
		glColor3f(1.0f,0.0f,0.0f);
		glPointSize(3.0f);
		glBegin(GL_POINTS);
		for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
		{
			TriMesh* tri_mesh=m_proxy_mesh[pid];
			for(size_t i=0;i<tri_mesh->vertices.size();i++)
			{
				if(m_is_sel_provert[pid][i])
					glVertex3f(tri_mesh->vertices[i][0],tri_mesh->vertices[i][1],tri_mesh->vertices[i][2]);
			}
		}
		glEnd();
	}
}

void MeshViewer::renderCSCMesh()
{
	if(!m_csc_mesh.size())
		return;
	//glEnable(GL_LIGHTING);
	//glDisable(GL_LIGHTING);
	m_shader_manager->bind((CShaderManager::ShaderType)m_render_para._shader_mode);
	for(int i=0;i<m_csc_mesh.size();i++)
		m_csc_mesh[i]->render(m_render_para);
	m_shader_manager->unbind();
	
	glColor3f(1.0f,0.0f,0.0f);
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
	{
		TriMesh* tri_mesh=m_proxy_mesh[pid];
		for(size_t i=0;i<tri_mesh->vertices.size();i++)
		{
			if(m_is_sel_provert[pid][i])
				glVertex3f(tri_mesh->vertices[i][0],tri_mesh->vertices[i][1],tri_mesh->vertices[i][2]);
		}
	}
	glEnd();


	if(normal_show)
	{
		glColor3f(0.0f,1.0f,0.0f);
		glLineWidth(1.0f);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);

		for(size_t pid=0;pid<m_proxy_mesh.size();pid++)
		{
			TriMesh* tri_mesh=m_proxy_mesh[pid];
			for(size_t i=0;i<tri_mesh->faces.size();i++)
			{
				for(size_t j=0;j<3;j++)
				{
					int index=tri_mesh->faces[i].v[j];
					Vec3 normal(tri_mesh->normals[index][0],tri_mesh->normals[index][1],tri_mesh->normals[index][2]);
					Vec3 vert(tri_mesh->vertices[index][0],tri_mesh->vertices[index][1],tri_mesh->vertices[index][2]);
					glVertex3f(vert(0),vert(1),vert(2));
					glVertex3f(vert(0)+normal(0)*normal_length,vert(1)+normal(1)*normal_length,vert(2)+normal(2)*normal_length);
				}
			}
		}
		glEnd();
	}
}


void MeshViewer::renderAABB()
{
	if(box_show)
	{
		glDisable(GL_LIGHTING);
		glPointSize(5.0f);
		for(size_t i=0;i<AABB.size();i++)
		{
			int pid=AABB[i].proxy_id;
			glColor3f(0.0f,1.0f,0.0f);
			glBegin(GL_POINTS);
			for(size_t j=0;j<AABB[i].vertex_list.size();j++)
			{
				int vid=AABB[i].vertex_list[j];
				glVertex3f(m_proxy_mesh[pid]->vertices[vid][0],m_proxy_mesh[pid]->vertices[vid][1],m_proxy_mesh[pid]->vertices[vid][2]);
			}
			glEnd();

			Vec3 center=(AABB[i].top_pos+AABB[i].bot_pos)*0.5;
			glColor3f(0.3725f,0.5137f,0.6549f);
			glPushMatrix();
			glTranslatef(center(0),center(1),center(2));
			auxWireBox(AABB[i].XL,AABB[i].YL,AABB[i].ZL);
			glPopMatrix();

			glDepthMask(GL_FALSE);
			glEnable(GL_BLEND);
			glColor4f(0.91f,0.82f,0.38f,0.2f);
			glPushMatrix();
			glTranslatef(center(0),center(1),center(2));
			auxSolidBox(AABB[i].XL,AABB[i].YL,AABB[i].ZL);
			glPopMatrix();
			glDepthMask(GL_TRUE);
			glDisable(GL_BLEND);
		}

		if(cur_boxID>=0)
		{
			Vec3 center=(AABB[cur_boxID].top_pos+AABB[cur_boxID].bot_pos)/2.0;
			glDisable(GL_LIGHTING);
			glLineWidth(3.0f);
			glBegin(GL_LINES);
			Vec3 dir=Vec3(1.0,0.0,0.0);
			glColor3f(1.0f,0.0f,0.0f);
			glVertex3f(center(0),center(1),center(2));
			glVertex3f(center(0)+dir(0)*axis_length,center(1)+dir(1)*axis_length,center(2)+dir(2)*axis_length);

			dir=Vec3(0.0,1.0,0.0);
			glColor3f(0.0f,1.0f,0.0f);
			glVertex3f(center(0),center(1),center(2));
			glVertex3f(center(0)+dir(0)*axis_length,center(1)+dir(1)*axis_length,center(2)+dir(2)*axis_length);

			dir=Vec3(0.0,0.0,1.0);
			glColor3f(0.0f,0.0f,1.0f);
			glVertex3f(center(0),center(1),center(2));
			glVertex3f(center(0)+dir(0)*axis_length,center(1)+dir(1)*axis_length,center(2)+dir(2)*axis_length);
			glEnd();
		}
	}
	else
	{
		if(cur_boxID<0)
			return;
		int i=cur_boxID;
		glDepthMask(GL_FALSE);
		glDisable(GL_LIGHTING);
		//setMaterial(yellow_material);
		glEnable(GL_BLEND);
		glColor4f(1.0f,0.0f,0.0f,0.4f);
		Vec3 center=(AABB[i].top_pos+AABB[i].bot_pos)*0.5;
		glPushMatrix();
		glTranslatef(center(0),center(1),center(2));

		glutSolidSphere(bp_sphere_radius*4.0,10,10);
		glPopMatrix();
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);

		/*int i=cur_boxID;
		glDisable(GL_LIGHTING);

		Vec3 center=(AABB[i].top_pos+AABB[i].bot_pos)*0.5;
		glColor3f(0.3725f,0.5137f,0.6549f);
		glPushMatrix();
		glTranslatef(center(0),center(1),center(2));
		auxWireBox(AABB[i].XL,AABB[i].YL,AABB[i].ZL);
		glPopMatrix();

		glDepthMask(GL_FALSE);
		glEnable(GL_BLEND);
		glColor4f(0.91f,0.82f,0.38f,0.2f);
		glPushMatrix();
		glTranslatef(center(0),center(1),center(2));
		auxSolidBox(AABB[i].XL,AABB[i].YL,AABB[i].ZL);
		glPopMatrix();
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);*/
	}
}

void MeshViewer::render_boxpair()
{
	if(!box_pair.size())
		return;
	if(!box_show)
		return;
	glEnable(GL_LIGHTING);
	setMaterial(yellow_material);
	for(size_t i=0;i<box_pair.size();i++)
	{
		Vec3 center=box_pair[i].center;
		glPushMatrix();
		glTranslatef(center(0),center(1),center(2));
	
		glutSolidSphere(bp_sphere_radius,10,10);
		glPopMatrix();
	}

	glDisable(GL_LIGHTING);
	glColor3f(1.0f,0.0f,0.0f);
	for(size_t i=0;i<box_pair.size();i++)
	{
		Vec3 center=box_pair[i].center;
		for(size_t j=0;j<box_pair[i].box_id_list.size();j++)
		{
			Vec3 dir=box_pair[i].trans_dir[j];
			glBegin(GL_LINES);
			glVertex3f(center(0),center(1),center(2));
			glVertex3f(center(0)+dir(0)*axis_length,center(1)+dir(1)*axis_length,center(2)+dir(2)*axis_length);
			glEnd();
		}
	}
}

void MeshViewer::renderGraphNode()
{
	if(!m_graph.size())
		return;
	if(m_graph[0].size()&&graph_show)
	{
		glDisable(GL_LIGHTING);
		glPointSize(5.0f);
		glColor3f(1.0f,0.0f,0.0f);
		glBegin(GL_POINTS);
		for(size_t pid=0;pid<m_graph.size();pid++)
		{
			for(size_t i=0;i<m_graph[pid].size();i++)
				glVertex3f(m_graph[pid][i](0),m_graph[pid][i](1),m_graph[pid][i](2));
		}
		glEnd();
	}

	if(curNodeId>=0)
	{
		glDisable(GL_LIGHTING);
		glPointSize(5.0f);
		glColor3f(0.0f,0.0f,1.0f);
		Vec3 vert=m_graph[curNodeProxy][curNodeId];
		glBegin(GL_POINTS);
		glVertex3f(vert(0),vert(1),vert(2));
		glEnd();

		glColor3f(0.0f,1.0f,0.0f);
		auto& influ_nodes=(*m_GraphEdge)[curNodeProxy][curNodeId];
		for(auto t=influ_nodes.begin();t!=influ_nodes.end();t++)
		{
			Vec3 inf_vert=m_graph[curNodeProxy][*t];
			glBegin(GL_POINTS);
			glVertex3f(inf_vert(0),inf_vert(1),inf_vert(2));
			glEnd();
		}
	}
	/*
	if(deformnode_show&&m_deformNodes&&(*m_deformNodes).size()>0)
	{
		glDisable(GL_LIGHTING);
		glPointSize(5.0f);
		glColor3f(1.0f,0.0f,0.0f);
		Vec3 vert=deform_graph[testId];
		glBegin(GL_POINTS);
		glVertex3f(vert(0),vert(1),vert(2));
		glEnd();

		glColor3f(0.0f,0.0f,1.0f);
		for(size_t i=0;i<(*m_deformNodes)[testId].size();i++)
		{
			int vid=(*m_deformNodes)[testId][i];
			glBegin(GL_POINTS);
			glVertex3f(m_triMesh->vertices[vid][0],m_triMesh->vertices[vid][1],m_triMesh->vertices[vid][2]);
			glEnd();
		}
	}*/
}

void MeshViewer::setCSCMesh(std::vector<TriMesh*>& tri_proxy_mesh)
{
	m_csc_mesh.resize(m_proxy_mesh.size());
	for(int i=0;i<(int)m_csc_mesh.size();i++)
	{
		m_csc_mesh[i]=new CSCMesh;
		if(!m_csc_mesh[i])
		{
			std::cout<<"Can't construct the CSC Mesh"<<std::endl;
			exit(0);
		}
		m_csc_mesh[i]->setTriMesh(tri_proxy_mesh[i]);
	}
}

void MeshViewer::resetModel()
{
	load_success=false;
	for(int i=0;i<m_init_vertices.size();i++)
		m_proxy_mesh[i]->vertices.assign(m_init_vertices[i].begin(),m_init_vertices[i].end());

	m_graph.assign(m_init_graph.begin(),m_init_graph.end());
	AABB.clear();
	box_pair.clear();
	deformer.clearBoxes();
	cur_boxID=-1;
	curNodeId=-1;
	load_success=true;
}
void MeshViewer::write_mesh()
{
	if(!m_csc_mesh.size())
		return;
	std::string filename;
	for(int i=0;i<m_csc_mesh.size();i++)
	{
		std::stringstream ss;
		ss<<"mesh";
		ss<<i;
		ss<<".obj";
		ss>>filename;
		m_csc_mesh[i]->write_file(filename.c_str());
	}
}

void MeshViewer::outputGraphInfo(const std::string& filename)const
{
	if(!m_proxy_mesh.size())
		return;
	if(!m_init_graph.size()||!m_GraphEdge||!m_deformVerts)
		return;

	FILE* fp=fopen(filename.c_str(),"wb");
	int nMesh=m_proxy_mesh.size();
	fwrite(&nMesh,sizeof(int),1,fp);

	for(int i=0;i<nMesh;i++)
	{
		//save graph nodes
		const std::vector<Vec3>& graph_nodes= m_init_graph[i];
		int nNodes=graph_nodes.size();
		fwrite(&nNodes,sizeof(int),1,fp);
		for(int j=0;j<nNodes;j++)
			fwrite(&graph_nodes[j],sizeof(double),3,fp);

		//save deform verts info
		const std::vector<Deformer::DeformVerts>& dv_list=(*m_deformVerts)[i];
		int nVerts=dv_list.size();
		fwrite(&nVerts,sizeof(int),1,fp);
		for(int j=0;j<nVerts;j++)
		{
			const Deformer::DeformVerts& dv=dv_list[j];
			int nInflu=dv.nodeIds.size();
			fwrite(&nInflu,sizeof(int),1,fp);
			for(int k=0;k<nInflu;k++)
			{
				fwrite(&dv.nodeIds[k],sizeof(int),1,fp);
				fwrite(&dv.nodeWeights[k],sizeof(double),1,fp);
			}
		}

		//save graph edges info
		const VecSet(int)& ge=(*m_GraphEdge)[i];
		nNodes=ge.size();
		fwrite(&nNodes,sizeof(nNodes),1,fp);
		
		for(int j=0;j<nNodes;j++)
		{
			const std::set<int>& node_link=ge[j];
			int nLink=node_link.size();
			fwrite(&nLink,sizeof(int),1,fp);
			for(auto t=node_link.begin();t!=node_link.end();t++)
			{
				int node_id=*t;
				fwrite(&node_id,sizeof(int),1,fp);
			}
		}
		std::cout<<"save mesh "<<i<<std::endl;
	}
	std::cout<<"Graph information has been output successfully!"<<std::endl;
	fclose(fp);
}

void MeshViewer::inputGraphInfo(const std::string&filename)
{
	if(!m_proxy_mesh.size())
		return;
	m_graph.clear();
	m_init_graph.clear();
	FILE* fp=fopen(filename.c_str(),"rb");
	int nMesh;
	fread(&nMesh,sizeof(int),1,fp);
	if(nMesh!=m_proxy_mesh.size())
	{
		std::cout<<"the graph file is worng!"<<std::endl;
		return;
	}
	m_graph.resize(nMesh);

	VecList(Deformer::DeformVerts) temp_deform_Verts;
	std::vector<VecSet(int)> tmp_graph_edge;
	temp_deform_Verts.resize(nMesh);
	tmp_graph_edge.resize(nMesh);
	for(int i=0;i<nMesh;i++)
	{
		std::vector<Vec3>& graph_nodes= m_graph[i];
		int nNodes;
		fread(&nNodes,sizeof(int),1,fp);
		for(int j=0;j<nNodes;j++)
		{
			Vec3 vert;
			fread(&vert,sizeof(double),3,fp);
			graph_nodes.push_back(vert);
		}

		std::vector<Deformer::DeformVerts>& dv_list=temp_deform_Verts[i];
		int nVerts;
		fread(&nVerts,sizeof(int),1,fp);
		dv_list.resize(nVerts);
		for(int j=0;j<nVerts;j++)
		{
			Deformer::DeformVerts& dv=dv_list[j];
			int nInflu;
			fread(&nInflu,sizeof(int),1,fp);
			dv.nodeIds.resize(nInflu);
			dv.nodeWeights.resize(nInflu);
			for(int k=0;k<nInflu;k++)
			{
				fread(&dv.nodeIds[k],sizeof(int),1,fp);
				fread(&dv.nodeWeights[k],sizeof(double),1,fp);
			}
		}

		VecSet(int)& ge=tmp_graph_edge[i];
		fread(&nNodes,sizeof(nNodes),1,fp);
		ge.resize(nNodes);
		for(int j=0;j<nNodes;j++)
		{
			std::set<int>& node_link=ge[j];
			int nLink;
			fread(&nLink,sizeof(int),1,fp);
			for(int k=0;k<nLink;k++)
			{
				int node_id;
				fread(&node_id,sizeof(int),1,fp);
				node_link.insert(node_id);
			}
		}
		std::cout<<"load mesh "<<i<<std::endl;
	}

	deformer.setGraphEdge(tmp_graph_edge);
	deformer.setDeformVert(temp_deform_Verts);
	m_init_graph.assign(m_graph.begin(),m_graph.end());
	deformer.setModel(this);
	deformer.initDeformation(true);
	std::cout<<"import graph nodes succesfully!"<<std::endl;
}

void MeshViewer::outputBoxes(const std::string&filename)const
{
	FILE*fp=fopen(filename.c_str(),"wb");
	int n=AABB.size();
	fwrite(&n,sizeof(int),1,fp);
	for(int i=0;i<(int)AABB.size();i++)
	{
		const BoundingBox& bb=AABB[i];
		fwrite(&bb.bot_pos,sizeof(double),3,fp);
		fwrite(&bb.top_pos,sizeof(double),3,fp);
		fwrite(&bb.init_pos,sizeof(double),3,fp);
		fwrite(&bb.bp_id,sizeof(int),1,fp);
		fwrite(&bb.proxy_id,sizeof(int),1,fp);
		fwrite(&bb.XL,sizeof(double),1,fp);
		fwrite(&bb.YL,sizeof(double),1,fp);
		fwrite(&bb.ZL,sizeof(double),1,fp);
		int vn=bb.vertex_list.size();
		fwrite(&vn,sizeof(int),1,fp);
		for(int j=0;j<(int)bb.vertex_list.size();j++)
			fwrite(&bb.vertex_list[j],sizeof(int),1,fp);
	}

	n=box_pair.size();
	fwrite(&n,sizeof(int),1,fp);
	for(int i=0;i<(int)box_pair.size();i++)
	{
		const BoxPair& bp=box_pair[i];
		fwrite(&bp.center,sizeof(double),3,fp);
		int bn=bp.box_id_list.size();
		fwrite(&bn,sizeof(int),1,fp);
		for(int j=0;j<bn;j++)
		{
			fwrite(&bp.box_id_list[j],sizeof(int),1,fp);
			fwrite(&bp.trans_dir[j],sizeof(double),3,fp);
		}
	}
	std::cout<<"output boxes info succesfully!"<<std::endl;
	fclose(fp);
}

void MeshViewer::inputBoxes(const std::string&filename)
{
	AABB.clear();
	box_pair.clear();
	FILE*fp=fopen(filename.c_str(),"rb");
	int n;
	fread(&n,sizeof(int),1,fp);

	for(int i=0;i<n;i++)
	{
		BoundingBox bb;
		fread(&bb.bot_pos,sizeof(double),3,fp);
		fread(&bb.top_pos,sizeof(double),3,fp);
		fread(&bb.init_pos,sizeof(double),3,fp);
		fread(&bb.bp_id,sizeof(int),1,fp);
		fread(&bb.proxy_id,sizeof(int),1,fp);
		fread(&bb.XL,sizeof(double),1,fp);
		fread(&bb.YL,sizeof(double),1,fp);
		fread(&bb.ZL,sizeof(double),1,fp);

		int vn;
		fread(&vn,sizeof(int),1,fp);
		bb.vertex_list.resize(vn);
		for(int j=0;j<vn;j++)
			fread(&bb.vertex_list[j],sizeof(int),1,fp);
		AABB.push_back(bb);
	}

	fread(&n,sizeof(int),1,fp);
	for(int i=0;i<n;i++)
	{
		BoxPair bp;
		fread(&bp.center,sizeof(double),3,fp);
		int bn;
		fread(&bn,sizeof(int),1,fp);
		bp.box_id_list.resize(bn);
		bp.trans_dir.resize(bn);

		for(int j=0;j<bn;j++)
		{
			fread(&bp.box_id_list[j],sizeof(int),1,fp);
			fread(&bp.trans_dir[j],sizeof(double),3,fp);
		}
		box_pair.push_back(bp);
	}
	std::cout<<"input boxes info succesfully!"<<std::endl;
	fclose(fp);
}

void MeshViewer::deform_noPair(int posx,int posy,int cur_box_id)
{
	cur_pt.setX(posx);
	cur_pt.setY(posy);

	Vec2 mouse_dir(cur_pt.x()-start_pt.x(),cur_pt.y()-start_pt.y());

	Vec3 start=(AABB[cur_box_id].bot_pos+AABB[cur_box_id].top_pos)/2.0;
	Vec3 end=start+m_axis_dir[m_axis]*axis_length;
	Vec2 dir2D=calAixs2D(start,end);

	//std::cout<<m_axis_dir[m_axis]<<std::endl;
	double delta=mouse_dir.dot(dir2D)*AABB_trans;
	Vec3 trans=m_axis_dir[m_axis]*delta;
	AABB[cur_box_id].bot_pos+=trans;
	AABB[cur_box_id].top_pos+=trans;
	start_pt=cur_pt;

	deformer.real_time_deform(cur_boxID);
}
void MeshViewer::readRenderInfo(const std::string& filename)
{
	FILE* fp=fopen(filename.c_str(),"rb");
	fread(proj_matrix,sizeof(double),16,fp);
	fread(model_matrix,sizeof(double),16,fp);
	fclose(fp);
}

void MeshViewer::deform_Pair(int posx,int posy,int cur_box_id,int bp_id)
{
	cur_pt.setX(posx);
	cur_pt.setY(posy);
	int order=0;
	Vec3 direction;
	for(int i=0;i<(int)box_pair[bp_id].box_id_list.size();i++)
	{
		if(box_pair[bp_id].box_id_list[i]==cur_box_id)
		{
			order=i;
			direction=box_pair[bp_id].trans_dir[i];
			break;
		}
	}

	Vec2 mouse_dir(cur_pt.x()-start_pt.x(),cur_pt.y()-start_pt.y());
	Vec3 start=(AABB[cur_box_id].bot_pos+AABB[cur_box_id].top_pos)*0.5;
	Vec3 end=start+ direction*axis_length;
	Vec2 dir2D=calAixs2D(start,end);
	double delta=mouse_dir.dot(dir2D)*AABB_trans;

	for(int i=0;i<box_pair[bp_id].box_id_list.size();i++)
	{
		int bid=box_pair[bp_id].box_id_list[i];
		Vec3 trans=box_pair[bp_id].trans_dir[i]*delta;
		AABB[bid].bot_pos+=trans;
		AABB[bid].top_pos+=trans;
		deformer.real_time_deform(bid);
	}
	start_pt=cur_pt;
}