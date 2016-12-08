//#include "StdAfx.h"
#include "SCMesh.h"
#include "GL/glut.h"
#include <fstream>
#include "../TriMesh/XForm.h"
//#include "../MyUtil/MyUtil.h"
#include "../TriMesh/apparentridge.h"
#include "../MyUtil/MyMacro.h"


#ifndef M_SQRT1_2
#define M_SQRT1_2 7.0710678118654752440E-1
#endif

#define CLAMP(x, min, max) ((x)<(min) ? (min) :((x)>(max) ? (max) : (x)))
CSCMesh::CSCMesh()
{
	m_pTriMesh = NULL;
	m_pOriTriMesh = NULL;
	m_meshColor = Color::white();
	m_edgeColor = Color::blue();
	m_lineColor = Color::black();
	m_feature_size = 0.0;
	m_sug_thresh = 0.01;
	m_rv_thresh = 0.1;
	m_test_c = m_test_sc = m_test_rv = true;
	m_bUseHermite = false;
	m_crease_high_thresh = M_PI/3.0;
	m_crease_low_thresh = M_PI/6.0;
	m_featureLine_width = 2.0;
}

CSCMesh::~CSCMesh()
{
	SAFE_DELETE(m_pTriMesh);
	SAFE_DELETE(m_pOriTriMesh);
	destroy_edge_info();
}

void CSCMesh::destroy()
{

}

void CSCMesh::destroyTriMesh()
{
	SAFE_DELETE(m_pTriMesh);
}

void CSCMesh::destroy_edge_info()
{
	int edgeNum = (int)m_pEdges.size();
	for (int i=0;i<edgeNum;i++)
	{
		delete m_pEdges[i];
	}
	m_pEdges.clear();
	int vertexNum = (int)m_pVertices.size();
	for (int i=0;i<vertexNum;i++)
	{
		delete m_pVertices[i];
	}
	m_pVertices.clear();
}

void CSCMesh::build_edge_info()
{
	//printf("Build edge info...\n");
	destroy_edge_info();
	m_pTriMesh->need_across_edge();
	std::vector<point> &v = m_pTriMesh->vertices;
	std::vector<TriMesh::Face> &nei_faces = m_pTriMesh->across_edge;
	int vNum = (int)v.size();
	m_pVertices.resize(vNum);
	for (int i=0;i<vNum;i++)
	{
		m_pVertices[i] = new CVertex();
	}
	int faceNum = (int)m_pTriMesh->faces.size();
	std::vector<bool> already_visited;
	already_visited.assign(faceNum,false);
	
	int nei_f_idx = 0;
	int vIdx=0,eIdx=0;;
	int edge_v1=0, edge_v2=0;
	CEdge *temp = NULL;
	for (int i=0;i<faceNum;i++)
	{
		for (int j=0;j<3;j++)
		{
			/**********added by stj: add face to the vertex***********/
			m_pVertices[m_pTriMesh->faces[i][j]]->add_face(i);
			/*********************************************************/

			nei_f_idx = nei_faces[i][j];	
			vIdx = m_pTriMesh->faces[i][j];	//the across edge opposite this v
			if (nei_f_idx!=-1)	//not boundary
			{
				if (!already_visited[nei_f_idx])
				{
					edge_v1 = m_pTriMesh->faces[i][(j+1)%3];
					edge_v2 = m_pTriMesh->faces[i][(j+2)%3];
					temp = new CEdge(edge_v1,edge_v2,i,nei_f_idx);
					m_pEdges.push_back(temp);
					eIdx = (int)m_pEdges.size()-1;
					m_pVertices[edge_v1]->add_edge(eIdx);
					m_pVertices[edge_v2]->add_edge(eIdx);
				}
			}else	//boundary
			{
				edge_v1 = m_pTriMesh->faces[i][(j+1)%3];
				edge_v2 = m_pTriMesh->faces[i][(j+2)%3];
				temp = new CEdge(edge_v1,edge_v2,i,-1);
				m_pEdges.push_back(temp);
				eIdx = (int)m_pEdges.size()-1;
				m_pVertices[edge_v1]->add_edge(eIdx);
				m_pVertices[edge_v2]->add_edge(eIdx);
			}
		}
		already_visited[i] = true;
	}
	//printf("done...\n");
}


void CSCMesh::initialize()
{
	if (!m_pTriMesh)
		return;
	m_pTriMesh->need_tstrips();
	m_pTriMesh->need_bsphere();
	m_pTriMesh->need_normals();
	m_pTriMesh->need_curvatures();
	m_pTriMesh->need_dcurv();
	face_normal();
	build_edge_info();
	detect_crease_edge();
	compute_feature_size();
}

void CSCMesh::update()
{
	m_pTriMesh->bsphere.valid = false;
	m_pTriMesh->need_bsphere();
	m_pTriMesh->normals.resize(0);
	m_pTriMesh->need_normals();
	m_pTriMesh->curv1.resize(0);
	m_pTriMesh->need_curvatures();
	m_pTriMesh->dcurv.resize(0);
	m_pTriMesh->need_dcurv();
	face_normal();
	build_edge_info();
	detect_crease_edge();
	compute_feature_size();
}

bool CSCMesh::read_file(const char *filename)
{
	/*SAFE_DELETE(m_pTriMesh);
	m_pTriMesh = TriMesh::read(filename);
	initialize();
	if (m_pTriMesh)
	{
		std::string path(filename);
		std::string dir;
		std::string::size_type  p = path.rfind("\\");
		dir = path.substr(0, p);
		SetCurrentDirectory(dir.c_str());
	}

	//added deform mesh
	if (m_pTriMesh)
	{
		SAFE_DELETE(m_pOriTriMesh);
		m_pOriTriMesh = MyUtil::clone_mesh(m_pTriMesh);
	}

	return (m_pTriMesh != NULL);
	*/
	return false;
}

bool CSCMesh::write_file(const char *filename)
{
	if (!m_pTriMesh)
	{
		printf("Write TriMesh File Error: no TriMesh!\n");
		return false;
	}
	return m_pTriMesh->write(filename);
}

static vector<float> ndotv, kr;
static vector<float> sctest_num, sctest_den, shtest_num;
static vector<float> q1, Dt1q1;
static vector<vec2> t1;
void CSCMesh::render(const RENDER_PARA &para)
{
	if (!m_pTriMesh)
		return;

	if(para._shader_mode == 0) // no shader
	{
		compute_perview(ndotv, kr, sctest_num, sctest_den, shtest_num,
			q1, t1, Dt1q1, para);


		glPushAttrib(GL_ALL_ATTRIB_BITS );


		glDisable(GL_LIGHTING);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, m_material.ambient_);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, m_material.diffuse_);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,m_material.specular_);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS,m_material.shininess_);


		glEnable(GL_BLEND);
		render_base_mesh(para);
		glDisable(GL_BLEND);

		//draw lines
		glDepthMask(GL_FALSE); // Do not remove me, else get dotted lines
		render_wire_frame(para);
		update_view_pos();
		render_feature_lines(para);
		glDepthMask(GL_TRUE);
		glPopAttrib();
	}
	else // shadermode
	{
		//if(para._shader_draw_faces)
			render_base_mesh(para);
		//if(para._shader_draw_lines)
			render_feature_lines(para);
	}
}

void CSCMesh::compute_feature_size()
{
	int nv = (int)m_pTriMesh->curv1.size();
	int nsamp = std::min(nv, 500);

	vector<float> samples;
	samples.reserve(nsamp * 2);

	for (int i = 0; i < nsamp; i++) {
		// Quick 'n dirty portable random number generator
		static unsigned randq = 0;
		randq = unsigned(1664525) * randq + unsigned(1013904223);

		int ind = randq % nv;
		samples.push_back(fabs(m_pTriMesh->curv1[ind]));
		samples.push_back(fabs(m_pTriMesh->curv2[ind]));
	}
	const float frac = 0.1f;
	const float mult = 0.01f;
	m_pTriMesh->need_bsphere();
	float max_feature_size = 0.05f * m_pTriMesh->bsphere.r;
	int which = int(frac * samples.size());
	nth_element(samples.begin(), samples.begin() + which, samples.end());
	m_feature_size = std::min(mult / samples[which], max_feature_size);
}


// Color the mesh by curvatures
void CSCMesh::compute_curv_colors()
{
	float cscale = sqr(8.0f * m_feature_size);
	int nv = (int)m_pTriMesh->vertices.size();
	m_curv_colors.resize(nv);
	for (int i = 0; i < nv; i++) {
		float H = 0.5f * (m_pTriMesh->curv1[i] + m_pTriMesh->curv2[i]);
		float K = m_pTriMesh->curv1[i] * m_pTriMesh->curv2[i];
		float h = 4.0f / 3.0f * fabs(atan2(H*H-K,H*H*sgn(H)));
		float s = M_2_PI * atan((2.0f*H*H-K)*cscale);
		m_curv_colors[i] = Color::hsv(h,s,1.0);
	}
}

void CSCMesh::compute_gcurv_colors()
{
	float cscale = 10.0f * m_feature_size;
	int nv = (int)m_pTriMesh->vertices.size();
	m_gcurv_colors.resize(nv);
	for (int i = 0; i < nv; i++) {
		float H = 0.5f * (m_pTriMesh->curv1[i] + m_pTriMesh->curv2[i]);
		float c = (atan(H*cscale) + M_PI_2) / M_PI;
		c = sqrt(c);
		int C = int(std::min(std::max(256.0 * c, 0.0), 255.99));
		m_gcurv_colors[i] = Color(C,C,C);
	}
}

void CSCMesh::draw_tstrips()
{
	const int *t = &m_pTriMesh->tstrips[0];
	const int *end = t + m_pTriMesh->tstrips.size();
	while (likely(t < end)) {
		int striplen = *t++;
		glDrawElements(GL_TRIANGLE_STRIP, striplen, GL_UNSIGNED_INT, t);
		t += striplen;
	}
}

void CSCMesh::render_wire_frame(const RENDER_PARA &para)
{
	if (!para._draw_wireframe)
		return;
	begin_wireframe();
	int nF = (int)m_pTriMesh->faces.size();
	int vID;
	for(int i = 0; i < nF; i++)
	{
		glBegin(GL_TRIANGLES);
		vID = m_pTriMesh->faces[i][0];
		glVertex3f(m_pTriMesh->vertices[vID][0], m_pTriMesh->vertices[vID][1], m_pTriMesh->vertices[vID][2]);

		vID = m_pTriMesh->faces[i][1];
		glVertex3f(m_pTriMesh->vertices[vID][0], m_pTriMesh->vertices[vID][1], m_pTriMesh->vertices[vID][2]);

		vID = m_pTriMesh->faces[i][2];
		glVertex3f(m_pTriMesh->vertices[vID][0], m_pTriMesh->vertices[vID][1], m_pTriMesh->vertices[vID][2]);
		glEnd();
	}
	end_wireframe();
}


void CSCMesh::begin_wireframe()
{
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(m_edgeColor[0], m_edgeColor[1], m_edgeColor[2],0.5);
	glLineWidth(0.5f);
	glDepthRange(0.0, 0.999999);
}

void CSCMesh::end_wireframe()
{
	glLineWidth(1.0f);
	glDepthRange(0.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_LIGHTING);
}

void CSCMesh::render_crease_edge(const RENDER_PARA &para)
{
	if (m_pEdges.size()==0 || m_creaseEdges.size()==0)
		return;

	glLineWidth(m_featureLine_width);
	glColor3f(para._line_color[0],para._line_color[1],para._line_color[2]);
	int creaseEdgeNum = (int)m_creaseEdges.size();
	int v1=0,v2=0;
	int edgeIdx = 0;
	glBegin(GL_LINES);
	for (int i=0;i<creaseEdgeNum;i++)
	{
		edgeIdx = m_creaseEdges[i];
		v1 = m_pEdges[edgeIdx]->m_vertex[0];
		v2 = m_pEdges[edgeIdx]->m_vertex[1];
		const point &p1 = m_pTriMesh->vertices[v1];
		const point &p2 = m_pTriMesh->vertices[v2];
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
	}
	glEnd();
}


// Draw the basic mesh, which we'll overlay with lines
void CSCMesh::render_base_mesh(const RENDER_PARA &para)
{
	TriMesh *pMesh =m_pTriMesh;
	if (!pMesh)
	{
		return;
	}
	if(para._shader_mode == 0) // no shader
	{
		// Draw the mesh, possibly with color and/or lighting

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glDepthFunc(GL_LESS);
		glEnable(GL_DEPTH_TEST);
		glPolygonOffset(para._glPlyOffset_factor, para._glPlyOffset_units);
		glEnable(GL_POLYGON_OFFSET_FILL);
		//glEnable(GL_CULL_FACE);
		glDisable(GL_CULL_FACE);
		if (!para._draw_npr_with_light)
		{
			glDisable(GL_LIGHTING);
			glColor3f(m_meshColor[0],m_meshColor[1],m_meshColor[2]);
		}

		glEnable(GL_COLOR_MATERIAL);
		double r=0,g=0,b=0;
		int nF = (int)pMesh->faces.size();
		int vID;
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < nF; i++)
		{
			vID = pMesh->faces[i][0];
			if (para._draw_npr_with_light)
				glNormal3f(pMesh->normals[vID][0], pMesh->normals[vID][1], pMesh->normals[vID][2]);
			glVertex3f(pMesh->vertices[vID][0], pMesh->vertices[vID][1], pMesh->vertices[vID][2]);

			vID = pMesh->faces[i][1];
			if (para._draw_npr_with_light)
				glNormal3f(pMesh->normals[vID][0], pMesh->normals[vID][1], pMesh->normals[vID][2]);
			glVertex3f(pMesh->vertices[vID][0], pMesh->vertices[vID][1], pMesh->vertices[vID][2]);

			vID = pMesh->faces[i][2];
			if (para._draw_npr_with_light)
				glNormal3f(pMesh->normals[vID][0], pMesh->normals[vID][1], pMesh->normals[vID][2]);
			glVertex3f(pMesh->vertices[vID][0], pMesh->vertices[vID][1], pMesh->vertices[vID][2]);
		}
		glEnd();

		glDisable(GL_COLOR_MATERIAL);

		if (!para._draw_npr_with_light)
		{
			glEnable(GL_LIGHTING);
		}
		glDisable(GL_CULL_FACE);
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDepthFunc(GL_LEQUAL);
		glPopAttrib();
	}// end if no shader
	else
	{
		glColor4f(1.0f,1.0f,1.0f,1.0f);
		int nF = (int)pMesh->faces.size();
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < nF; i++)
		{
#if 1
			vec v[3];
			v[0] = pMesh->vertices[pMesh->faces[i][0]];
			v[1] = pMesh->vertices[pMesh->faces[i][1]];
			v[2] = pMesh->vertices[pMesh->faces[i][2]];

			VEC3F vv[3]={VEC3F(v[0][0],v[0][1],v[0][2]), 
				VEC3F(v[1][0],v[1][1],v[1][2]), 
				VEC3F(v[2][0],v[2][1],v[2][2])};
	
			VEC3F nm = (vv[1]-vv[0]).Cross(vv[2]-vv[0]);
			VEC3D nm2(nm[0],nm[1],nm[2]);
			double s=nm2.Length();
			for(int i=0;i<3;i++)
				nm2[i]/=s;
			
			glNormal3dv(&nm2[0]);
			glVertex3fv(&vv[0][0]);
			glVertex3fv(&vv[1][0]);
			glVertex3fv(&vv[2][0]);
#endif
		}
		glEnd();
	}// end else shader mode
}


void CSCMesh::update_view_pos()
{
	float mv_data[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mv_data);	//row-major
	XForm<float> xf(mv_data[0], mv_data[1], mv_data[2],  mv_data[3],	//column-major
					 mv_data[4], mv_data[5], mv_data[6],  mv_data[7],
					 mv_data[8], mv_data[9], mv_data[10], mv_data[11],
					 mv_data[12], mv_data[13], mv_data[14], mv_data[15]);
	m_viewpos = inv(xf) * point(0,0,0);
}

// Compute per-vertex n dot l, n dot v, radial curvature, and
// derivative of curvature for the current view
void CSCMesh::compute_perview(vector<float> &ndotv, vector<float> &kr,
					 vector<float> &sctest_num, vector<float> &sctest_den,
					 vector<float> &shtest_num, vector<float> &q1,
					 vector<vec2> &t1, vector<float> &Dt1q1,const RENDER_PARA &para,
					 bool extra_sin2theta)
{
	if (0)//(para._draw_apparent)
	{
		m_pTriMesh->need_adjacentfaces();
	}

	int nv = (int)m_pTriMesh->vertices.size();
	double scthresh = m_sug_thresh / sqr(m_feature_size);
	bool need_DwKr = false;//(para.m_bRenderSC);

	ndotv.resize(nv);
	kr.resize(nv);
	if (0)//(para._draw_apparent) 
	{
		q1.resize(nv);
		t1.resize(nv);
		Dt1q1.resize(nv);
	}
	if (need_DwKr) {
		sctest_num.resize(nv);
		sctest_den.resize(nv);
		if (0)//(para._draw_sh)
			shtest_num.resize(nv);
	}


	// Compute quantities at each vertex
#pragma omp parallel for
	for (int i = 0; i < nv; i++) {
		// Compute n DOT v
		vec viewdir = m_viewpos - m_pTriMesh->vertices[i];
		float rlv = 1.0f / len(viewdir);
		viewdir *= rlv;
		ndotv[i] = viewdir DOT m_pTriMesh->normals[i];
		float u = viewdir DOT m_pTriMesh->pdir1[i], u2 = u*u;
		float v = viewdir DOT m_pTriMesh->pdir2[i], v2 = v*v;

		// Note:  this is actually Kr * sin^2 theta
		kr[i] = m_pTriMesh->curv1[i] * u2 + m_pTriMesh->curv2[i] * v2;

		if (0)//(para._draw_apparent) 
		{
			float csc2theta = 1.0f / (u2 + v2);
			compute_viewdep_curv(m_pTriMesh, i, ndotv[i],
				u2*csc2theta, u*v*csc2theta, v2*csc2theta,
				q1[i], t1[i]);
		}
		if (!need_DwKr)
			continue;

		// Use DwKr * sin(theta) / cos(theta) for cutoff test
		sctest_num[i] = u2 * (     u*m_pTriMesh->dcurv[i][0] +
			3.0f*v*m_pTriMesh->dcurv[i][1]) +
			v2 * (3.0f*u*m_pTriMesh->dcurv[i][2] +
			v*m_pTriMesh->dcurv[i][3]);
		float csc2theta = 1.0f / (u2 + v2);
		sctest_num[i] *= csc2theta;
		float tr = (m_pTriMesh->curv2[i] - m_pTriMesh->curv1[i]) *
			u * v * csc2theta;
		sctest_num[i] -= 2.0f * ndotv[i] * sqr(tr);
		if (extra_sin2theta)
			sctest_num[i] *= u2 + v2;

		sctest_den[i] = ndotv[i];

		sctest_num[i] -= scthresh * sctest_den[i];
	}
	if (0)//(para._draw_apparent) 
	{
#pragma omp parallel for
		for (int i = 0; i < nv; i++)
			compute_Dt1q1(m_pTriMesh, i, ndotv[i], q1, t1, Dt1q1[i]);
	}
}


// Draw principal highlights on a face
void CSCMesh::draw_face_ph(int v0, int v1, int v2, bool do_ridge,
	const vector<float> &ndotv, bool do_bfcull,
	bool do_test, float thresh)
{
	TriMesh *themesh = m_pTriMesh;
	// Backface culling
	if (likely(do_bfcull &&
		ndotv[v0] <= 0.0f && ndotv[v1] <= 0.0f && ndotv[v2] <= 0.0f))
		return;

	// Orient principal directions based on the largest principal curvature
	float k0 = themesh->curv1[v0];
	float k1 = themesh->curv1[v1];
	float k2 = themesh->curv1[v2];
	if (do_test && do_ridge && (std::min)((std::min)(k0,k1),k2) < 0.0f)
		return;
	if (do_test && !do_ridge && (std::max)((std::max)(k0,k1),k2) > 0.0f)
		return;

	vec d0 = themesh->pdir1[v0];
	vec d1 = themesh->pdir1[v1];
	vec d2 = themesh->pdir1[v2];
	float kmax = fabs(k0);
	// dref is the e1 vector with the largest |k1|
	vec dref = d0;
	if (fabs(k1) > kmax)
		kmax = fabs(k1), dref = d1;
	if (fabs(k2) > kmax)
		kmax = fabs(k2), dref = d2;

	// Flip all the e1 to agree with dref
	if ((d0 DOT dref) < 0.0f) d0 = -d0;
	if ((d1 DOT dref) < 0.0f) d1 = -d1;
	if ((d2 DOT dref) < 0.0f) d2 = -d2;

	// If directions have flipped (more than 45 degrees), then give up
	if ((d0 DOT dref) < M_SQRT1_2 ||
		(d1 DOT dref) < M_SQRT1_2 ||
		(d2 DOT dref) < M_SQRT1_2)
		return;

	// Compute view directions, dot products @ each vertex
	vec viewdir0 = m_viewpos - themesh->vertices[v0];
	vec viewdir1 = m_viewpos - themesh->vertices[v1];
	vec viewdir2 = m_viewpos - themesh->vertices[v2];

	// Normalize these for cos(theta) later...
	normalize(viewdir0);
	normalize(viewdir1);
	normalize(viewdir2);

	// e1 DOT w sin(theta) 
	// -- which is zero when looking down e2
	float dot0 = viewdir0 DOT d0;
	float dot1 = viewdir1 DOT d1;
	float dot2 = viewdir2 DOT d2;

	// We have a "zero crossing" if the dot products along an edge
	// have opposite signs
	int z01 = (dot0*dot1 <= 0.0f);
	int z12 = (dot1*dot2 <= 0.0f);
	int z20 = (dot2*dot0 <= 0.0f);

	if (z01 + z12 + z20 < 2)
		return;

	// Draw line segment
	float test0 = (sqr(themesh->curv1[v0]) - sqr(themesh->curv2[v0])) *
		viewdir0 DOT themesh->normals[v0];
	float test1 = (sqr(themesh->curv1[v1]) - sqr(themesh->curv2[v1])) *
		viewdir0 DOT themesh->normals[v1];
	float test2 = (sqr(themesh->curv1[v2]) - sqr(themesh->curv2[v2])) *
		viewdir0 DOT themesh->normals[v2];

	if (!z01) {
		draw_segment_ridge(v1, v2, v0,
			dot1, dot2, dot0,
			test1, test2, test0,
			thresh, false);
	} else if (!z12) {
		draw_segment_ridge(v2, v0, v1,
			dot2, dot0, dot1,
			test2, test0, test1,
			thresh, false);
	} else if (!z20) {
		draw_segment_ridge(v0, v1, v2,
			dot0, dot1, dot2,
			test0, test1, test2,
			thresh, false);
	}
}

// Draw principal highlights
void CSCMesh::draw_mesh_ph(bool do_ridge, const vector<float> &ndotv, bool do_bfcull,
	bool do_test, float thresh)
{
	TriMesh *themesh = m_pTriMesh;
	const int *t = &themesh->tstrips[0];
	const int *stripend = t;
	const int *end = t + themesh->tstrips.size();

	// Walk through triangle strips
	while (1) {
		if (unlikely(t >= stripend)) {
			if (unlikely(t >= end))
				return;
			// New strip: each strip is stored as
			// length followed by indices
			stripend = t + 1 + *t;
			// Skip over length plus first two indices of
			// first face
			t += 3;
		}

		draw_face_ph(*(t-2), *(t-1), *t, do_ridge,
			ndotv, do_bfcull, do_test, thresh);
		t++;
	}
}


// Draw the boundaries on the mesh
void draw_boundaries(TriMesh *themesh, bool do_hidden)
{
	themesh->need_faces();
	themesh->need_across_edge();
	if (do_hidden) {
		glColor3f(0.6, 0.6, 0.6);
		glLineWidth(1.5);
	} else {
		glColor3f(0.05, 0.05, 0.05);
		glLineWidth(2.5);
	}
	glBegin(GL_LINES);
	for (int i = 0; i < (int)themesh->faces.size(); i++) {
		for (int j = 0; j < 3; j++) {
			if (themesh->across_edge[i][j] >= 0)
				continue;
			int v1 = themesh->faces[i][(j+1)%3];
			int v2 = themesh->faces[i][(j+2)%3];
			glVertex3fv(themesh->vertices[v1]);
			glVertex3fv(themesh->vertices[v2]);
		}
	}
	glEnd();
}

// Draw the feature lines
void CSCMesh::render_feature_lines(const RENDER_PARA &para)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	// These are static so the memory isn't reallocated on every frame
	//static vector<float> ndotv, kr;
	//static vector<float> sctest_num, sctest_den, shtest_num;
	//static vector<float> q1, Dt1q1;
	//static vector<vec2> t1;
	//compute_perview(ndotv, kr, sctest_num, sctest_den, shtest_num,
	//	q1, t1, Dt1q1, para);
	int nv = (int)m_pTriMesh->vertices.size();
	
	// Enable antialiased lines
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	//glEnable(GL_MULTISAMPLE);

	//glEnable(GL_DEPTH_TEST);

	// The main rendering pass
	// Apparent ridges
	if (0)//(para._draw_apparent) 
	{
		glLineWidth(2.5);
		glBegin(GL_LINES);
		//draw_mesh_app_ridges(m_pTriMesh,ndotv, q1, t1, Dt1q1, true,
		//	para._test_ar, para._ar_thresh / sqr(m_feature_size));
		draw_mesh_app_ridges(m_pTriMesh,ndotv, q1, t1, Dt1q1, true,
			true, 0.1 / sqr(m_feature_size));
		glEnd();
	}

	// Suggestive contours and contours
	if (0)//(para.m_bRenderSC) 
	{
		float fade = 1/*para.m_bFaded*/ ? 0.03f / sqr(m_feature_size) : 0.0f;
		glLineWidth(m_featureLine_width);
		glBegin(GL_LINES);
		draw_isolines(kr, sctest_num, sctest_den, ndotv,
			true, m_bUseHermite, true, fade);
		glEnd();
	}
	if (0)//(para.m_bRenderC) 
	{
		glLineWidth(m_featureLine_width);
		glBegin(GL_LINES);
		draw_isolines(ndotv, kr, vector<float>(), ndotv,
			false, false, true, 0.0f);
		glEnd();
	}

	// Ridges and valleys
	glColor3f(para._line_color[0],para._line_color[1],para._line_color[2]);
	if (0)//(para.m_bRenderRidge) 
	{
		glLineWidth(2);
		glBegin(GL_LINES);
		draw_mesh_ridges(true, ndotv, true, m_test_rv,
			m_rv_thresh / m_feature_size);
		glEnd();
	}
	if (0)//(para.m_bRenderValley) 
	{
		glLineWidth(2);
		glBegin(GL_LINES);
		draw_mesh_ridges(false, ndotv, true, m_test_rv,
			m_rv_thresh / m_feature_size);
		glEnd();
	}

	// Principal highlights
	if (0)//(para._draw_phridges || para._draw_phvalleys) 
	{
		glLineWidth(2);
		glBegin(GL_LINES);
		float thresh = 0.04/*para._ph_thresh*/ / sqr(m_feature_size);
		if (0)//(para._draw_phridges)
			draw_mesh_ph(true, ndotv, true, true/*para._test_ph*/, thresh);
		if (0)//(para._draw_phvalleys)
			draw_mesh_ph(false, ndotv, true, true/*para._test_ph*/, thresh);
		glEnd();
	}

	// Suggestive highlights
	if (0)//(para._draw_sh) 
	{
		//currcolor = vec(0.3,0.3,0.3);
		float fade = 1/*para.m_bFaded*/ ? 0.03f / sqr(m_feature_size) : 0.0f;
		glLineWidth(2.5);
		glBegin(GL_LINES);
		draw_isolines(kr, shtest_num, sctest_den, ndotv,
			true, m_bUseHermite, true/*para._test_sh*/, fade);
		glEnd();
	}

	
	render_crease_edge(para);


	//glDisable(GL_MULTISAMPLE);

	//render_silhouette(ndotv,para);
	//glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
	glPopAttrib();
}

// Draw exterior silhouette of the mesh: this just draws
// thick contours, which are partially hidden by the mesh.
// Note: this needs to happen *before* draw_base_mesh...
void CSCMesh::render_silhouette(const vector<float> &ndotv,const RENDER_PARA &para)
{
	//if (!para._draw_es)
	{
		return;
	}
	glDepthMask(GL_FALSE);
	glLineWidth(6);
	glBegin(GL_LINES);
	draw_isolines(ndotv, vector<float>(), vector<float>(), ndotv,
		false, false, false, 0.0f);
	glEnd();

	// Wide lines are gappy, so fill them in
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPointSize(6);
	glBegin(GL_POINTS);
	draw_isolines(ndotv, vector<float>(), vector<float>(), ndotv,
		false, false, false, 0.0f);
	glEnd();

	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
}

// Takes a scalar field and renders the zero crossings, but only where
// test_num/test_den is greater than 0.
void CSCMesh::draw_isolines(const vector<float> &val,
				   const vector<float> &test_num,
				   const vector<float> &test_den,
				   const vector<float> &ndotv,
				   bool do_bfcull, bool do_hermite,
				   bool do_test, float fade)
{
	const int *t = &m_pTriMesh->tstrips[0];
	const int *stripend = t;
	const int *end = t + m_pTriMesh->tstrips.size();

	// Walk through triangle strips
	while (1) {
		if (unlikely(t >= stripend)) {
			if (unlikely(t >= end))
				return;
			// New strip: each strip is stored as
			// length followed by indices
			stripend = t + 1 + *t;
			// Skip over length plus first two indices of
			// first face
			t += 3;
		}
		// Draw a line if, among the values in this triangle,
		// at least one is positive and one is negative
		const float &v0 = val[*t], &v1 = val[*(t-1)], &v2 = val[*(t-2)];
		if (unlikely((v0 > 0.0f || v1 > 0.0f || v2 > 0.0f) &&
			(v0 < 0.0f || v1 < 0.0f || v2 < 0.0f)))
			draw_face_isoline(*(t-2), *(t-1), *t,
			val, test_num, test_den, ndotv,
			do_bfcull, do_hermite, do_test, fade);
		t++;
	}
}

// Find a zero crossing using Hermite interpolation
float CSCMesh::find_zero_hermite(int v0, int v1, float val0, float val1,
						const vec &grad0, const vec &grad1)
{
	if (unlikely(val0 == val1))
		return 0.5f;

	// Find derivatives along edge (of interpolation parameter in [0,1]
	// which means that e01 doesn't get normalized)
	vec e01 = m_pTriMesh->vertices[v1] - m_pTriMesh->vertices[v0];
	float d0 = e01 DOT grad0, d1 = e01 DOT grad1;

	// This next line would reduce val to linear interpolation
	//d0 = d1 = (val1 - val0);

	// Use hermite interpolation:
	//   val(s) = h1(s)*val0 + h2(s)*val1 + h3(s)*d0 + h4(s)*d1
	// where
	//  h1(s) = 2*s^3 - 3*s^2 + 1
	//  h2(s) = 3*s^2 - 2*s^3
	//  h3(s) = s^3 - 2*s^2 + s
	//  h4(s) = s^3 - s^2
	//
	//  val(s)  = [2(val0-val1) +d0+d1]*s^3 +
	//            [3(val1-val0)-2d0-d1]*s^2 + d0*s + val0
	// where
	//
	//  val(0) = val0; val(1) = val1; val'(0) = d0; val'(1) = d1
	//

	// Coeffs of cubic a*s^3 + b*s^2 + c*s + d
	float a = 2 * (val0 - val1) + d0 + d1;
	float b = 3 * (val1 - val0) - 2 * d0 - d1;
	float c = d0, d = val0;

	// -- Find a root by bisection
	// (as Newton can wander out of desired interval)

	// Start with entire [0,1] interval
	float sl = 0.0f, sr = 1.0f, valsl = val0, valsr = val1;

	// Check if we're in a (somewhat uncommon) 3-root situation, and pick
	// the middle root if it happens (given we aren't drawing curvy lines,
	// seems the best approach..)
	//
	// Find extrema of derivative (a -> 3a; b -> 2b, c -> c),
	// and check if they're both in [0,1] and have different signs
	float disc = 4 * b - 12 * a * c;
	if (disc > 0 && a != 0) {
		disc = sqrt(disc);
		float r1 = (-2 * b + disc) / (6 * a);
		float r2 = (-2 * b - disc) / (6 * a);
		if (r1 >= 0 && r1 <= 1 && r2 >= 0 && r2 <= 1) {
			float vr1 = (((a * r1 + b) * r1 + c) * r1) + d;
			float vr2 = (((a * r2 + b) * r2 + c) * r2) + d;
			// When extrema have different signs inside an
			// interval with endpoints with different signs,
			// the middle root is in between the two extrema
			if (vr1 < 0.0f && vr2 >= 0.0f ||
				vr1 > 0.0f && vr2 <= 0.0f) {
					// 3 roots
					if (r1 < r2) {
						sl = r1;
						valsl = vr1;
						sr = r2;
						valsr = vr2;
					} else {
						sl = r2;
						valsl = vr2;
						sr = r1;
						valsr = vr1;
					}
			}
		}
	}

	// Bisection method (constant number of interations)
	for (int iter = 0; iter < 10; iter++) {
		float sbi = (sl + sr) / 2.0f;
		float valsbi = (((a * sbi + b) * sbi) + c) * sbi + d;

		// Keep the half which has different signs
		if (valsl < 0.0f && valsbi >= 0.0f ||
			valsl > 0.0f && valsbi <= 0.0f) {
				sr = sbi;
				valsr = valsbi;
		} else {
			sl = sbi;
			valsl = valsbi;
		}
	}

	return 0.5f * (sl + sr);
}

// Compute gradient of (kr * sin^2 theta) at vertex i
vec CSCMesh::gradkr(int i)
{
	vec viewdir = m_viewpos - m_pTriMesh->vertices[i];
	float rlen_viewdir = 1.0f / len(viewdir);
	viewdir *= rlen_viewdir;

	float ndotv = viewdir DOT m_pTriMesh->normals[i];
	float sintheta = sqrt(1.0f - sqr(ndotv));
	float csctheta = 1.0f / sintheta;
	float u = (viewdir DOT m_pTriMesh->pdir1[i]) * csctheta;
	float v = (viewdir DOT m_pTriMesh->pdir2[i]) * csctheta;
	float kr = m_pTriMesh->curv1[i] * u*u + m_pTriMesh->curv2[i] * v*v;
	float tr = u*v * (m_pTriMesh->curv2[i] - m_pTriMesh->curv1[i]);
	float kt = m_pTriMesh->curv1[i] * (1.0f - u*u) +
		m_pTriMesh->curv2[i] * (1.0f - v*v);
	vec w     = u * m_pTriMesh->pdir1[i] + v * m_pTriMesh->pdir2[i];
	vec wperp = u * m_pTriMesh->pdir2[i] - v * m_pTriMesh->pdir1[i];
	const Vec<4> &C = m_pTriMesh->dcurv[i];

	vec g = m_pTriMesh->pdir1[i] * (u*u*C[0] + 2.0f*u*v*C[1] + v*v*C[2]) +
		m_pTriMesh->pdir2[i] * (u*u*C[1] + 2.0f*u*v*C[2] + v*v*C[3]) -
		2.0f * csctheta * tr * (rlen_viewdir * wperp +
		ndotv * (tr * w + kt * wperp));
	g *= (1.0f - sqr(ndotv));
	g -= 2.0f * kr * sintheta * ndotv * (kr * w + tr * wperp);
	return g;
}


// Find a zero crossing between val0 and val1 by linear interpolation
// Returns 0 if zero crossing is at val0, 1 if at val1, etc.
static inline float find_zero_linear(float val0, float val1)
{
	return val0 / (val0 - val1);
}


// Draw part of a zero-crossing curve on one triangle face, but only if
// "test_num/test_den" is positive.  v0,v1,v2 are the indices of the 3
// vertices, "val" are the values of the scalar field whose zero
// crossings we are finding, and "test_*" are the values we are testing
// to make sure they are positive.  This function assumes that val0 has
// opposite sign from val1 and val2 - the following function is the
// general one that figures out which one actually has the different sign.
void CSCMesh::draw_face_isoline2(int v0, int v1, int v2,
						const vector<float> &val,
						const vector<float> &test_num,
						const vector<float> &test_den,
						bool do_hermite, bool do_test, float fade)
{
	// How far along each edge?
	float w10 = do_hermite ?
		find_zero_hermite(v0, v1, val[v0], val[v1],
		gradkr(v0), gradkr(v1)) :
	find_zero_linear(val[v0], val[v1]);
	float w01 = 1.0f - w10;
	float w20 = do_hermite ?
		find_zero_hermite(v0, v2, val[v0], val[v2],
		gradkr(v0), gradkr(v2)) :
	find_zero_linear(val[v0], val[v2]);
	float w02 = 1.0f - w20;

	// Points along edges
	point p1 = w01 * m_pTriMesh->vertices[v0] + w10 * m_pTriMesh->vertices[v1];
	point p2 = w02 * m_pTriMesh->vertices[v0] + w20 * m_pTriMesh->vertices[v2];

	float test_num1 = 1.0f, test_num2 = 1.0f;
	float test_den1 = 1.0f, test_den2 = 1.0f;
	float z1 = 0.0f, z2 = 0.0f;
	bool valid1 = true;
	if (do_test) {
		// Interpolate to find value of test at p1, p2
		test_num1 = w01 * test_num[v0] + w10 * test_num[v1];
		test_num2 = w02 * test_num[v0] + w20 * test_num[v2];
		if (!test_den.empty()) {
			test_den1 = w01 * test_den[v0] + w10 * test_den[v1];
			test_den2 = w02 * test_den[v0] + w20 * test_den[v2];
		}
		// First point is valid iff num1/den1 is positive,
		// i.e. the num and den have the same sign
		valid1 = ((test_num1 >= 0.0f) == (test_den1 >= 0.0f));
		// There are two possible zero crossings of the test,
		// corresponding to zeros of the num and den
		if ((test_num1 >= 0.0f) != (test_num2 >= 0.0f))
			z1 = test_num1 / (test_num1 - test_num2);
		if ((test_den1 >= 0.0f) != (test_den2 >= 0.0f))
			z2 = test_den1 / (test_den1 - test_den2);
		// Sort and order the zero crossings
		if (z1 == 0.0f)
			z1 = z2, z2 = 0.0f;
		else if (z2 < z1)
			swap(z1, z2);
	}

	// If the beginning of the segment was not valid, and
	// no zero crossings, then whole segment invalid
	if (!valid1 && !z1 && !z2)
		return;

	// Draw the valid piece(s)
	int npts = 0;
	if (valid1) {
		glColor4f(m_lineColor[0], m_lineColor[1], m_lineColor[2],
			test_num1 / (test_den1 * fade + test_num1));
		glVertex3fv(p1);
		npts++;
	}
	if (z1) {
		float num = (1.0f - z1) * test_num1 + z1 * test_num2;
		float den = (1.0f - z1) * test_den1 + z1 * test_den2;
		glColor4f(m_lineColor[0], m_lineColor[1], m_lineColor[2],
			num / (den * fade + num));
		glVertex3fv((1.0f - z1) * p1 + z1 * p2);
		npts++;
	}
	if (z2) {
		float num = (1.0f - z2) * test_num1 + z2 * test_num2;
		float den = (1.0f - z2) * test_den1 + z2 * test_den2;
		glColor4f(m_lineColor[0], m_lineColor[1], m_lineColor[2],
			num / (den * fade + num));
		glVertex3fv((1.0f - z2) * p1 + z2 * p2);
		npts++;
	}
	if (npts != 2) {
		glColor4f(m_lineColor[0], m_lineColor[1], m_lineColor[2],
			test_num2 / (test_den2 * fade + test_num2));
		glVertex3fv(p2);
	}
}


// See above.  This is the driver function that figures out which of
// v0, v1, v2 has a different sign from the others.
void CSCMesh::draw_face_isoline(int v0, int v1, int v2,
					   const vector<float> &val,
					   const vector<float> &test_num,
					   const vector<float> &test_den,
					   const vector<float> &ndotv,
					   bool do_bfcull, bool do_hermite,
					   bool do_test, float fade)
{
	// Backface culling
	if (likely(do_bfcull && ndotv[v0] <= 0.0f &&
		ndotv[v1] <= 0.0f && ndotv[v2] <= 0.0f))
		return;

	// Quick reject if derivs are negative
	if (do_test) {
		if (test_den.empty()) {
			if (test_num[v0] <= 0.0f &&
				test_num[v1] <= 0.0f &&
				test_num[v2] <= 0.0f)
				return;
		} else {
			if (test_num[v0] <= 0.0f && test_den[v0] >= 0.0f &&
				test_num[v1] <= 0.0f && test_den[v1] >= 0.0f &&
				test_num[v2] <= 0.0f && test_den[v2] >= 0.0f)
				return;
			if (test_num[v0] >= 0.0f && test_den[v0] <= 0.0f &&
				test_num[v1] >= 0.0f && test_den[v1] <= 0.0f &&
				test_num[v2] >= 0.0f && test_den[v2] <= 0.0f)
				return;
		}
	}

	// Figure out which val has different sign, and draw
	if (val[v0] < 0.0f && val[v1] >= 0.0f && val[v2] >= 0.0f ||
		val[v0] > 0.0f && val[v1] <= 0.0f && val[v2] <= 0.0f)
		draw_face_isoline2(v0, v1, v2,
		val, test_num, test_den,
		do_hermite, do_test, fade);
	else if (val[v1] < 0.0f && val[v2] >= 0.0f && val[v0] >= 0.0f ||
		val[v1] > 0.0f && val[v2] <= 0.0f && val[v0] <= 0.0f)
		draw_face_isoline2(v1, v2, v0,
		val, test_num, test_den,
		do_hermite, do_test, fade);
	else if (val[v2] < 0.0f && val[v0] >= 0.0f && val[v1] >= 0.0f ||
		val[v2] > 0.0f && val[v0] <= 0.0f && val[v1] <= 0.0f)
		draw_face_isoline2(v2, v0, v1,
		val, test_num, test_den,
		do_hermite, do_test, fade);
}

// Draw the ridges (valleys) of the mesh
void CSCMesh::draw_mesh_ridges(bool do_ridge, const vector<float> &ndotv,
					  bool do_bfcull, bool do_test, float thresh)
{
	const int *t = &m_pTriMesh->tstrips[0];
	const int *stripend = t;
	const int *end = t + m_pTriMesh->tstrips.size();

	// Walk through triangle strips
	while (1) {
		if (unlikely(t >= stripend)) {
			if (unlikely(t >= end))
				return;
			// New strip: each strip is stored as
			// length followed by indices
			stripend = t + 1 + *t;
			// Skip over length plus first two indices of
			// first face
			t += 3;
		}

		draw_face_ridges(*(t-2), *(t-1), *t,
			do_ridge, ndotv, do_bfcull, do_test, thresh);
		t++;
	}
}


// Draw ridges or valleys (depending on do_ridge) in a triangle v0,v1,v2
// - uses ndotv for backface culling (enabled with do_bfcull)
// - do_test checks for curvature maxima/minina for ridges/valleys
//   (when off, it draws positive minima and negative maxima)
// Note: this computes ridges/valleys every time, instead of once at the
//   start (given they aren't view dependent, this is wasteful)
// Algorithm based on formulas of Ohtake et al., 2004.
void CSCMesh::draw_face_ridges(int v0, int v1, int v2,
					  bool do_ridge,
					  const vector<float> &ndotv,
					  bool do_bfcull, bool do_test, float thresh)
{
	// Backface culling
	if (likely(do_bfcull &&
		ndotv[v0] <= 0.0f && ndotv[v1] <= 0.0f && ndotv[v2] <= 0.0f))
		return;

	// Check if ridge possible at vertices just based on curvatures
	if (do_ridge) {
		if ((m_pTriMesh->curv1[v0] <= 0.0f) ||
			(m_pTriMesh->curv1[v1] <= 0.0f) ||
			(m_pTriMesh->curv1[v2] <= 0.0f))
			return;
	} else {
		if ((m_pTriMesh->curv1[v0] >= 0.0f) ||
			(m_pTriMesh->curv1[v1] >= 0.0f) ||
			(m_pTriMesh->curv1[v2] >= 0.0f))
			return;
	}

	// Sign of curvature on ridge/valley
	float rv_sign = do_ridge ? 1.0f : -1.0f;

	// The "tmax" are the principal directions of maximal curvature,
	// flipped to point in the direction in which the curvature
	// is increasing (decreasing for valleys).  Note that this
	// is a bit different from the notation in Ohtake et al.,
	// but the tests below are equivalent.
	const float &emax0 = m_pTriMesh->dcurv[v0][0];
	const float &emax1 = m_pTriMesh->dcurv[v1][0];
	const float &emax2 = m_pTriMesh->dcurv[v2][0];
	vec tmax0 = rv_sign * m_pTriMesh->dcurv[v0][0] * m_pTriMesh->pdir1[v0];
	vec tmax1 = rv_sign * m_pTriMesh->dcurv[v1][0] * m_pTriMesh->pdir1[v1];
	vec tmax2 = rv_sign * m_pTriMesh->dcurv[v2][0] * m_pTriMesh->pdir1[v2];

	// We have a "zero crossing" if the tmaxes along an edge
	// point in opposite directions
	bool z01 = ((tmax0 DOT tmax1) <= 0.0f);
	bool z12 = ((tmax1 DOT tmax2) <= 0.0f);
	bool z20 = ((tmax2 DOT tmax0) <= 0.0f);

	if (z01 + z12 + z20 < 2)
		return;

	if (do_test) {
		const point &p0 = m_pTriMesh->vertices[v0],
			&p1 = m_pTriMesh->vertices[v1],
			&p2 = m_pTriMesh->vertices[v2];

		// Check whether we have the correct flavor of extremum:
		// Is the curvature increasing along the edge?
		z01 = z01 && ((tmax0 DOT (p1 - p0)) >= 0.0f ||
			(tmax1 DOT (p1 - p0)) <= 0.0f);
		z12 = z12 && ((tmax1 DOT (p2 - p1)) >= 0.0f ||
			(tmax2 DOT (p2 - p1)) <= 0.0f);
		z20 = z20 && ((tmax2 DOT (p0 - p2)) >= 0.0f ||
			(tmax0 DOT (p0 - p2)) <= 0.0f);

		if (z01 + z12 + z20 < 2)
			return;
	}

	// Draw line segment
	const float &kmax0 = m_pTriMesh->curv1[v0];
	const float &kmax1 = m_pTriMesh->curv1[v1];
	const float &kmax2 = m_pTriMesh->curv1[v2];
	if (!z01) {
		draw_segment_ridge(v1, v2, v0,
			emax1, emax2, emax0,
			kmax1, kmax2, kmax0,
			thresh, false);
	} else if (!z12) {
		draw_segment_ridge(v2, v0, v1,
			emax2, emax0, emax1,
			kmax2, kmax0, kmax1,
			thresh, false);
	} else if (!z20) {
		draw_segment_ridge(v0, v1, v2,
			emax0, emax1, emax2,
			kmax0, kmax1, kmax2,
			thresh, false);
	} else {
		// All three edges have crossings -- connect all to center
		draw_segment_ridge(v1, v2, v0,
			emax1, emax2, emax0,
			kmax1, kmax2, kmax0,
			thresh, true);
		draw_segment_ridge(v2, v0, v1,
			emax2, emax0, emax1,
			kmax2, kmax0, kmax1,
			thresh, true);
		draw_segment_ridge(v0, v1, v2,
			emax0, emax1, emax2,
			kmax0, kmax1, kmax2,
			thresh, true);
	}
}


// Draw part of a ridge/valley curve on one triangle face.  v0,v1,v2
// are the indices of the 3 vertices; this function assumes that the
// curve connects points on the edges v0-v1 and v1-v2
// (or connects point on v0-v1 to center if to_center is true)
void CSCMesh::draw_segment_ridge(int v0, int v1, int v2,
	  float emax0, float emax1, float emax2,
	  float kmax0, float kmax1, float kmax2,
	  float thresh, bool to_center)
  {
	  // Interpolate to find ridge/valley line segment endpoints
	  // in this triangle and the curvatures there
	  float w10 = fabs(emax0) / (fabs(emax0) + fabs(emax1));
	  float w01 = 1.0f - w10;
	  point p01 = w01 * m_pTriMesh->vertices[v0] + w10 * m_pTriMesh->vertices[v1];
	  float k01 = fabs(w01 * kmax0 + w10 * kmax1);

	  point p12;
	  float k12;
	  if (to_center) {
		  // Connect first point to center of triangle
		  p12 = (m_pTriMesh->vertices[v0] +
			  m_pTriMesh->vertices[v1] +
			  m_pTriMesh->vertices[v2]) / 3.0f;
		  k12 = fabs(kmax0 + kmax1 + kmax2) / 3.0f;
	  } else {
		  // Connect first point to second one (on next edge)
		  float w21 = fabs(emax1) / (fabs(emax1) + fabs(emax2));
		  float w12 = 1.0f - w21;
		  p12 = w12 * m_pTriMesh->vertices[v1] + w21 * m_pTriMesh->vertices[v2];
		  k12 = fabs(w12 * kmax1 + w21 * kmax2);
	  }

	  // Don't draw below threshold
	  k01 -= thresh;
	  if (k01 < 0.0f)
		  k01 = 0.0f;
	  k12 -= thresh;
	  if (k12 < 0.0f)
		  k12 = 0.0f;

	  // Skip lines that you can't see...
	  if (k01 == 0.0f && k12 == 0.0f)
		  return;

	  // Fade lines
	  if (1) {
		  k01 /= (k01 + thresh);
		  k12 /= (k12 + thresh);
	  } else {
		  k01 = k12 = 1.0f;
	  }

	  // Draw the line segment
	  glVertex3fv(p01);
	  glVertex3fv(p12);
  }

float CSCMesh::get_dihedral_angle(int edgeIdx)
{
	int edgeNum = (int)m_pEdges.size();
	if (edgeIdx<0 || edgeIdx>=edgeNum)
	{
		printf("Edge Idx Error: cannot get dihedral angle for given edge Idx!\n");
		return 0.0;
	}
	if (m_face_normals.size()==0)
	{
		printf("Error: Face normals are not got yet!\n");
		return 0.0;
	}
	if (m_pEdges[edgeIdx]->m_faces[0]==-1 || m_pEdges[edgeIdx]->m_faces[1]==-1)	//if is boundary edge
	{
		return M_PI;
	}
	int f1 = m_pEdges[edgeIdx]->m_faces[0];
	int f2 = m_pEdges[edgeIdx]->m_faces[1];
	const vec& fn1 = m_face_normals[f1];
	const vec& fn2 = m_face_normals[f2];
	float cosine_theta = fn1 DOT fn2;
	float theta = acos(CLAMP(cosine_theta,-1.0,1.0));
	return theta;
}


void CSCMesh::hyst_connect_from_oneVertex(int edgeIdx,int vIdx,std::vector<bool> &creaseFlag)
{
	int edge_nei_num = (int)m_pVertices[vIdx]->m_edges.size();
	int nei_edge_idx=0;
	float value = 0;
	for (int i=0;i<edge_nei_num;i++)
	{
		nei_edge_idx = m_pVertices[vIdx]->m_edges[i];
		if (nei_edge_idx != edgeIdx && !creaseFlag[nei_edge_idx])
		{
			value = get_dihedral_angle(nei_edge_idx); 
			if (value >= m_crease_low_thresh) 
			{
				creaseFlag[nei_edge_idx] = true;
				hyst_connect(nei_edge_idx,creaseFlag);
			}
		}
	}
}

void CSCMesh::hyst_connect(int edgeIdx,std::vector<bool> &creaseFlag) 
{
	int v1 = m_pEdges[edgeIdx]->m_vertex[0];
	int v2 = m_pEdges[edgeIdx]->m_vertex[1];
	hyst_connect_from_oneVertex(edgeIdx,v1,creaseFlag);
	hyst_connect_from_oneVertex(edgeIdx,v2,creaseFlag);
}	


void CSCMesh::detect_crease_edge()
{
	//printf("Detecting crease edges...\n");
	m_creaseEdges.clear();
	int edgeNum = (int)m_pEdges.size();
	std::vector<bool> bCrease_flag;
	bCrease_flag.assign(edgeNum,false);
	float value = 0;
	for(int i=0;i<edgeNum;i++) 
	{
		if (bCrease_flag[i])	//already detected
			continue;
		value = get_dihedral_angle(i); 
		if (value >= m_crease_high_thresh) 
		{
			bCrease_flag[i] = true;
			hyst_connect(i,bCrease_flag);
		}
	}
	for (int i=0;i<edgeNum;i++)
	{
		if (bCrease_flag[i])
		{
			m_creaseEdges.push_back(i);
		}
	}
	//printf("done...\n");
}

void CSCMesh::face_normal()
{
	//printf("Calculating face normals...\n");
	int nf = (int)m_pTriMesh->faces.size();
	if(m_face_normals.size() != nf) 
		m_face_normals.resize(nf);
	if (m_face_areas.size()!=nf)
		m_face_areas.resize(nf);
#pragma omp parallel for
	for (int i = 0; i < nf; i++) 
	{
		const point &p0 = m_pTriMesh->vertices[m_pTriMesh->faces[i][0]];
		const point &p1 = m_pTriMesh->vertices[m_pTriMesh->faces[i][1]];
		const point &p2 = m_pTriMesh->vertices[m_pTriMesh->faces[i][2]];
		vec a = p0-p1, b = p1-p2;
		vec facenormal = a CROSS b;
		m_face_areas[i] = len(facenormal)*0.5;
		m_face_normals[i] = normalize(facenormal);
	}
	//printf("done...\n");
}

void CSCMesh::createMesh()
{
	if (!m_pTriMesh)
	{
		m_pTriMesh = new TriMesh();
	}
}

void CSCMesh::setTriMesh(TriMesh *pMesh)
{
	destroyTriMesh();
	m_pTriMesh = pMesh;
	initialize();
}