//#include "stdafx.h"
#include "PolyOP.h"
#include <carve/csg.hpp>

#include <carve/interpolator.hpp>
#include <carve/csg_triangulator.hpp>
#include "write_ply.hpp"
#include "../MyUtil/MyType.h"
#include "../MyUtil/MyUtil.h"
#include <GL/GL.h>

// Tesselate an arbitrary n-gon.  Appends triangles to "tris".
static void tesselate(const vector<point> &verts, const vector<int> &thisface,
	vector<TriMesh::Face> &tris)
{
	if (thisface.size() < 3)
		return;
	if (thisface.size() == 3) {
		tris.push_back(TriMesh::Face(thisface[0],
			thisface[1],
			thisface[2]));
		return;
	}
	if (thisface.size() == 4) {
		// Triangulate in the direction that
		// gives the shorter diagonal
		const point &p0 = verts[thisface[0]], &p1 = verts[thisface[1]];
		const point &p2 = verts[thisface[2]], &p3 = verts[thisface[3]];
		float d02 = dist2(p0, p2);
		float d13 = dist2(p1, p3);
		int i = (d02 < d13) ? 0 : 1;
		tris.push_back(TriMesh::Face(thisface[i],
			thisface[(i+1)%4],
			thisface[(i+2)%4]));
		tris.push_back(TriMesh::Face(thisface[i],
			thisface[(i+2)%4],
			thisface[(i+3)%4]));
		return;
	}

	// 5-gon or higher - just tesselate arbitrarily...
	for (size_t i = 2; i < thisface.size(); i++)
		tris.push_back(TriMesh::Face(thisface[0],
		thisface[i-1],
		thisface[i]));
}


void tesselate_smart(const vector<point> &verts, const vector<int> &thisface,
	vector<TriMesh::Face> &tris)
{
	if (thisface.size() < 3)
		return;
	if (thisface.size() == 3) {
		tris.push_back(TriMesh::Face(thisface[0],
			thisface[1],
			thisface[2]));
		return;
	}
	if (thisface.size() == 4) {
		// Triangulate in the direction that
		// gives the shorter diagonal
		const point &p0 = verts[thisface[0]], &p1 = verts[thisface[1]];
		const point &p2 = verts[thisface[2]], &p3 = verts[thisface[3]];
		float d02 = dist2(p0, p2);
		float d13 = dist2(p1, p3);
		int i = (d02 < d13) ? 0 : 1;
		tris.push_back(TriMesh::Face(thisface[i],
			thisface[(i+1)%4],
			thisface[(i+2)%4]));
		tris.push_back(TriMesh::Face(thisface[i],
			thisface[(i+2)%4],
			thisface[(i+3)%4]));
		return;
	}

	// 5-gon or higher - just tesselate arbitrarily...
	std::vector<int> vTri;
	std::vector<VEC3D> vFacePts(thisface.size());
	for (int i=0;i<(int)vFacePts.size();i++)
	{
		vFacePts[i][0] = verts[thisface[i]][0];
		vFacePts[i][1] = verts[thisface[i]][1];
		vFacePts[i][2] = verts[thisface[i]][2];
	}
	MyUtil::tess_poly_3D(vFacePts,vTri);
	for (int i=0;i<(int)vTri.size();i+=3)
	{
		tris.push_back(TriMesh::Face(thisface[vTri[i]],
			thisface[vTri[i+1]],thisface[vTri[i+2]]));
	}
}

TriMesh *Polyhedron_2_TriMesh(const poly_t *pPolyhedron)
{
	TriMesh *pTriMesh = new TriMesh;
	pTriMesh->vertices.resize(pPolyhedron->vertices.size());
	for (int i=0;i<(int)pPolyhedron->vertices.size();i++)
	{
		pTriMesh->vertices[i][0] = (float)pPolyhedron->vertices[i].v[0];
		pTriMesh->vertices[i][1] = (float)pPolyhedron->vertices[i].v[1];
		pTriMesh->vertices[i][2] = (float)pPolyhedron->vertices[i].v[2];
	}
	//add faces, if necessary, tess them
#if 0
	std::vector<carve::triangulate::tri_idx> tri_ret;
	triangulate_polyhedron(pPolyhedron,tri_ret);
	pTriMesh->faces.resize(tri_ret.size());
	for (int i=0;i<(int)tri_ret.size();i++)
	{
		pTriMesh->faces[i][0] = tri_ret[i].a;
		pTriMesh->faces[i][1] = tri_ret[i].b;
		pTriMesh->faces[i][2] = tri_ret[i].c;
	}
#else
	std::vector<int> this_face;
	int vn, vidx;
	for (int i=0;i<(int)pPolyhedron->faces.size();i++)
	{
		vn = (int)pPolyhedron->faces[i].nVertices();
		this_face.resize(vn);
		for (int j=0;j<vn;j++)
		{
			const carve::poly::Vertex<3> *pV = pPolyhedron->faces[i].vertex(j);
			vidx = pPolyhedron->vertexToIndex(pV);
			this_face[j] = vidx;
		}
		tesselate_smart(pTriMesh->vertices, this_face, pTriMesh->faces);
		//tesselate(pTriMesh->vertices, this_face, pTriMesh->faces);
	}
#endif
	return pTriMesh;
}

void transform_polyhedron(const MAT3D &rot, const VEC3D &t, poly_t *pPolyhedron)
{
	VEC3D tmp;
	for (int i=0;i<(int)pPolyhedron->vertices.size();i++)
	{
		tmp[0] = pPolyhedron->vertices[i].v[0];
		tmp[1] = pPolyhedron->vertices[i].v[1];
		tmp[2] = pPolyhedron->vertices[i].v[2];
		tmp = rot * tmp + t;
		pPolyhedron->vertices[i].v[0]= tmp[0];
		pPolyhedron->vertices[i].v[1]= tmp[1];
		pPolyhedron->vertices[i].v[2]= tmp[2];
	}
}

//void transform_trimesh(const MAT3D &rot, const VEC3D &t, TriMesh *pTriMesh, bool bCalcNormal)
//{
//	VEC3D tmp;
//	for (int i=0;i<(int)pTriMesh->vertices.size();i++)
//	{
//		tmp[0] = pTriMesh->vertices[i][0];
//		tmp[1] = pTriMesh->vertices[i][1];
//		tmp[2] = pTriMesh->vertices[i][2];
//		tmp = rot * tmp + t;
//		pTriMesh->vertices[i][0]= (float)tmp[0];
//		pTriMesh->vertices[i][1]= (float)tmp[1];
//		pTriMesh->vertices[i][2]= (float)tmp[2];
//	}
//
//	if (bCalcNormal)
//	{
//		//recalc normals
//		pTriMesh->normals.resize(0);
//		pTriMesh->need_normals();
//		pTriMesh->need_tstrips();
//	}
//}

void transform_polyhedron(const MAT3D &rot, const VEC3D &t, const poly_t *pSrc, poly_t *pTgt)
{
	if (pTgt->vertices.size()!=pSrc->vertices.size() ||
		pTgt->faces.size() != pSrc->faces.size())
	{
		printf("transform polyhedron failed! v or f not same size!\n");
		return;
	}
	VEC3D tmp;
	for (int i=0;i<(int)pSrc->vertices.size();i++)
	{
		tmp[0] = pSrc->vertices[i].v[0];
		tmp[1] = pSrc->vertices[i].v[1];
		tmp[2] = pSrc->vertices[i].v[2];
		tmp = rot * tmp + t;
		pTgt->vertices[i].v[0]= tmp[0];
		pTgt->vertices[i].v[1]= tmp[1];
		pTgt->vertices[i].v[2]= tmp[2];
	}

	//pTgt->commonFaceInit(true);	
	for (size_t i = 0; i < pTgt->faces.size(); i++) 
	{
		pTgt->faces[i].recalc();
	}
	pTgt->init();
}


poly_t *transform_polyhedron_new(const MAT3D &rot, const VEC3D &t, const poly_t *pSrc)
{
	std::vector<poly_t::vertex_t> v(pSrc->vertices.size());
	VEC3D tmp;
	for (int i=0;i<(int)v.size();i++)
	{
		tmp[0] = pSrc->vertices[i].v[0];
		tmp[1] = pSrc->vertices[i].v[1];
		tmp[2] = pSrc->vertices[i].v[2];
		tmp = rot * tmp + t;
		v[i].v[0]= tmp[0];
		v[i].v[1]= tmp[1];
		v[i].v[2]= tmp[2];
	}
	std::vector<poly_t::face_t> faces(pSrc->faces.size());
	int vn, vidx;
	std::vector<const poly_t::vertex_t*> vpV;
	for (int i=0;i<(int)faces.size();i++)
	{
		vn = (int)pSrc->faces[i].nVertices();
		vpV.resize(vn);
		for (int j=0;j<vn;j++)
		{
			const carve::poly::Vertex<3> *pV = pSrc->faces[i].vertex(j);
			vidx = pSrc->vertexToIndex(pV);
			vpV[j] = &v[vidx];
		}
		faces[i] = poly_t::face_t(vpV);
	}
	return new poly_t(faces);
}


poly_t *local_scale_polyhedron_new(const poly_t *pSrc, const double scales[3])
{
	std::vector<poly_t::vertex_t> v(pSrc->vertices.size());
	for (int i=0;i<(int)v.size();i++)
	{
		v[i].v[0] = pSrc->vertices[i].v[0] * scales[0];
		v[i].v[1] = pSrc->vertices[i].v[1] * scales[1];
		v[i].v[2] = pSrc->vertices[i].v[2] * scales[2];
	}
	std::vector<poly_t::face_t> faces(pSrc->faces.size());
	int vn, vidx;
	std::vector<const poly_t::vertex_t*> vpV;
	for (int i=0;i<(int)faces.size();i++)
	{
		vn = (int)pSrc->faces[i].nVertices();
		vpV.resize(vn);
		for (int j=0;j<vn;j++)
		{
			const carve::poly::Vertex<3> *pV = pSrc->faces[i].vertex(j);
			vidx = pSrc->vertexToIndex(pV);
			vpV[j] = &v[vidx];
		}
		faces[i] = poly_t::face_t(vpV);
	}
	return new poly_t(faces);
}

void transform_trimesh(const MAT3D &rot, const VEC3D &t, const TriMesh *pSrc, TriMesh *pTgt, bool bCalcNormal)
{
	if (pTgt->vertices.size()!=pSrc->vertices.size() ||
		pTgt->faces.size() != pSrc->faces.size())
	{
		printf("transform trimesh failed! v or f not same size!\n");
		return;
	}
	VEC3D tmp;
	for (int i=0;i<(int)pSrc->vertices.size();i++)
	{
		tmp[0] = pSrc->vertices[i][0];
		tmp[1] = pSrc->vertices[i][1];
		tmp[2] = pSrc->vertices[i][2];
		tmp = rot * tmp + t;
		pTgt->vertices[i][0]= (float)tmp[0];
		pTgt->vertices[i][1]= (float)tmp[1];
		pTgt->vertices[i][2]= (float)tmp[2];
	}

	if (bCalcNormal)
	{
		//recalc normals
		pTgt->normals.resize(0);
		pTgt->need_normals();
		pTgt->need_tstrips();
	}
}

TriMesh *clone_trimesh(const TriMesh *pMesh)
{
	TriMesh *pTgt = new TriMesh();
	pTgt->vertices.assign(pMesh->vertices.begin(),pMesh->vertices.end());
	pTgt->normals.assign(pMesh->normals.begin(),pMesh->normals.end());
	pTgt->faces.assign(pMesh->faces.begin(),pMesh->faces.end());
	pTgt->need_tstrips();
	return pTgt;
}

poly_t *clone_Polyhedron(const poly_t *pPoly){
	std:: vector< poly_t:: vertex_t> v(pPoly->vertices.size());
	v.assign(pPoly->vertices.begin(), pPoly->vertices.end());

	std:: vector<poly_t::face_t> faces(pPoly->faces.size());
	for (int i = 0; i < (int)faces.size(); i++)
	{
		//faces[i] = pPoly->faces[i];
		std::vector<const poly_t::vertex_t*> pPts(pPoly->faces[i].nVertices());
		for (int j = 0; j < (int)pPts.size(); j++)
		{
			pPts[j] = &v[pPoly->vertexToIndex(pPoly->faces[i].vertex(j))];
		}

		//faces[i] = poly_t::face_t (
		//	&v[pPoly->vertexToIndex(pPoly->faces[i].vertex(0))],
		//	&v[pPoly->vertexToIndex(pPoly->faces[i].vertex(1))],
		//	&v[pPoly->vertexToIndex(pPoly->faces[i].vertex(2))]			
		//);		

		//faces[0] = poly_t::face_t(top_poly_v_order1);
		faces[i] = poly_t::face_t (pPts);

	}
	return new poly_t (faces,v);
}



poly_t *texturedCube(
	const carve::math::Matrix &transform = carve::math::Matrix::IDENT()) {

		std::vector<poly_t::vertex_t> v;
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(+1.0, +1.0, +1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(-1.0, +1.0, +1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(-1.0, -1.0, +1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(+1.0, -1.0, +1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(+1.0, +1.0, -1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(-1.0, +1.0, -1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(-1.0, -1.0, -1.0)));
		v.push_back(poly_t::vertex_t(transform * carve::geom::VECTOR(+1.0, -1.0, -1.0)));

		std::vector<poly_t::face_t> faces;

		faces.reserve(6);
		faces.push_back(poly_t::face_t(&v[0], &v[1], &v[2], &v[3]));
		faces.push_back(poly_t::face_t(&v[7], &v[6], &v[5], &v[4]));
		faces.push_back(poly_t::face_t(&v[0], &v[4], &v[5], &v[1]));
		faces.push_back(poly_t::face_t(&v[1], &v[5], &v[6], &v[2]));
		faces.push_back(poly_t::face_t(&v[2], &v[6], &v[7], &v[3]));
		faces.push_back(poly_t::face_t(&v[3], &v[7], &v[4], &v[0]));


		poly_t *poly = new poly_t(faces);

		return poly;
}

poly_t *csg_operate(const poly_t *A, const poly_t *B, CSG_TYPE type)
{
#if 0	//for debug
	poly_t *r1 = texturedCube(
		carve::math::Matrix::TRANS(0,0,4) *
		carve::math::Matrix::SCALE(4,4,4));

	poly_t *r2 = texturedCube(
		carve::math::Matrix::TRANS(0,0,5) *
		//carve::math::Matrix::ROT(80,0.707,0.707,0) *
		carve::math::Matrix::SCALE(3, 3, 4));

	carve::csg::CSG csg;
	//fv_tex.installHooks(csg);
	// f_tex_num.installHooks(csg);

	poly_t *r3 = csg.compute(r1, r2, carve::csg::CSG::A_MINUS_B);
	 writeOBJ(std::string("test.obj"), r3);
	//TriMesh *pMesh = Polyhedron_2_TriMesh(r3);
	//pMesh->write("test.obj");

	return NULL;

#else

	carve::csg::CSG csg;
	csg.hooks.registerHook(new carve::csg::CarveTriangulator, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	poly_t *pNewPolyHedron = NULL;
	switch(type)
	{
	case UNION:
		printf("UNION not done yet!\n");
		break;
	case INTERSECT:
		printf("INTERSECT not done yet!\n");
		break;
	case MINUS:
		pNewPolyHedron  = csg.compute(A, B, carve::csg::CSG::A_MINUS_B);
		writeOBJ(std::string("A.obj"), A);
		writeOBJ(std::string("B.obj"), B);
		writeOBJ(std::string("test.obj"), pNewPolyHedron);
		break;
	default:
		break;
	}
	return pNewPolyHedron;
#endif
}

poly_t *TriMesh_2_Polyhedron(const TriMesh *pTriMesh)
{
	std::vector<poly_t::vertex_t> v(pTriMesh->vertices.size());
	for (int i=0;i<(int)v.size();i++)
	{
		v[i].v[0] = pTriMesh->vertices[i][0];
		v[i].v[1] = pTriMesh->vertices[i][1];
		v[i].v[2] = pTriMesh->vertices[i][2];
	}
	std::vector<poly_t::face_t> faces(pTriMesh->faces.size());
	for (int i=0;i<(int)faces.size();i++)
	{
		faces[i] = poly_t::face_t(&v[pTriMesh->faces[i].v[0]],
									&v[pTriMesh->faces[i].v[1]],
									&v[pTriMesh->faces[i].v[2]]);
	}
	return new poly_t(faces);
}


void local_scale_polyhedron(poly_t *pPolyhedron, const double scale[3])
{
	for (int i=0;i<(int)pPolyhedron->vertices.size();i++)
	{
		pPolyhedron->vertices[i].v[0] *= scale[0];
		pPolyhedron->vertices[i].v[1] *= scale[1];
		pPolyhedron->vertices[i].v[2] *= scale[2];
	}

	//pPolyhedron->commonFaceInit(true);
	for (size_t i = 0; i < pPolyhedron->faces.size(); i++) 
	{
		pPolyhedron->faces[i].recalc();
	}
	pPolyhedron->init();
}

void init_2_local_scale_polyhedron(poly_t *pInit, poly_t *pPolyhedron, const double scale[3])
{
	for (int i=0;i<(int)pInit->vertices.size();i++)
	{
		pPolyhedron->vertices[i].v[0] = pInit->vertices[i].v[0] * scale[0];
		pPolyhedron->vertices[i].v[1] = pInit->vertices[i].v[1] * scale[1];
		pPolyhedron->vertices[i].v[2] = pInit->vertices[i].v[2] * scale[2];
	}

	//pPolyhedron->commonFaceInit(true);
	for (size_t i = 0; i < pPolyhedron->faces.size(); i++) 
	{
		pPolyhedron->faces[i].recalc();
	}
	pPolyhedron->init();
}

void local_scale_trimesh(TriMesh *pMesh, const double scale[3])
{
	for (int i=0;i<(int)pMesh->vertices.size();i++)
	{
		pMesh->vertices[i][0] *= (float)scale[0];
		pMesh->vertices[i][1] *= (float)scale[1];
		pMesh->vertices[i][2] *= (float)scale[2];
	}
}

void init_2_local_scale_trimesh(TriMesh *pInit, TriMesh *pMesh, const double scale[3])
{
	for (int i=0;i<(int)pMesh->vertices.size();i++)
	{
		pMesh->vertices[i][0] = pInit->vertices[i][0] * (float)scale[0];
		pMesh->vertices[i][1] = pInit->vertices[i][1] * (float)scale[1];
		pMesh->vertices[i][2] = pInit->vertices[i][2] * (float)scale[2];
	}
}

bool detectTriMeshCollision(const TriMesh *pMesh1, const TriMesh *pMesh2)
{
	TRIANGLE3D tri1, tri2;
	for (int i=0;i<(int)pMesh1->faces.size();i++)
	{
		const vec &v0 = pMesh1->vertices[pMesh1->faces[i].v[0]];
		const vec &v1 = pMesh1->vertices[pMesh1->faces[i].v[1]];
		const vec &v2 = pMesh1->vertices[pMesh1->faces[i].v[2]];
		tri1.V[0][0] = v0[0];	tri1.V[0][1] = v0[1];	tri1.V[0][2] = v0[2];
		tri1.V[1][0] = v1[0];	tri1.V[1][1] = v1[1];	tri1.V[1][2] = v1[2];
		tri1.V[2][0] = v2[0];	tri1.V[2][1] = v2[1];	tri1.V[2][2] = v2[2];
		for (int j=0;j<(int)pMesh2->faces.size();j++)
		{
			const vec &va = pMesh2->vertices[pMesh2->faces[j].v[0]];
			const vec &vb = pMesh2->vertices[pMesh2->faces[j].v[1]];
			const vec &vc = pMesh2->vertices[pMesh2->faces[j].v[2]];
			tri2.V[0][0] = va[0];	tri2.V[0][1] = va[1];	tri2.V[0][2] = va[2];
			tri2.V[1][0] = vb[0];	tri2.V[1][1] = vb[1];	tri2.V[1][2] = vb[2];
			tri2.V[2][0] = vc[0];	tri2.V[2][1] = vc[1];	tri2.V[2][2] = vc[2];

			//intersection detect
			INTRO_TRI3TRI3_D intr_tri3_tri3(tri1,tri2);
			if (intr_tri3_tri3.Test())
			{
				return true;
			}
		}
	}
	return false;
}

poly_t *create_polyHedron_sweepFace(const std::vector<VEC3D> &face_poly, const VEC3D &z_axis, double h)
{
#if 0	//for debug
	int face_vn = (int)face_poly.size();
	VEC3D face_center = MyUtil::calcCenter(&face_poly[0],face_vn);
	VEC3D offset = z_axis*h;
	std::vector<poly_t::vertex_t> v(face_poly.size());
	for (int i=0;i<face_vn;i++)
	{
		v[i].v[0] = face_poly[i][0];
		v[i].v[1] = face_poly[i][1];
		v[i].v[2] = face_poly[i][2];	//base face
	}

	//faces
	std::vector<poly_t::face_t> faces(1);
	std::vector<const poly_t::vertex_t*> top_poly_v_order1(face_vn);
	for (int i=0;i<face_vn;i++)	//NOTE: the order is very important!!! face normal will affect the csg result
	{
		top_poly_v_order1[i] = &v[i];
	}	
	faces[0] = poly_t::face_t(top_poly_v_order1);
	poly_t *pPolyhedron = new poly_t(faces,v);
	return pPolyhedron;
#else

	if (face_poly.size()<3)
	{
		printf("face poly pt num < 3\n");
		return NULL;
	}
	int face_vn = (int)face_poly.size();
	VEC3D face_center = MyUtil::calcCenter(&face_poly[0],face_vn);
	VEC3D offset = z_axis*h;

	//poly vertex
	std::vector<poly_t::vertex_t> v(face_poly.size()*2);
	for (int i=0;i<face_vn;i++)
	{
		v[i].v[0] = face_poly[i][0];
		v[i].v[1] = face_poly[i][1];
		v[i].v[2] = face_poly[i][2];	//base face

		v[i+face_vn].v[0] =	v[i].v[0] + offset[0];				//end face
		v[i+face_vn].v[1] =	v[i].v[1] + offset[1];
		v[i+face_vn].v[2] =	v[i].v[2] + offset[2];
	}

	//faces
	std::vector<poly_t::face_t> faces(face_vn + 2);
	//!!!!NOTE: the order of face is very important!!! if the normal is wrong, the csg will be wrong!!!
	//top face poly
	std::vector<const poly_t::vertex_t*> top_poly_v_order1(face_vn), top_poly_v_order2(face_vn);
	for (int i=0;i<face_vn;i++)	//NOTE: the order is very important!!! face normal will affect the csg result
	{
		top_poly_v_order1[i] = &v[i];
		top_poly_v_order2[i] = &v[(face_vn-1-i)];
	}	
	//bottom face poly
	std::vector<const poly_t::vertex_t*> btm_poly_v_order1(face_vn), btm_poly_v_order2(face_vn);
	for (int i=0;i<face_vn;i++)	//NOTE: the order is very important!!! face normal will affect the csg result
	{
		btm_poly_v_order1[i] = &v[(face_vn-1-i) + face_vn];
		btm_poly_v_order2[i] = &v[i + face_vn];
	}	
	//calc the correct order
	int rev_flag = 0;
	if(((face_poly[0]-face_center).Cross(face_poly[1]-face_poly[0])).Dot(z_axis)>0)
	{
		rev_flag = 1;
	}
	if(!rev_flag)
	{
		faces[0] = poly_t::face_t(top_poly_v_order1);
		faces[1] = poly_t::face_t(btm_poly_v_order1);
	}else
	{
		faces[0] = poly_t::face_t(top_poly_v_order2);
		faces[1] = poly_t::face_t(btm_poly_v_order2);
	}
	int f_counter = 2;
	//shaft faces
	if (!rev_flag)
	{
		for (int i=0;i<face_vn-1;i++)
		{
			faces[f_counter] = poly_t::face_t(&v[i], &v[i+face_vn], &v[i+1+face_vn],&v[i+1]);
			f_counter ++;
		}
		faces[f_counter] = poly_t::face_t(&v[face_vn-1], &v[face_vn-1+face_vn], &v[face_vn],&v[0]);
	}else
	{
		for (int i=0;i<face_vn-1;i++)
		{
			faces[f_counter] = poly_t::face_t(&v[i], &v[i+1],&v[i+1+face_vn],&v[i+face_vn]);
			f_counter ++;
		}
		faces[f_counter] = poly_t::face_t(&v[face_vn-1], &v[0], &v[face_vn], &v[face_vn-1+face_vn]);
	}

	poly_t *pPolyhedron = new poly_t(faces,v);
	return pPolyhedron;
#endif
}


// Draw the basic poly, which we'll overlay with lines
void render_base_polyhedron(const poly_t *pPolyhedron, const float color[3],const RENDER_PARA &para)
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
		glColor3f(color[0],color[1],color[2]);
	}

	glEnable(GL_COLOR_MATERIAL);
	int nF = (int)pPolyhedron->faces.size();

	int fvn;
	for (int i=0;i<nF;i++)
	{
		glBegin(GL_POLYGON);
		fvn = (int)pPolyhedron->faces[i].nVertices();
		for (int j=0;j<fvn;j++)
		{
			const carve::poly::Vertex<3> *pV = pPolyhedron->faces[i].vertex(j);
			glVertex3d(pV->v[0],pV->v[1],pV->v[2]);
		}
		glEnd();
	}

	glDisable(GL_COLOR_MATERIAL);

	if (!para._draw_npr_with_light)
	{
		glEnable(GL_LIGHTING);
	}

	glDisable(GL_CULL_FACE);
	glDisable(GL_POLYGON_OFFSET_FILL);
	glDepthFunc(GL_LEQUAL);
	glPopAttrib();
}



void render_polyhedron_npr_edges(const poly_t *pPolyhedron,const RENDER_PARA &para)
{
	glDisable(GL_LIGHTING);

	// Enable antialiased lines
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	glLineWidth(para._line_width);
	glColor3f(para._line_color[0],para._line_color[1],para._line_color[2]);

	//add random jitters
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

#if 1
#if 1
	glBegin(GL_LINES);
	const int nV = (int)pPolyhedron->vertices.size();
	for (int i=0;i<nV-1;i++)
	{
		const carve::poly::Vertex<3> *pV = &pPolyhedron->vertices[i];
		const carve::poly::Vertex<3> *pV2 = &pPolyhedron->vertices[i+1];

		glVertex3d(pV->v[0], pV->v[1],pV->v[2]);
		glVertex3d(pV2->v[0], pV2->v[1],pV2->v[2]);
	}
	glEnd();
#else
	glBegin(GL_LINES);
	int nF = (int)pPolyhedron->faces.size();
	int fvn;
	for (int i=0;i<nF;i++)
	{
		fvn = (int)pPolyhedron->faces[i].nVertices();
		for (int j=0;j<fvn-1;j++)
		{
			const carve::poly::Vertex<3> *pV = pPolyhedron->faces[i].vertex(j);
			const carve::poly::Vertex<3> *pV2 = pPolyhedron->faces[i].vertex(j+1);
			glVertex3f(pV->v[0], pV->v[1],pV->v[2]);
			glVertex3f(pV2->v[0], pV2->v[1],pV2->v[2]);
		}
		const carve::poly::Vertex<3> *pV = pPolyhedron->faces[i].vertex(fvn-1);
		const carve::poly::Vertex<3> *pV2 = pPolyhedron->faces[i].vertex(0);
		glVertex3f(pV->v[0], pV->v[1],pV->v[2]);
		glVertex3f(pV2->v[0], pV2->v[1],pV2->v[2]);
	}
	glEnd();
#endif
#else
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	int nF = (int)pPolyhedron->faces.size();
	int fvn;
	for (int i=0;i<nF;i++)
	{
		glBegin(GL_POLYGON);
		fvn = (int)pPolyhedron->faces[i].nVertices();
		for (int j=0;j<fvn;j++)
		{
			const carve::poly::Vertex<3> *pV = pPolyhedron->faces[i].vertex(j);
			glVertex3f(pV->v[0], pV->v[1],pV->v[2]);
		}
		glEnd();
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif
	glDisable(GL_BLEND);



	//glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POINT_SMOOTH);
	glEnable(GL_LIGHTING);
}

void render_polyhedron_NPR(const poly_t *pPolyHedron, const float color[3],const RENDER_PARA &para)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS );

	/***************render basic poly**************/
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glPolygonOffset(para._glPlyOffset_factor, para._glPlyOffset_units);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glDisable(GL_CULL_FACE);
	if (!para._draw_npr_with_light)
	{
		glDisable(GL_LIGHTING);
		glColor3f(1.f,1.f,1.f);
	}

	if (1)
	{
		glEnable(GL_COLOR_MATERIAL);
		render_base_polyhedron(pPolyHedron,color, para);
		glDisable(GL_COLOR_MATERIAL);
	}

	if (!para._draw_npr_with_light)
	{
		glEnable(GL_LIGHTING);
	}

	glDisable(GL_CULL_FACE);
	glDisable(GL_POLYGON_OFFSET_FILL);
	glDepthFunc(GL_LEQUAL);

	//draw lines
	glDepthMask(GL_FALSE); // Do not remove me, else get dotted lines
	render_polyhedron_npr_edges(pPolyHedron,para);
	glDepthMask(GL_TRUE);

	glPopAttrib();
}

void triangulate_polyhedron(const poly_t *poly, std::vector<carve::triangulate::tri_idx> &result)
{
	result.resize(0);
	carve::triangulate::tri_idx tmp;
	for (size_t i = 0; i < poly->faces.size(); ++i) 
	{
		const carve::poly::Face<3> &f = poly->faces[i];
		std::vector<carve::triangulate::tri_idx> cur_face_ret;

		std::vector<const carve::poly::Polyhedron::vertex_t *> vloop;
		f.getVertexLoop(vloop);

		carve::triangulate::triangulate(carve::poly::p2_adapt_project<3>(f.project), vloop, cur_face_ret);
		if (0)
		{
			carve::triangulate::improve(carve::poly::p2_adapt_project<3>(f.project), vloop, cur_face_ret);
		}

		for (size_t j = 0; j < cur_face_ret.size(); ++j) 
		{
			tmp.a = poly->vertexToIndex_fast(vloop[cur_face_ret[j].a]);
			tmp.b = poly->vertexToIndex_fast(vloop[cur_face_ret[j].b]);
			tmp.c = poly->vertexToIndex_fast(vloop[cur_face_ret[j].c]);
			result.push_back(tmp);
		}
	}
}