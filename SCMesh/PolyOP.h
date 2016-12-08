#ifndef _POLY_OP_
#define _POLY_OP_

#include "../TriMesh/TriMesh.h"
#include <carve/polyhedron_decl.hpp>
#include "../MyUtil/MyType.h"
#include "../MyUtil/MyPara.h"
#include <carve/triangulator.hpp>

enum CSG_TYPE {UNION, INTERSECT, MINUS};

typedef carve::poly::Polyhedron poly_t;

TriMesh *Polyhedron_2_TriMesh(const poly_t *pPolyhedron);
poly_t *TriMesh_2_Polyhedron(const TriMesh *pTriMesh);
void transform_polyhedron(const MAT3D &rot, const VEC3D &t, poly_t *pPolyhedron);
void local_scale_polyhedron(poly_t *pPolyhedron, const double scale[3]);
void init_2_local_scale_polyhedron(poly_t *pInit, poly_t *pPolyhedron, const double scale[3]);
//void transform_trimesh(const MAT3D &rot, const VEC3D &t, TriMesh *pTriMesh, bool bCalcNormal=false);
void transform_polyhedron(const MAT3D &rot, const VEC3D &t, const poly_t *pSrc, poly_t *pTgt);
poly_t *transform_polyhedron_new(const MAT3D &rot, const VEC3D &t, const poly_t *pSrc);
poly_t *local_scale_polyhedron_new(const poly_t *pPolyhedron, const double scale[3]);
void transform_trimesh(const MAT3D &rot, const VEC3D &t, const TriMesh *pSrc, TriMesh *pTgt, bool bCalcNormal=false);
void local_scale_trimesh(TriMesh *pMesh, const double scale[3]);
void init_2_local_scale_trimesh(TriMesh *pInit, TriMesh *pMesh, const double scale[3]);
TriMesh *clone_trimesh(const TriMesh *pMesh);
poly_t *clone_Polyhedron(const poly_t *pPoly);

poly_t *csg_operate(const poly_t *A, const poly_t *B, CSG_TYPE type);

bool detectTriMeshCollision(const TriMesh *pMesh1, const TriMesh *pMesh2);

poly_t *create_polyHedron_sweepFace(const std::vector<VEC3D> &face_poly, const VEC3D &z_axis, double h);

void render_polyhedron_NPR(const poly_t *pPolyHedron, const float color[3], const RENDER_PARA &para);

void triangulate_polyhedron(const poly_t *pPolyhedron, std::vector<carve::triangulate::tri_idx> &result);

void tesselate_smart(const vector<point> &verts, const vector<int> &thisface,
	vector<TriMesh::Face> &tris);
#endif