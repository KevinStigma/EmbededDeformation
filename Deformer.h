#ifndef DEFORMER_H
#define DEFORMER_H
#include "Data_Structure.h"
#include "../kdtree/PointTree.h"
#include "TriMesh/TriMesh.h"
#include <windows.h>
#include <WinBase.h>
#include <QLineEdit>
class MeshViewer;
class  Deformer
{
public:
	struct SelVert
	{
		int id;
		int proxy_id;
		Vec3 exp_pos;
		SelVert(int idx=-1,Vec3 pos=Vec3(0,0,0),int pid=-1):id(idx),exp_pos(pos),proxy_id(pid){}
	};
	struct DeformVerts
	{
		std::vector<int> nodeIds;
		std::vector<double>nodeWeights;
		//WeightNode(int id=-1,double w=0):ind(id),weight(w){}
	};

	Deformer();
	~Deformer();
	void setModel(MeshViewer* meshViwer);
	void setGraphEdge(const std::vector<VecSet(int)>&ge);
	void setDeformVert(const VecList(DeformVerts)&dv);
	void initDeformation(bool has_load=false);
	void deform();
	void real_time_deform(int box_id);

	inline void getWeights(double& rotw,double& regw,double& conw)const
	{
		rotw=RotWeight;
		regw=RegWeight;
		conw=ConWeight;
	}
	inline void setWeights(double rotw,double regw,double conw)
	{
		RotWeight=rotw;
		RegWeight=regw;
		ConWeight=conw;
		std::cout<<"rot:"<<RotWeight<<std::endl;
		std::cout<<"reg:"<<RegWeight<<std::endl;
		std::cout<<"con:"<<ConWeight<<std::endl;
	}
	inline void clearBoxes()
	{
		m_sel_verts.clear();
		box_vert_ind.clear();
	}
protected:
	void buildDeformationGraph(TriMesh* p_tri_mesh,std::vector<Vec3>&graph_node,int sample_count);
	void buildDeformVertsNodes(TriMesh* tri_mesh,const std::vector<Vec3>& graph,std::vector<DeformVerts>& dVerts,VecList(int) &dNodes,VecSet(int)& graph_edge);
	void setIdentityRots(Vector& x,int nNodes);
	int constructSelectecVertices(int pid);
	void constructSelectecVertices4RealTime(int box_id);
	void DefineFxDim(int pid,int count,int &n);
	void DefineJacobiMatDim(int pid,int& m,int& n);


	double Optimize(int pid,Vector& xStart, int nMaxIter);

	void initJacobiMat(int pid,SpMat& jacobi, SpMat& jacobiT);
	void FastCalcJacobiMat(int pid,const Vector& x,SpMat& jacobi, SpMat& jacobiT);
	void CalcJacobiMat(int pid,const Vector& x,SpMat& jacobi, SpMat& jacobiT);
	void CalcEnergyFunc(int pid,Vector& x,Vector& fx);
	void DefineJacobiStructure(int pid,SpMat& jacobi, SpMat& jacobiT);
	void updateVertex_Node(int pid,const Vector&x);
	void modifyJTJ(SpMat& JtJ);
	void FastAtAGivenStructure(const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& At, Eigen::SparseMatrix<double>& AtA);

	void timePassRecord(std::string info,const DWORD& start_time);
private:
	std::vector<std::vector<DeformVerts>> m_deformVerts;
	std::vector<VecList(int)> m_deformNodes;
	std::vector<VecSet(int)> m_GraphEdge;
	std::vector<BoundingBox>* m_AABB;
	std::vector<SelVert> m_sel_verts;
	std::vector<TriMesh*>* m_proxy_mesh;
	std::vector<std::vector<Vec3>>*  m_graph;
	std::vector<std::pair<int,int>> box_vert_ind;
	std::ofstream out;

	double RotWeight,RegWeight,ConWeight;
	const int xEachNode;
	int deform_graph_max_k;
	std::vector<int> deform_max_nodes_num;
	std::vector<int>jacobi_m,jacobi_n,fx_n;
	Vector m_x;
	//SpMat m_jacobi;
	//SpMat m_jacobiT;
	std::vector<SpMat> m_jacobi;
	std::vector<SpMat> m_jacobiT;
};
#endif