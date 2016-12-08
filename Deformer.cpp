#include "Deformer.h"
#include "MeshViewer.h"
#include <fstream>
Deformer::Deformer():deform_graph_max_k(4),m_AABB(NULL),xEachNode(12),RotWeight(0.01),RegWeight(1),ConWeight(1000),m_proxy_mesh(NULL),m_graph(NULL)
{
	/*
	RotWeight=0.1;
	RegWeight=1;
	ConWeight=100;
	*/
	RotWeight=10;
	RegWeight=1;
	ConWeight=10;
	out.open("time record.txt");
}

Deformer::~Deformer()
{
	out.close();
}

void Deformer::timePassRecord(std::string info,const DWORD& start_time)
{
	DWORD cur_time=GetTickCount();
	std::cout<<info<<std::endl;
	std::cout<<cur_time-start_time<<" ms"<<std::endl;

	out<<info<<std::endl;
	out<<cur_time-start_time<<" ms"<<std::endl;
}
void Deformer::setGraphEdge(const std::vector<VecSet(int)>&ge)
{
	m_GraphEdge.assign(ge.begin(),ge.end());
}
void Deformer::setDeformVert(const VecList(DeformVerts)&dv)
{
	m_deformVerts.assign(dv.begin(),dv.end());
}
void Deformer::setModel(MeshViewer*mesh_viwer)
{
	if(!mesh_viwer)
		return;
	//deform_max_nodes_num=pModel->numvertices*0.5;
	m_AABB=mesh_viwer->getBoundingBoxes();
	m_proxy_mesh=mesh_viwer->getProxyMesh();
	m_graph=mesh_viwer->getGraphNode();

	deform_max_nodes_num.resize(m_proxy_mesh->size());

	for(int i=0;i<deform_max_nodes_num.size();i++)
		deform_max_nodes_num[i]=200;

	mesh_viwer->setGraphEdge(&m_GraphEdge);
	mesh_viwer->setDeformVerts(&m_deformVerts);
}

void Deformer::initDeformation(bool has_load)
{
	if(!m_proxy_mesh||!m_graph)
		return;
	m_jacobi.resize(m_proxy_mesh->size());
	m_jacobiT.resize(m_proxy_mesh->size());
	fx_n.resize(m_proxy_mesh->size());
	jacobi_m.resize(m_proxy_mesh->size());
	jacobi_n.resize(m_proxy_mesh->size());

	if(!has_load)
	{
		m_graph->resize(m_proxy_mesh->size());
		m_deformVerts.resize(m_proxy_mesh->size());
		m_deformNodes.resize(m_proxy_mesh->size());
		m_GraphEdge.resize(m_proxy_mesh->size());
		for(int i=0;i<(int)m_proxy_mesh->size();i++)
		{
			std::cout<<i<<std::endl;
			buildDeformationGraph((*m_proxy_mesh)[i],(*m_graph)[i],deform_max_nodes_num[i]);
			buildDeformVertsNodes((*m_proxy_mesh)[i],(*m_graph)[i],m_deformVerts[i],m_deformNodes[i],m_GraphEdge[i]);
		}
	}
}

void Deformer::buildDeformationGraph(TriMesh* p_tri_mesh,std::vector<Vec3>&graph_node,int sample_count)
{
	const int nVerts = p_tri_mesh->vertices.size();
	int nSamples = sample_count;
	graph_node.clear();

	srand(1234);
	int cur_id = rand() % nVerts;
	std::vector<std::priority_queue<float>> distMap(nVerts);

	while(graph_node.size()<nSamples)
	{
		Vec3 v(p_tri_mesh->vertices[cur_id][0],p_tri_mesh->vertices[cur_id][1],p_tri_mesh->vertices[cur_id][2]);
		graph_node.push_back(v);

		float maxDist = 0.f;
		for (int iv = 0; iv < nVerts; iv++)
		{
			//const ldp::Float3& v1 = mesh->vertex_list[iv];
			Vec3 v1(p_tri_mesh->vertices[iv][0],p_tri_mesh->vertices[iv][1],p_tri_mesh->vertices[iv][2]);
			distMap[iv].push(-(v - v1).norm());
			float minDist = -distMap[iv].top();
			if (minDist > maxDist)
			{
				maxDist = minDist;
				cur_id = iv;
			}
		}
	}
	std::cout<<"graph node build successfuly num:"<<sample_count<<std::endl;
}


void Deformer::buildDeformVertsNodes(TriMesh* tri_mesh,const std::vector<Vec3>& graph,std::vector<DeformVerts>& dVerts,VecList(int) &dNodes,VecSet(int)& graph_edge)
{
	const int nVerts = tri_mesh->vertices.size();
	const int nNodes = graph.size();
	dVerts.clear();
	dVerts.resize(nVerts);
	dNodes.clear();
	dNodes.resize(nNodes);
	typedef ldp::kdtree::PointTree<float>::Point Point;
	const int K = deform_graph_max_k;
	assert(nNodes > K);

	ldp::kdtree::PointTree<float> tree;
	// build kdtree on the graph
	std::vector<Point> treePoints;
	for (int i=0;i<graph.size();i++)
	{
		Point p;
		p.idx = i;
		for(size_t j=0;j<3;j++)
			p.p._data[j]=graph[i](j);

		treePoints.push_back(p);
	}
	tree.build(treePoints);

	// k-nearest neighbor to init the deformVerts
	for (size_t vId = 0; vId < tri_mesh->vertices.size(); vId++)
	{
		DeformVerts& deformVert = dVerts[vId];
		Point vertP;
		for(size_t j=0;j<3;j++)
			vertP.p._data[j]=tri_mesh->vertices[vId][j];

		std::vector<Point> knnPoints(K + 1);
		tree.kNearestPoints(vertP, knnPoints);
		const float dMax = (vertP.p - knnPoints.back().p).length();
		float dSum = 0;
		for(int k = 0; k < K; k++)
		{
			deformVert.nodeIds.push_back(knnPoints[k].idx);
			float d = (vertP.p - knnPoints[k].p).length();
			float w = ldp::sqr(1 - d /(1e-6 + dMax));//ldp test  notice
			deformVert.nodeWeights.push_back(w);
			dSum += w;

			dNodes[deformVert.nodeIds.back()].push_back(vId);
		}

		if (fabs(dSum)>1e-6)
			for (int k = 0; k < K; k++)
				deformVert.nodeWeights[k] /= (double)dSum;
	}// end for vId

	graph_edge.resize(nNodes);
	for (int iVert = 0; iVert < (int)dVerts.size(); iVert++)
	{
		const DeformVerts& dv = dVerts[iVert];

		for (int k1 = 0; k1 <(int)dv.nodeIds.size(); k1++)
		{
			for (int k2 = k1 + 1; k2 <(int)dv.nodeIds.size(); k2++)
			{
				int node_id1=dv.nodeIds[k1];
				int node_id2=dv.nodeIds[k2];
				graph_edge[node_id1].insert(node_id2);
				graph_edge[node_id2].insert(node_id1);
			}
		}
	}
}

void Deformer::setIdentityRots(Vector& x,int nNodes)
{
	x.resize(12 * nNodes);
	x.setZero();
	for (int i = 0; i < nNodes; i++)
	{
		x[i * xEachNode] = 1;
		x[i * xEachNode + 4] = 1;
		x[i * xEachNode + 8] = 1;
	}
}


int Deformer::constructSelectecVertices(int pid)
{
	int count=0;
	//m_sel_verts.clear();
	

	for(size_t i=0;i<m_AABB->size();i++)
	{
		const auto& bb=(*m_AABB)[i];
		if(bb.proxy_id!=pid)
			continue;
		Vec3 trans=(bb.bot_pos+bb.top_pos)*0.5-bb.init_pos;

		if(trans.norm()<1e-6)
			trans=Vec3(0,0,0);
		//else
			//count++;
		TriMesh* triMesh=(*m_proxy_mesh)[bb.proxy_id];
		std::pair<int,int> pa;
		pa.first=m_sel_verts.size();
		for(size_t j=0;j<bb.vertex_list.size();j++)
		{
			int vid=bb.vertex_list[j];
			Vec3 v_pos(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
			m_sel_verts.push_back(SelVert(vid,v_pos+trans,pid));
			count++;
		}
		pa.second=m_sel_verts.size();
		box_vert_ind[i]=pa;
	}

	return count;
}

void Deformer::constructSelectecVertices4RealTime(int box_id)
{
	//m_sel_verts.clear();
	const auto& bb=(*m_AABB)[box_id];
	const auto& pa=box_vert_ind[box_id];
	Vec3 trans=(bb.bot_pos+bb.top_pos)*0.5-bb.init_pos;

	TriMesh* triMesh=(*m_proxy_mesh)[bb.proxy_id];
	for(size_t j=0;j<bb.vertex_list.size();j++)
	{
		int vid=bb.vertex_list[j];
		Vec3 v_pos(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
		m_sel_verts[pa.first+j].exp_pos=v_pos+trans;
	}
}

void Deformer::DefineFxDim(int pid,int count,int &n)
{
	n=0;
	//Erot
	int nNodes=(*m_graph)[pid].size();
	n+=nNodes*6;

	//Ereg
	for(int i=0;i<nNodes;i++)
	{
		for(int j=0;j<(int)m_GraphEdge[pid][i].size();j++)
			n+=3;
	}

	//Econ
	n+=count*3;
}

void Deformer::DefineJacobiMatDim(int pid,int& m,int& n)
{
	n=(*m_graph)[pid].size()*xEachNode;
	m=fx_n[pid];
}

void Deformer::DefineJacobiStructure(int pid,SpMat& jacobi, SpMat& jacobiT)
{
	if(jacobi.rows()!=jacobi_m[pid]||jacobi.cols()!=jacobi_n[pid])
	{
		jacobi.resize(jacobi_m[pid],jacobi_n[pid]);
		jacobiT.resize(jacobi_n[pid],jacobi_m[pid]);
		std::cout<<"Jacobi Matrix:"<<jacobi_m[pid]<<" "<<jacobi_n[pid]<<std::endl;
	}
}


double Deformer::Optimize(int pid,Vector& xStart, int nMaxIter)
{
	//define jacobi structure
	DefineJacobiStructure(pid,m_jacobi[pid], m_jacobiT[pid]);

	//DWORD start_time=GetTickCount();

	SpMat JacTJac;
	Vector fx(m_jacobi[pid].rows()), h(m_jacobi[pid].cols()), g(m_jacobi[pid].cols()), fx1(m_jacobi[pid].rows());

	//define structure of J'J
	JacTJac = m_jacobiT[pid] * m_jacobi[pid];
	Eigen::SimplicialCholesky<SpMat> solver;
	solver.analyzePattern(JacTJac.triangularView<Eigen::Lower>());

	//timePassRecord("iteration before",start_time);

	//Gauss-Newton Optimization
	for(int iter=0; iter<nMaxIter; iter++)
	{
		std::cout<<"iteration:"<<iter<<std::endl;
		//timePassRecord("iteration",start_time);
		//std::cout<<"Jacobi Mat"<<std::endl;
		FastCalcJacobiMat(pid,xStart, m_jacobi[pid], m_jacobiT[pid]);	//J
		
		//timePassRecord("Jacobi",start_time);
		
		JacTJac = m_jacobiT[pid] * m_jacobi[pid];//J'J
		//timePassRecord("JacTJac",start_time);

		//modifyJTJ(JacTJac);
		//std::cout<<"Energy Func"<<std::endl;

		CalcEnergyFunc(pid,xStart, fx);	//f
		//timePassRecord("Energy Func",start_time);
		//solve: J'J h =  - J' f(x)
		g = m_jacobiT[pid] * (-fx);
		//timePassRecord("get g",start_time);
		
		solver.compute(JacTJac);
		//timePassRecord("solver.compute",start_time);
		solver.factorize(JacTJac.triangularView<Eigen::Lower>());
		h = solver.solve(g);

		double normv = xStart.norm();
		double old_energy = fx.dot(fx);

		//timePassRecord("solve",start_time);
		//std::cout<<"old_energy:"<<fx.dot(fx)<<std::endl;
		for (double alpha = 1; alpha > 1e-15; alpha *= 0.5)
		{
			Vector x = xStart + h;
			CalcEnergyFunc(pid,x, fx1);	//f
			double new_energy = fx1.dot(fx1);
			if (new_energy > old_energy)
				h = h * 0.5;
			else
			{
				xStart = x;
				break;
			}
		}
		double normh = h.norm();
		std::cout<<"norm:"<<normh<<std::endl;
		//timePassRecord("get h",start_time);
		//std::cout<<"result:"<<fx.dot(fx)<<std::endl;

		if(normh < (normv+real(1e-6)) * real(1e-6))
			break;
	}
	return fx.dot(fx);
}

void Deformer::FastAtAGivenStructure(const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& At, Eigen::SparseMatrix<double>& AtA)
{
	const static int nThread = 1;
	//omp_set_num_threads(nThread);

	Eigen::VectorXd Tmps[nThread];
	Eigen::VectorXi Marks[nThread];
	for(int i=0; i<nThread; i++)
	{
		Tmps[i].resize(AtA.innerSize());
		Marks[i].resize(AtA.innerSize());
		Marks[i].setZero();
	}

	//#pragma omp parallel for
	for(int j=0; j<AtA.outerSize(); j++)
	{
		int tid = 0;//omp_get_thread_num();
		Eigen::VectorXd& Tmp = Tmps[tid];
		Eigen::VectorXi& Mark = Marks[tid];
		for(Eigen::SparseMatrix<double>::InnerIterator it_A(A, j); it_A; ++it_A)
		{
			int k = it_A.index();
			double v_A = it_A.value();

			for(Eigen::SparseMatrix<double>::InnerIterator it_At(At, k); it_At; ++it_At)
			{
				int i = it_At.index();
				double v_At = it_At.value();
				if(!Mark[i])
				{
					Mark[i] = 1;
					Tmp[i] = v_A*v_At;
				}
				else
					Tmp[i] += v_A*v_At;
			}//end for it_At
		}//end for it_A

		for(Eigen::SparseMatrix<double>::InnerIterator it(AtA, j); it; ++it)
		{
			int i = it.index();
			it.valueRef() = Tmp[i];
			Mark[i] = 0;
		}
	}//end for i
}

void Deformer::deform()
{
	if(!m_graph||!m_graph->size())
	{
		std::cout<<"You must construct graph node first!"<<std::endl;
		return ;
	}

	box_vert_ind.resize(m_AABB->size());
	m_sel_verts.clear();

	for(int pid=0;pid<m_proxy_mesh->size();pid++)
	{
		/*int sel_change_bn=constructSelectecVertices(pid);
		if(!sel_change_bn)
			continue;*/

		int count=constructSelectecVertices(pid);
		if(!count)
			continue;
		std::cout<<"m_sel_verts:"<<m_sel_verts.size()<<std::endl;
		//setIdentityRots(m_x,(*m_graph)[pid].size());
		DefineFxDim(pid,count,fx_n[pid]);
		DefineJacobiMatDim(pid,jacobi_m[pid],jacobi_n[pid]);
		initJacobiMat(pid,m_jacobi[pid],m_jacobiT[pid]);
		//double value=Optimize(pid,m_x,10);
		//std::cout<<value<<std::endl;
		//updateVertex_Node(pid,m_x);
		std::cout<<"m_sel_verts:"<<m_sel_verts.size()<<std::endl;
	}
	
}

void Deformer::real_time_deform(int box_id)
{
	if(!m_graph||!m_graph->size())
	{
		std::cout<<"You must construct graph node first!"<<std::endl;
		return;
	}
	if(box_id<0)
		return;
	std::cout<<m_sel_verts.size()<<std::endl;
	constructSelectecVertices4RealTime(box_id);
	int pid=(*m_AABB)[box_id].proxy_id;
	setIdentityRots(m_x,(*m_graph)[pid].size());	
	double value=Optimize(pid,m_x,10);
	std::cout<<value<<std::endl;
	updateVertex_Node(pid,m_x);
}

void Deformer::initJacobiMat(int pid,SpMat& jacobi, SpMat& jacobiT)
{
	std:vector<Eigen::Triplet<double>> trips;

	int index=0;
	//Erot
	const auto&graph_nodes=(*m_graph)[pid];
	int nNode=graph_nodes.size();
	for(int i=0;i<nNode;i++)
	{
		int k=i*xEachNode;
		trips.push_back(Eigen::Triplet<double>(index,k+0,0));
		trips.push_back(Eigen::Triplet<double>(index,k+1,0));
		trips.push_back(Eigen::Triplet<double>(index,k+2,0));
		trips.push_back(Eigen::Triplet<double>(index,k+3,0));
		trips.push_back(Eigen::Triplet<double>(index,k+4,0));
		trips.push_back(Eigen::Triplet<double>(index,k+5,0));
		
		trips.push_back(Eigen::Triplet<double>(index+1,k+0,0));
		trips.push_back(Eigen::Triplet<double>(index+1,k+1,0));
		trips.push_back(Eigen::Triplet<double>(index+1,k+2,0));
		trips.push_back(Eigen::Triplet<double>(index+1,k+6,0));
		trips.push_back(Eigen::Triplet<double>(index+1,k+7,0));
		trips.push_back(Eigen::Triplet<double>(index+1,k+8,0));
		
		trips.push_back(Eigen::Triplet<double>(index+2,k+3,0));
		trips.push_back(Eigen::Triplet<double>(index+2,k+4,0));
		trips.push_back(Eigen::Triplet<double>(index+2,k+5,0));
		trips.push_back(Eigen::Triplet<double>(index+2,k+6,0));
		trips.push_back(Eigen::Triplet<double>(index+2,k+7,0));
		trips.push_back(Eigen::Triplet<double>(index+2,k+8,0));
		

		for(int j=0;j<3;j++)
		{
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+0,0));
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+1,0));
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+2,0));
		}
		index+=6;
	}


	//Ereg
	//std::cout<<"Jacobi Econ"<<std::endl;
	for(int i=0;i<nNode;i++)
	{
		int k1=i*xEachNode;
		for(auto t=m_GraphEdge[pid][i].begin();t!=m_GraphEdge[pid][i].end();t++)
		{
			int k=*t;
			for(int j=0;j<3;j++)
			{
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j,0));
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+3,0));
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+6,0));

				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+9,0));
				trips.push_back(Eigen::Triplet<double>(index+j,k*xEachNode+j+9,0));

			}
			index+=3;
		}
	}

	//Econ
	TriMesh*triMesh=(*m_proxy_mesh)[pid];
	for(int i=0;i<(int)m_sel_verts.size();i++)
	{
		if(m_sel_verts[i].proxy_id!=pid)
			continue;
		int vid=m_sel_verts[i].id;
		Vec3 ve(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
		for(int j=0;j<deform_graph_max_k;j++)
		{
			int nid=m_deformVerts[pid][vid].nodeIds[j];
			double wei=m_deformVerts[pid][vid].nodeWeights[j];
			Vec3 node=graph_nodes[nid];
			int k1=nid*xEachNode;
			for(int k=0;k<3;k++)
			{
				trips.push_back(Eigen::Triplet<double>(index+k,k1+k,0));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+3+k,0));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+6+k,0));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+9+k,0));
			}
		}
		index+=3;
	}

	jacobi.resize(jacobi_m[pid],jacobi_n[pid]);
	jacobi.setFromTriplets(trips.begin(),trips.end());
	jacobiT=jacobi.transpose();
}


void Deformer::FastCalcJacobiMat(int pid,const Vector& x,SpMat& jacobi, SpMat& jacobiT)
{
	double cur_weight;	
	std:vector<Eigen::Triplet<double>> trips;

	int index=0;
	cur_weight=RotWeight;
	//Erot
	const auto&graph_nodes=(*m_graph)[pid];
	int nNode=graph_nodes.size();
	for(int i=0;i<nNode;i++)
	{
		int k=i*xEachNode;
		trips.push_back(Eigen::Triplet<double>(index,k+0,x(k+3)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index,k+1,x(k+4)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index,k+2,x(k+5)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index,k+3,x(k+0)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index,k+4,x(k+1)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index,k+5,x(k+2)*cur_weight));

		trips.push_back(Eigen::Triplet<double>(index+1,k+0,x(k+6)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+1,k+1,x(k+7)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+1,k+2,x(k+8)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+1,k+6,x(k+0)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+1,k+7,x(k+1)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+1,k+8,x(k+2)*cur_weight));

		trips.push_back(Eigen::Triplet<double>(index+2,k+3,x(k+6)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+2,k+4,x(k+7)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+2,k+5,x(k+8)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+2,k+6,x(k+3)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+2,k+7,x(k+4)*cur_weight));
		trips.push_back(Eigen::Triplet<double>(index+2,k+8,x(k+5)*cur_weight));


		for(int j=0;j<3;j++)
		{
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+0,2*x(k+j*3+0)*cur_weight));
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+1,2*x(k+j*3+1)*cur_weight));
			trips.push_back(Eigen::Triplet<double>(index+3+j,k+j*3+2,2*x(k+j*3+2)*cur_weight));
		}
		index+=6;
	}


	//Ereg
	//std::cout<<"Jacobi Econ"<<std::endl;
	cur_weight=RegWeight;
	for(int i=0;i<nNode;i++)
	{
		int k1=i*xEachNode;
		for(auto t=m_GraphEdge[pid][i].begin();t!=m_GraphEdge[pid][i].end();t++)
		{
			int k=*t;
			for(int j=0;j<3;j++)
			{
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j,(graph_nodes[k](0)-graph_nodes[i](0))*cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+3,(graph_nodes[k](1)-graph_nodes[i](1))*cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+6,(graph_nodes[k](2)-graph_nodes[i](2))*cur_weight));

				trips.push_back(Eigen::Triplet<double>(index+j,k1+j+9,cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+j,k*xEachNode+j+9,-cur_weight));

			}
			index+=3;
		}
	}

	//Econ
	cur_weight=ConWeight;
	TriMesh*triMesh=(*m_proxy_mesh)[pid];
	for(int i=0;i<(int)m_sel_verts.size();i++)
	{
		if(m_sel_verts[i].proxy_id!=pid)
			continue;
		int vid=m_sel_verts[i].id;
		Vec3 ve(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
		for(int j=0;j<deform_graph_max_k;j++)
		{
			int nid=m_deformVerts[pid][vid].nodeIds[j];
			double wei=m_deformVerts[pid][vid].nodeWeights[j];
			Vec3 node=graph_nodes[nid];
			int k1=nid*xEachNode;
			for(int k=0;k<3;k++)
			{
				trips.push_back(Eigen::Triplet<double>(index+k,k1+k,wei*(ve(0)-node(0))*cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+3+k,wei*(ve(1)-node(1))*cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+6+k,wei*(ve(2)-node(2))*cur_weight));
				trips.push_back(Eigen::Triplet<double>(index+k,k1+9+k,wei*cur_weight));
			}
		}
		index+=3;
	}

	jacobi.resize(jacobi_m[pid],jacobi_n[pid]);
	jacobi.setFromTriplets(trips.begin(),trips.end());
	jacobiT=jacobi.transpose();
}

void Deformer::CalcJacobiMat(int pid,const Vector& x,SpMat& jacobi, SpMat& jacobiT)
{	
	
	/*std:vector<Eigen::Triplet<double>> a;
		a.push_back(Eigen::Triplet<double>(0,1,2));

		jacobi.resize();
		jacobi.setFromTriplets(a.begin(),a.end());
		*/
	
	double cur_weight;
	int index=0;
	std::cout<<"Jacobi row:"<<jacobi.rows()<<" Jacobi col:"<<jacobi.cols()<<std::endl;
	//std::cout<<"Jacobi Eor and Ereg"<<std::endl;
	//Erot
	cur_weight=RotWeight;

	const auto&graph_nodes=(*m_graph)[pid];
	int nNode=graph_nodes.size();
	for(int i=0;i<nNode;i++)
	{
		int k=i*xEachNode;
		jacobi.coeffRef(index,k+0)=x(k+3)*cur_weight;
		jacobi.coeffRef(index,k+1)=x(k+4)*cur_weight;
		jacobi.coeffRef(index,k+2)=x(k+5)*cur_weight;
		jacobi.coeffRef(index,k+3)=x(k+0)*cur_weight;
		jacobi.coeffRef(index,k+4)=x(k+1)*cur_weight;
		jacobi.coeffRef(index,k+5)=x(k+2)*cur_weight;

		jacobi.coeffRef(index+1,k+0)=x(k+6)*cur_weight;
		jacobi.coeffRef(index+1,k+1)=x(k+7)*cur_weight;
		jacobi.coeffRef(index+1,k+2)=x(k+8)*cur_weight;
		jacobi.coeffRef(index+1,k+6)=x(k+0)*cur_weight;
		jacobi.coeffRef(index+1,k+7)=x(k+1)*cur_weight;
		jacobi.coeffRef(index+1,k+8)=x(k+2)*cur_weight;

		jacobi.coeffRef(index+2,k+3)=x(k+6)*cur_weight;
		jacobi.coeffRef(index+2,k+4)=x(k+7)*cur_weight;
		jacobi.coeffRef(index+2,k+5)=x(k+8)*cur_weight;
		jacobi.coeffRef(index+2,k+6)=x(k+3)*cur_weight;
		jacobi.coeffRef(index+2,k+7)=x(k+4)*cur_weight;
		jacobi.coeffRef(index+2,k+8)=x(k+5)*cur_weight;


		jacobiT.coeffRef(k+0,index)=x(k+3)*cur_weight;
		jacobiT.coeffRef(k+1,index)=x(k+4)*cur_weight;
		jacobiT.coeffRef(k+2,index)=x(k+5)*cur_weight;
		jacobiT.coeffRef(k+3,index)=x(k+0)*cur_weight;
		jacobiT.coeffRef(k+4,index)=x(k+1)*cur_weight;
		jacobiT.coeffRef(k+5,index)=x(k+2)*cur_weight;

		jacobiT.coeffRef(k+0,index+1)=x(k+6)*cur_weight;
		jacobiT.coeffRef(k+1,index+1)=x(k+7)*cur_weight;
		jacobiT.coeffRef(k+2,index+1)=x(k+8)*cur_weight;
		jacobiT.coeffRef(k+6,index+1)=x(k+0)*cur_weight;
		jacobiT.coeffRef(k+7,index+1)=x(k+1)*cur_weight;
		jacobiT.coeffRef(k+8,index+1)=x(k+2)*cur_weight;

		jacobiT.coeffRef(k+3,index+2)=x(k+6)*cur_weight;
		jacobiT.coeffRef(k+4,index+2)=x(k+7)*cur_weight;
		jacobiT.coeffRef(k+5,index+2)=x(k+8)*cur_weight;
		jacobiT.coeffRef(k+6,index+2)=x(k+3)*cur_weight;
		jacobiT.coeffRef(k+7,index+2)=x(k+4)*cur_weight;
		jacobiT.coeffRef(k+8,index+2)=x(k+5)*cur_weight;

		for(int j=0;j<3;j++)
		{
			jacobi.coeffRef(index+3+j,k+j*3+0)=2*x(k+j*3+0)*cur_weight;
			jacobi.coeffRef(index+3+j,k+j*3+1)=2*x(k+j*3+1)*cur_weight;
			jacobi.coeffRef(index+3+j,k+j*3+2)=2*x(k+j*3+2)*cur_weight;

			jacobiT.coeffRef(k+j*3+0,index+3+j)=2*x(k+j*3+0)*cur_weight;
			jacobiT.coeffRef(k+j*3+1,index+3+j)=2*x(k+j*3+1)*cur_weight;
			jacobiT.coeffRef(k+j*3+2,index+3+j)=2*x(k+j*3+2)*cur_weight;
		}
		index+=6;
	}

	
	//Ereg
	//std::cout<<"Jacobi Econ"<<std::endl;
	cur_weight=RegWeight;
	for(int i=0;i<nNode;i++)
	{
		int k1=i*xEachNode;
		for(auto t=m_GraphEdge[pid][i].begin();t!=m_GraphEdge[pid][i].end();t++)
		{
			int k=*t;
			for(int j=0;j<3;j++)
			{
				jacobi.coeffRef(index+j,k1+j)=(graph_nodes[k](0)-graph_nodes[i](0))*cur_weight;
				jacobi.coeffRef(index+j,k1+j+3)=(graph_nodes[k](1)-graph_nodes[i](1))*cur_weight;
				jacobi.coeffRef(index+j,k1+j+6)=(graph_nodes[k](2)-graph_nodes[i](2))*cur_weight;

				jacobi.coeffRef(index+j,k1+j+9)=cur_weight;
				jacobi.coeffRef(index+j,k*xEachNode+j+9)=-cur_weight;


				jacobiT.coeffRef(k1+j,index+j)=(graph_nodes[k](0)-graph_nodes[i](0))*cur_weight;
				jacobiT.coeffRef(k1+j+3,index+j)=(graph_nodes[k](1)-graph_nodes[i](1))*cur_weight;
				jacobiT.coeffRef(k1+j+6,index+j)=(graph_nodes[k](2)-graph_nodes[i](2))*cur_weight;

				jacobiT.coeffRef(k1+j+9,index+j)=cur_weight;
				jacobiT.coeffRef(k*xEachNode+j+9,index+j)=-cur_weight;
			}
			index+=3;
		}
	}
	
	//Econ
	cur_weight=ConWeight;
	TriMesh*triMesh=(*m_proxy_mesh)[pid];
	for(int i=0;i<(int)m_sel_verts.size();i++)
	{
		if(m_sel_verts[i].proxy_id!=pid)
			continue;
		int vid=m_sel_verts[i].id;
		Vec3 ve(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
		for(int j=0;j<deform_graph_max_k;j++)
		{
			int nid=m_deformVerts[pid][vid].nodeIds[j];
			double wei=m_deformVerts[pid][vid].nodeWeights[j];
			Vec3 node=graph_nodes[nid];
			int k1=nid*xEachNode;
			for(int k=0;k<3;k++)
			{
				jacobi.coeffRef(index+k,k1+k)=wei*(ve(0)-node(0))*cur_weight;
				jacobi.coeffRef(index+k,k1+3+k)=wei*(ve(1)-node(1))*cur_weight;
				jacobi.coeffRef(index+k,k1+6+k)=wei*(ve(2)-node(2))*cur_weight;
				jacobi.coeffRef(index+k,k1+9+k)=wei*cur_weight;

				jacobiT.coeffRef(k1+k,index+k)=wei*(ve(0)-node(0))*cur_weight;
				jacobiT.coeffRef(k1+3+k,index+k)=wei*(ve(1)-node(1))*cur_weight;
				jacobiT.coeffRef(k1+6+k,index+k)=wei*(ve(2)-node(2))*cur_weight;
				jacobiT.coeffRef(k1+9+k,index+k)=wei*cur_weight;
			}
		}
		index+=3;
	}
	//jacobiT=jacobi.transpose();
}

void Deformer::CalcEnergyFunc(int pid,Vector& x,Vector& fx)
{
	double cur_weight;
	auto&graph_nodes=(*m_graph)[pid];
	int nNode=graph_nodes.size();
	cur_weight=RotWeight;
	int index=0;
	
	//Erot
	for(int i=0;i<nNode;i++)
	{
		int k=i*xEachNode;
		Mat3 mat;
		mat<<x(k+0),x(k+3),x(k+6),
			x(k+1),x(k+4),x(k+7),
			x(k+2),x(k+5),x(k+8);
		Vec3 c[3];
		//std::cout<<mat<<std::endl;
		for(int j=0;j<3;j++)
			c[j]=mat.col(j);
		
		fx(index+0)=cur_weight*c[0].dot(c[1]);
		fx(index+1)=cur_weight*c[0].dot(c[2]);
		fx(index+2)=cur_weight*c[1].dot(c[2]);
		fx(index+3)=cur_weight*(c[0].dot(c[0])-1.0);
		fx(index+4)=cur_weight*(c[1].dot(c[1])-1.0);
		fx(index+5)=cur_weight*(c[2].dot(c[2])-1.0);
		
		/*std::cout<<"fx:"<<std::endl;
		for(int j=0;j<6;j++)
			std::cout<<fx(index+j)<<std::endl;
			*/
		index+=6;
	}

	//Ereg
	cur_weight=RegWeight;
	for(int i=0;i<nNode;i++)
	{
		int k=i*xEachNode;
		Mat3 mat;
		mat<<x(k+0),x(k+3),x(k+6),
			x(k+1),x(k+4),x(k+7),
			x(k+2),x(k+5),x(k+8);
		Vec3 tj(x(k+9),x(k+10),x(k+11));
		Vec3 gj=graph_nodes[i];
		for(auto t=m_GraphEdge[pid][i].begin();t!=m_GraphEdge[pid][i].end();t++)
		{
			int k1=(*t)*xEachNode;
			Vec3 gk=graph_nodes[*t];
			Vec3 tk(x(k1+9),x(k1+10),x(k1+11));
			Vec3 new_n=mat*(gk-gj)+gj+tj-gk-tk;
			fx(index+0)=cur_weight*new_n(0);
			fx(index+1)=cur_weight*new_n(1);
			fx(index+2)=cur_weight*new_n(2);
			index+=3;
		}
	}
	
	//Econ
	//std::cout<<"Energy Econ"<<std::endl;
	cur_weight=ConWeight;
	TriMesh*triMesh=(*m_proxy_mesh)[pid];
	for(int i=0;i<(int)m_sel_verts.size();i++)
	{
		if(pid!=m_sel_verts[i].proxy_id)
			continue;
		int vid=m_sel_verts[i].id;
		Vec3 ve(triMesh->vertices[vid][0],triMesh->vertices[vid][1],triMesh->vertices[vid][2]);
		Vec3 new_v(0,0,0);
			
		for(int j=0;j<deform_graph_max_k;j++)
		{
			int nid=m_deformVerts[pid][vid].nodeIds[j];
			double wei=m_deformVerts[pid][vid].nodeWeights[j];
			int k=nid*xEachNode;
			Mat3 mat;
			mat<<x(k+0),x(k+3),x(k+6),
				x(k+1),x(k+4),x(k+7),
				x(k+2),x(k+5),x(k+8);
			Vec3 gj=graph_nodes[nid];
			Vec3 tj(x(k+9),x(k+10),x(k+11));
			new_v+=(wei*(mat*(ve-gj)+gj+tj));
		}
		fx(index+0)=cur_weight*(new_v(0)-m_sel_verts[i].exp_pos(0));
		fx(index+1)=cur_weight*(new_v(1)-m_sel_verts[i].exp_pos(1));
		fx(index+2)=cur_weight*(new_v(2)-m_sel_verts[i].exp_pos(2));
		//std::cout<<fx(index+0)<<" "<<fx(index+1)<<" "<<fx(index+2)<<std::endl;
		index+=3;
	}
}

void Deformer::updateVertex_Node(int pid,const Vector&x)
{
	//int tid=m_sel_verts.back().id;
	auto& graph_nodes=(*m_graph)[pid];
	TriMesh* triMesh=(*m_proxy_mesh)[pid];
	//Vertex and normal
	int nVerts=triMesh->vertices.size();
	int nNodes=graph_nodes.size();
	for(int i=0;i<nVerts;i++)
	{
		Vec3 new_v(0,0,0),new_n(0,0,0);
		Vec3 ve(triMesh->vertices[i][0],triMesh->vertices[i][1],triMesh->vertices[i][2]);
		Vec3 nor(triMesh->normals[i][0],triMesh->normals[i][1],triMesh->normals[i][2]);
		for(int j=0;j<deform_graph_max_k;j++)
		{
			int nid=m_deformVerts[pid][i].nodeIds[j];
			double wei=m_deformVerts[pid][i].nodeWeights[j];
			int k=nid*xEachNode;
			Mat3 mat;
			mat<<x(k+0),x(k+3),x(k+6),
				x(k+1),x(k+4),x(k+7),
				x(k+2),x(k+5),x(k+8);
			Vec3 gj=graph_nodes[nid];
			Vec3 tj(x(k+9),x(k+10),x(k+11));
			new_v+=wei*(mat*(ve-gj)+gj+tj);

			Mat3 inv_t_mat=mat.inverse().transpose();
			new_n+=wei*inv_t_mat*nor;
		}
		new_n.normalize();
		triMesh->vertices[i]=vec3(new_v(0),new_v(1),new_v(2));
		triMesh->normals[i]=vec3(new_n(0),new_n(1),new_n(2));
	}

	//Node
	for(int i=0;i<nNodes;i++)
	{
		int k=i*xEachNode;
		Vec3 trans(x(k+9),x(k+10),x(k+11));
		graph_nodes[i]+=trans;
	}

	for(int i=0;i<m_AABB->size();i++)
	{
		if((*m_AABB)[i].proxy_id==pid)
		{
			//std::cout<<"now:"<<((*m_AABB)[i].bot_pos+(*m_AABB)[i].top_pos)*0.5<<std::endl;
			//std::cout<<"init:"<<(*m_AABB)[i].init_pos<<std::endl;
			(*m_AABB)[i].init_pos=((*m_AABB)[i].bot_pos+(*m_AABB)[i].top_pos)*0.5;
		}
	}
	//std::cout<<"m_sel_verts:"<<m_sel_verts.size()<<std::endl;
}

void Deformer::modifyJTJ(SpMat& JtJ)
{
	int rows=JtJ.rows();
	for(int i=0;i<rows;i++)
	{
		JtJ.coeffRef(i,i)+=1e-6;
	}
}