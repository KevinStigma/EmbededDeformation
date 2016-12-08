#include "stdafx.h"
#include "MyUtil.h"
#include "MyMacro.h"
#include "MatUtil.h"
extern "C"{
#include "../Triangle/triangle.h"
};
#include "mkl_lapack.h"
#include "gl/GL.h"
#include "../Drawing/Drawing.h"
#include "../Proxy/ProxyPrim.h"
#include "../Proxy/ProxyCam.h"
#include "../TriMesh/TriMesh.h"
namespace MyUtil
{
	void init_triangulateIO_from_poly(triangulateio *pIO, const std::vector<VEC2F> &poly, int close_flag)
	{
		init_trianglulateio(pIO);
		pIO->numberofpoints = (int)poly.size();
		pIO->pointlist = (REAL *) malloc(pIO->numberofpoints * 2 * sizeof(REAL));
		for (int i=0;i<pIO->numberofpoints;i++)
		{
			pIO->pointlist[i*2] = poly[i][0];
			pIO->pointlist[i*2+1] = poly[i][1];
		}
		/*********input segments************/
		pIO->numberofsegments = (int)poly.size()-1;
		if(close_flag) pIO->numberofsegments++;
		pIO->segmentlist = (int *) malloc(pIO->numberofsegments * 2 * sizeof(int));
		for (int i=0;i<(int)poly.size()-1;i++)
		{
			pIO->segmentlist[i*2] = i;
			pIO->segmentlist[i*2+1] = i+1;
		}
		if(close_flag)
		{
			pIO->segmentlist[pIO->numberofsegments*2-2] = (int)poly.size()-1;
			pIO->segmentlist[pIO->numberofsegments*2-1] = 0;
		}
	}

	void destroyMem_trianglulateio(struct triangulateio *io)
	{
		if(io->pointlist)  free(io->pointlist);                                               /* In / out */
		if(io->pointattributelist) free(io->pointattributelist);                                      /* In / out */
		if(io->pointmarkerlist) free(io->pointmarkerlist);                                          /* In / out */
		
		if(io->trianglelist) free(io->trianglelist);                                             /* In / out */
		if(io->triangleattributelist) free(io->triangleattributelist);                                   /* In / out */
		if(io->trianglearealist) free(io->trianglearealist);                                    /* In only */
		if(io->neighborlist) free(io->neighborlist);                                           /* Out only */
		
		if(io->segmentlist) free(io->segmentlist);                                              /* In / out */
		if(io->segmentmarkerlist) free(io->segmentmarkerlist);                             /* In / out */
		
		if(io->holelist) free(io->holelist);                        /* In / pointer to array copied out */
		
		if(io->regionlist) free(io->regionlist);                      /* In / pointer to array copied out */
		
		if(io->edgelist) free(io->edgelist);                                                 /* Out only */
		if(io->edgemarkerlist) free(io->edgemarkerlist);           /* Not used with Voronoi diagram; out only */
		if(io->normlist) free(io->normlist);              /* Used only with Voronoi diagram; out only */

		//all set to 0
		init_trianglulateio(io);
	}

	int tess_poly_2D(const std::vector<VEC2F> &poly, std::vector<int> &vTriangle, int close_flag)
	{
		//vTriangle.resize(0);
		//TESS_EC_F(poly,Wm4::Query::QT_FILTERED,0.001f,vTriangle);

		//return (int)vTriangle.size();

		vTriangle.resize(0);

		triangulateio input, triout, vorout;

		/* Define input points. */
		init_triangulateIO_from_poly(&input,poly,close_flag);

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */
		init_trianglulateio(&triout);
		init_trianglulateio(&vorout);

		/*****triangulate**************/
		triangulate("pz", &input, &triout, &vorout);

		//if vertex change, no need!!!!
		if (triout.numberofpoints!=input.numberofpoints || triout.numberoftriangles<1)
		{
			return 0;
		}

		vTriangle.resize(triout.numberoftriangles*3);
		memcpy(&vTriangle[0],triout.trianglelist,sizeof(int)*triout.numberoftriangles*3);

		destroyMem_trianglulateio(&input);
		destroyMem_trianglulateio(&triout);
		destroyMem_trianglulateio(&vorout);


		return vTriangle.size();
	}

	void init_triangulateIO_from_poly(triangulateio *pIO, const std::vector<VEC2D> &poly, int close_flag)
	{
		init_trianglulateio(pIO);
		pIO->numberofpoints = (int)poly.size();
		pIO->pointlist = (REAL *) malloc(pIO->numberofpoints * 2 * sizeof(REAL));
		for (int i=0;i<pIO->numberofpoints;i++)
		{
			pIO->pointlist[i*2] = poly[i][0];
			pIO->pointlist[i*2+1] = poly[i][1];
		}
		/*********input segments************/
		pIO->numberofsegments = (int)poly.size()-1;
		if(close_flag) pIO->numberofsegments++;
		pIO->segmentlist = (int *) malloc(pIO->numberofsegments * 2 * sizeof(int));
		for (int i=0;i<(int)poly.size()-1;i++)
		{
			pIO->segmentlist[i*2] = i;
			pIO->segmentlist[i*2+1] = i+1;
		}
		if(close_flag)
		{
			pIO->segmentlist[pIO->numberofsegments*2-2] = (int)poly.size()-1;
			pIO->segmentlist[pIO->numberofsegments*2-1] = 0;
		}
	}

	int tess_poly_2D(const std::vector<VEC2D> &poly, std::vector<int> &vTriangle, int close_flag)
	{
		vTriangle.resize(0);

		triangulateio input, triout, vorout;

		/* Define input points. */
		init_triangulateIO_from_poly(&input,poly,close_flag);

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */
		init_trianglulateio(&triout);
		init_trianglulateio(&vorout);

		/*****triangulate**************/
		triangulate("pz", &input, &triout, &vorout);

		//if vertex change, no need!!!!
		if (triout.numberofpoints!=input.numberofpoints || triout.numberoftriangles<1)
		{
			return 0;
		}

		vTriangle.resize(triout.numberoftriangles*3);
		memcpy(&vTriangle[0],triout.trianglelist,sizeof(int)*triout.numberoftriangles*3);

		destroyMem_trianglulateio(&input);
		destroyMem_trianglulateio(&triout);
		destroyMem_trianglulateio(&vorout);


		return vTriangle.size();
	}


	void init_triangulateIO_from_polies(triangulateio *pIO, const std::vector<VEC2F> &pts, 
		const std::vector<int> &vPolyStart, const std::vector<int> &vCloseFlags)
	{
		init_trianglulateio(pIO);
		pIO->numberofpoints = (int)pts.size();
		pIO->pointlist = (REAL *) malloc(pIO->numberofpoints * 2 * sizeof(REAL));
		for (int i=0;i<pIO->numberofpoints;i++)
		{
			pIO->pointlist[i*2] = pts[i][0];
			pIO->pointlist[i*2+1] = pts[i][1];
		}
		////point markders
		//pIO->pointmarkerlist = (int*)malloc(pIO->numberofpoints * sizeof(int));
		//for (int i=0;i<(int)vPolyStart.size()-1;i++)
		//{
		//	const int child_start = vPolyStart[i];
		//	const int child_end = vPolyStart[i+1];
		//	for (int j = child_start; j < child_end; j++)
		//	{
		//		pIO->pointmarkerlist[j] = i;	//markers for different poly	
		//	}
		//}


		/*********input segments************/
		pIO->numberofsegments = 0;
		for (int i=0;i<(int)vPolyStart.size()-1;i++)
		{
			const int child_start = vPolyStart[i];
			const int child_end = vPolyStart[i+1];
			pIO->numberofsegments += (child_end-child_start-1);

			if (vCloseFlags[i])
			{
				pIO->numberofsegments++;
			}
		}
		pIO->segmentlist = (int *) malloc(pIO->numberofsegments * 2 * sizeof(int));
		//pIO->segmentmarkerlist = (int *) malloc(pIO->numberofsegments * sizeof(int));

		int counter = 0;
		for (int i = 0; i < (int)vPolyStart.size()-1; i++)
		{
			const int child_start = vPolyStart[i];
			const int child_end = vPolyStart[i+1];
			for (int j = child_start; j < child_end-1; j++)
			{
				pIO->segmentlist[2*counter] = j;
				pIO->segmentlist[2*counter+1] = j+1;
				//pIO->segmentmarkerlist[counter] = i;
				counter++;
			}
			if (vCloseFlags[i])
			{
				pIO->segmentlist[2*counter] = child_end-1;
				pIO->segmentlist[2*counter+1] = child_start;
				//pIO->segmentmarkerlist[counter] = i;
				counter++;
			}
		}

		//holes
		if (vPolyStart.size()>2)
		{
			//get outer poly
			int outer_poly = getOuterPoly(pts,vPolyStart);

			if (outer_poly>=0)	//have outer poly
			{


				pIO->numberofholes = (int)vPolyStart.size()-2;	//one is outer, the last just an "end"
				pIO->holelist = (REAL*)malloc(pIO->numberofholes*2*sizeof(REAL));


				VEC2F hole_pos;
				REAL *hole_ptr = pIO->holelist;
				for (int i=0;i<(int)vPolyStart.size()-1;i++)
				{
					if (i == outer_poly)
					{
						continue;
					}

					const int child_start = vPolyStart[i];
					const int child_end = vPolyStart[i+1];
					const int n = child_end - child_start;
					if (n<1)
					{
						printf("Error: poly has no pts!\n");
						exit(0);
					}
					hole_pos[0] = hole_pos[1] = 0;
					for (int j = child_start; j < child_end; j++)
					{
						hole_pos += pts[j];
					}
					hole_pos/=(float)n;
					hole_ptr[0] = hole_pos[0];
					hole_ptr[1] = hole_pos[1];
					hole_ptr+=2;
				}	
			}
		}
	}

	int tess_poly_2D_withHole(const std::vector<VEC2F> &pts, const std::vector<int> &vPolyStart, 
							const std::vector<int> &vCloseFlags,std::vector<int> &vTriangle)
	{
		vTriangle.resize(0);

		triangulateio input, triout, vorout;

		/* Define input points. */
		init_triangulateIO_from_polies(&input,pts,vPolyStart,vCloseFlags);

		/* Make necessary initializations so that Triangle can return a */
		/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */
		init_trianglulateio(&triout);
		init_trianglulateio(&vorout);

		/*****triangulate**************/
		triangulate("pz", &input, &triout, &vorout);


		////if vertex change, no need!!!!
		if (triout.numberofpoints!=input.numberofpoints)
		{
			return 0;
		}

		vTriangle.resize(triout.numberoftriangles*3);
		memcpy(&vTriangle[0],triout.trianglelist,sizeof(int)*triout.numberoftriangles*3);

		destroyMem_trianglulateio(&input);

		triout.holelist = NULL;
		destroyMem_trianglulateio(&triout);

		vorout.holelist = NULL;
		destroyMem_trianglulateio(&vorout);


		return vTriangle.size();
	}


	int tess_poly_3D(const std::vector<VEC3D> &poly, std::vector<int> &vTriangle, int close_flag)
	{
		if (poly.size()<3)
		{
			return 0;
		}
		const VEC3D &p0 = poly[0];
		const VEC3D &p1 = poly[1];
		const VEC3D &p2 = poly[2];
		VEC3D fn = (p1-p0).Cross(p2-p1);
		fn.Normalize();
		VEC3D fu,fv;
		build_plane_UV(fn,fu,fv);
		std::vector<VEC2D> poly2D(poly.size());
		VEC2D tmp;
		for (int i=0;i<(int)poly.size();i++)
		{
			tmp = getUV(p0,fu,fv,poly[i]);
			poly2D[i][0] = tmp[0];
			poly2D[i][1] = tmp[1];
		}

		return tess_poly_2D(poly2D,vTriangle,close_flag);
	}

	/*
	   Return the angle between two vectors on a plane
	   The angle is from vector 1 to vector 2, positive anticlockwise
	   The result is between -pi -> pi
	*/
	double Angle2D(double x1, double y1, double x2, double y2)
	{
	   double dtheta,theta1,theta2;

	   theta1 = atan2(y1,x1);
	   theta2 = atan2(y2,x2);
	   dtheta = theta2 - theta1;
	   double TWOPI = M_PI * 2.0;
	   while (dtheta > M_PI)
		  dtheta -= TWOPI;
	   while (dtheta < -M_PI)
		  dtheta += TWOPI;

	   return(dtheta);
	}


	bool isPtInsidePolygon(const VEC2F *poly_pts, int n, const VEC2F &pt)
	{
		float angle=0;
		VEC2F p1,p2;
		for (int i=0;i<n-1;i++) 
		{
			p1.Y() = poly_pts[i].Y() - pt.Y();
			p1.X() = poly_pts[i].X() - pt.X();
			//p2.Y() = poly_pts[(i+1)%n].Y() - p.Y();
			//p2.X() = poly_pts[(i+1)%n].X() - p.X();
			p2.Y() = poly_pts[(i+1)].Y() - pt.Y();
			p2.X() = poly_pts[(i+1)].X() - pt.X();
			angle += (float)Angle2D(p1.X(),p1.Y(),p2.X(),p2.Y());
		}
		p1.Y() = poly_pts[n-1].Y() - pt.Y();
		p1.X() = poly_pts[n-1].X() - pt.X();
		p2.Y() = poly_pts[0].Y() - pt.Y();
		p2.X() = poly_pts[0].X() - pt.X();
		angle += (float)Angle2D(p1.X(),p1.Y(),p2.X(),p2.Y());

		if (fabs(angle) - M_PI < -1e-6f)
			return false;
		else
			return true;
	}


	float CalcAngleSum(const VEC3F &q,const VEC3F *p,int n)
	{
		int i;
		float m1,m2;
		float anglesum=0,costheta;
		VEC3F p1,p2;

		for (i=0;i<n;i++) 
		{
			p1 = p[i]-q;
			p2 = p[(i+1)%n] - q;
			m1 = p1.Length();
			m2 = p2.Length();
			if (m1*m2 <= 1e-7f)
				return(M_PI*2.f); /* We are on a node, consider this inside */
			else
				costheta = (p1.Dot(p2)) / (m1*m2);
			anglesum += acos(costheta);
		}
		return(anglesum);
	}

	double CalcAngleSum(const VEC3D &q,const VEC3D *p,int n)
	{
		int i;
		double m1,m2;
		double anglesum=0,costheta;
		VEC3D p1,p2;

		for (i=0;i<n;i++) 
		{
			p1 = p[i]-q;
			p2 = p[(i+1)%n] - q;
			m1 = p1.Length();
			m2 = p2.Length();
			if (m1*m2 <= 1e-7f)
				return(M_PI*2.f); /* We are on a node, consider this inside */
			else
				costheta = (p1.Dot(p2)) / (m1*m2);
			anglesum += acos(costheta);
		}
		return(anglesum);
	}

	bool isPtInsidePolygon(const VEC3F *poly_pts, int n, const VEC3F &pt)
	{
		float angle=CalcAngleSum(pt,poly_pts,n);

		if (fabs(angle) - M_PI*2.f < -1e-4f)
			return false;
		else
			return true;
	}

	bool isPtInsidePolygon(const VEC3D *poly_pts, int n, const VEC3D &pt)
	{
		double angle=CalcAngleSum(pt,poly_pts,n);

		if (fabs(angle) - M_PI*2.f < -1e-4f)
			return false;
		else
			return true;
	}


	bool isPolyInsidePoly(const VEC2F *small_poly, int m, const VEC2F *big_poly, int n)
	{
		for (int i=0;i<m;i++)
		{
			if (!isPtInsidePolygon(big_poly,n,small_poly[i]))
			{
				return false;
			}
		}
		return true;
	}


	bool isPtInsidePolygon(const VEC2D *poly_pts, int n, const VEC2D &pt)
	{
		double angle=0;
		VEC2D p1,p2;
		for (int i=0;i<n-1;i++) 
		{
			p1.Y() = poly_pts[i].Y() - pt.Y();
			p1.X() = poly_pts[i].X() - pt.X();
			//p2.Y() = poly_pts[(i+1)%n].Y() - p.Y();
			//p2.X() = poly_pts[(i+1)%n].X() - p.X();
			p2.Y() = poly_pts[(i+1)].Y() - pt.Y();
			p2.X() = poly_pts[(i+1)].X() - pt.X();
			angle += Angle2D(p1.X(),p1.Y(),p2.X(),p2.Y());
		}
		p1.Y() = poly_pts[n-1].Y() - pt.Y();
		p1.X() = poly_pts[n-1].X() - pt.X();
		p2.Y() = poly_pts[0].Y() - pt.Y();
		p2.X() = poly_pts[0].X() - pt.X();
		angle += Angle2D(p1.X(),p1.Y(),p2.X(),p2.Y());

		if (fabs(angle) - M_PI < -1e-6f)
			return false;
		else
			return true;
	}


	int getOuterPoly(const std::vector<VEC2F> &pts, const std::vector<int> &vPolyStart)
	{
		int fail_flag = 0;
		int outer_idx = -1;
		for (int i=0;i<(int)vPolyStart.size()-1;i++)
		{
			const VEC2F *big_poly = &pts[vPolyStart[i]];
			const int n = vPolyStart[i+1] - vPolyStart[i];
			fail_flag = 0;
			for (int j=0;j<(int)vPolyStart.size()-1;j++)
			{
				if (i==j)
				{
					continue;
				}
				const VEC2F *small_poly = &pts[vPolyStart[j]];
				const int m = vPolyStart[j+1] - vPolyStart[j];
				if (!isPolyInsidePoly(small_poly,m,big_poly,n))
				{
					fail_flag = 1;
					break;
				}
			}
			if (!fail_flag)
			{
				outer_idx = i;
			}
		}
		if (outer_idx<0)
		{
			printf("WARNING: no outer poly!\n");

		}
		return outer_idx;
	}

	VEC2F proj_pt_2_seg(const VEC2F seg_ends[2], const VEC2F &pt)
	{
		VEC2F tmp_vec = pt - seg_ends[0];
		VEC2F seg_dir = seg_ends[1] - seg_ends[0];
		seg_dir.Normalize();
		return seg_ends[0] + seg_dir * tmp_vec.Dot(seg_dir);
	}

	float segPtDist2(const VEC2F &pt, const VEC2F seg[2], VEC2F &nearest_pt)
	{
		float seg_len = (seg[1]-seg[0]).Length();
		if ((pt-seg[0]).Dot(seg[1]-seg[0]) < 1e-8f * (pt-seg[0]).Length() * seg_len) 
		{
			nearest_pt = seg[0];
		}else if ((pt-seg[1]).Dot(seg[0]-seg[1]) < 1e-8f * (pt-seg[1]).Length() * seg_len) 
		{
			nearest_pt = seg[1];
		}else
		{
			nearest_pt = proj_pt_2_seg(seg,pt);

		}
		return (nearest_pt-pt).SquaredLength();  
	}

	bool isRectSeparate(const float rect1[4], const float rect2[4])//minx miny maxx maxy
	{
		return rect1[2] < rect2[0] || rect1[0]>rect2[2] || rect1[3]<rect2[1] || rect1[1]>rect2[3];
	}
	bool isRectSeparate(const double rect1[4], const double rect2[4])//minx miny maxx maxy
	{
		return rect1[2] < rect2[0] || rect1[0]>rect2[2] || rect1[3]<rect2[1] || rect1[1]>rect2[3];
	}


	int calcSubMat(const cv::Mat &float_mat, int left, int top, int subW, int subH, 
		cv::Mat &subMat)
	{
		int w = float_mat.cols;
		int h = float_mat.rows;
		if (left<0 || left + subW > w-1 ||
			top<0 || top + subH>h-1)
		{
			return 0;
		}

		if(subMat.rows!=subW || subMat.cols!=subH)
		{
			subMat = cv::Mat(cv::Size(subW, subH), CV_32FC1);
		}

		
		for (int i = 0; i < subH; i++)
		{
			const float *ori_ptr = float_mat.ptr<float>(top+i) + left;
			float *sub_ptr = subMat.ptr<float>(i);
			memcpy(sub_ptr,ori_ptr,sizeof(float)*subW);
		}
		return 1;
	}

	float averMat(const cv::Mat &float_mat)
	{
		float aver_val = 0.f;
		for (int i=0;i<float_mat.rows;i++)
		{
			const float *ptr = float_mat.ptr<float>(i);
			for (int j=0;j<float_mat.cols;j++)
			{
				aver_val += ptr[j];
			}
		}
		return aver_val/(float)(float_mat.rows*float_mat.cols);
	}

	int quantitizeMat(const cv::Mat &float_mat, int bin_num, cv::Mat &quantMat)
	{
		int y_interval = float_mat.rows / bin_num;
		int x_interval = float_mat.cols / bin_num;

		if (quantMat.rows != bin_num || quantMat.cols!=bin_num)
		{
			quantMat = cv::Mat(cv::Size(bin_num,bin_num),CV_32FC1);
		}

		quantMat.setTo(0);
		cv::Mat subMat(cv::Size(x_interval,y_interval),CV_32FC1);
		for (int i=0;i<quantMat.rows;i++)
		{
			const int top = i*y_interval;
			float *ptr = quantMat.ptr<float>(i);
			for (int j=0;j<quantMat.cols;j++)
			{
				const int left = j*x_interval;
				calcSubMat(float_mat,left,top,x_interval,y_interval,subMat);
				ptr[j] = averMat(subMat);
			}
		}
		return 1;
	}


	void cvMat_2_floatVec(const cv::Mat &float_mat, float *outVec)
	{
		for (int i=0;i<float_mat.rows;i++)
		{
			const float *ptr = float_mat.ptr<float>(i);
			memcpy(outVec,ptr,sizeof(float)*float_mat.cols);
			outVec += float_mat.cols;
		}
	}


	void addEqualVec(std::vector<int> &left, const std::vector<int> &right)
	{
		if (left.size()!=right.size())
		{
			printf("Fatal error: add equal vector failed! size not the same!\n");
			exit(0);
		}
		for (size_t i=0;i<left.size();i++)
		{
			left[i] += right[i];
		}
	}

	void addEqualVec(std::vector<float> &left, const std::vector<float> &right)
	{
		if (left.size()!=right.size())
		{
			printf("Fatal error: add equal vector failed! size not the same!\n");
			exit(0);
		}
		for (size_t i=0;i<left.size();i++)
		{
			left[i] += right[i];
		}
	}

	void initVec(std::vector<int> &tgt, int val)
	{
		if (!tgt.size())
		{
			return;
		}
		if (val ==0 || val==-1)
		{
			memset(&tgt[0],val,sizeof(int)*tgt.size());
		}else
		{
			std::fill(tgt.begin(),tgt.end(),val);
		}
	}

	void counts2Probs(const std::vector<int> &vCounts, std::vector<float> &vProbs)
	{
		if (!vCounts.size())
		{
			return;
		}
		if (vProbs.size()!=vCounts.size())
		{
			vProbs.resize(vCounts.size());
		}
		memset(&vProbs[0],0,sizeof(float)*vProbs.size());
		float sum = 0.f;
		for (size_t i=0;i<vCounts.size();i++)
		{
			sum += vCounts[i];
		}
		if (sum<1e-6f)
		{
			return;
		}
		float invSum = 1.f/sum;
		for (size_t i=0;i<vCounts.size();i++)
		{
			vProbs[i] = (float)vCounts[i] * invSum;
		}
	}

	void normalizeVec(std::vector<float> &probVec)
	{
		float sum = 0.f;
		for (size_t i=0;i<probVec.size();i++)
		{
			sum += probVec[i];
		}
		if (sum<1e-6f)
		{
			return;
		}
		float invSum = 1.f/sum;
		for (size_t i=0;i<probVec.size();i++)
		{
			probVec[i] = (float)probVec[i] * invSum;
		}
	}

	void principalDir(const VEC2F *pts, int n, VEC2F &maxDir, VEC2F &minDir)
	{
		if (n<=0)
		{
			printf("Error: calc principalDir failed! no points!\n");
			return;
		}
		VEC2F center;
		center[0] = center[1] = 0;
		for (int i=0;i<n;i++)
		{
			center[0] += (float)pts[i].X();
			center[1] += (float)pts[i].Y();
		}
		center /= (float)n;

		double a[4];
		memset(a, 0, sizeof(double)*4);
		for (int i=0;i<n;i++)
		{
			a[0] += (pts[i].X() - center[0])*(pts[i].X() - center[0]);
			a[1] += (pts[i].X() - center[0])*(pts[i].Y() - center[1]);
			a[3] += (pts[i].Y() - center[1])*(pts[i].Y() - center[1]);	
		}
		a[2] = a[1];


		char jobz;
		char uplo;
		int n;
		double work[100];
		int ldm;
		int lwork;
		int info;
		double w[2];
		jobz = 'V';
		uplo = 'L';
		n    = 2;
		ldm  = 2;
		lwork = 100;
		info = 0;
		dsyev(&jobz,&uplo,&n, a,&ldm,w,work,&lwork,&info); //w in ascending order

		double invSum = 1.0 / sqrt(a[2]*a[2]+a[3]*a[3]);
		maxDir[0] = (float)(a[2]*invSum);
		maxDir[1] = (float)(a[3]*invSum);
		invSum = 1.0 / sqrt(a[0]*a[0]+a[1]*a[1]);
		minDir[0] = (float)(a[0]*invSum);
		minDir[1] = (float)(a[1]*invSum);
	}

	

	void render_box(const VEC3D* points, bool bRenderEdge, float r, float g, float b,float a,
		float edge_r, float edge_g, float edge_b)
	{
		if (bRenderEdge)
		{
			//edge
			glColor3f(edge_r,edge_g,edge_b);
			glLineWidth(2.f);
			glBegin(GL_LINES);
			glVertex3d(points[0][0], points[0][1], points[0][2]);
			glVertex3d(points[1][0], points[1][1], points[1][2]);

			glVertex3d(points[2][0], points[2][1], points[2][2]);
			glVertex3d(points[1][0], points[1][1], points[1][2]);

			glVertex3d(points[2][0], points[2][1], points[2][2]);
			glVertex3d(points[3][0], points[3][1], points[3][2]);

			glVertex3d(points[0][0], points[0][1], points[0][2]);
			glVertex3d(points[3][0], points[3][1], points[3][2]);

			glVertex3d(points[4][0], points[4][1], points[4][2]);
			glVertex3d(points[5][0], points[5][1], points[5][2]);

			glVertex3d(points[6][0], points[6][1], points[6][2]);
			glVertex3d(points[5][0], points[5][1], points[5][2]);

			glVertex3d(points[6][0], points[6][1], points[6][2]);
			glVertex3d(points[7][0], points[7][1], points[7][2]);

			glVertex3d(points[4][0], points[4][1], points[4][2]);
			glVertex3d(points[7][0], points[7][1], points[7][2]);

			glVertex3d(points[4][0], points[4][1], points[4][2]);
			glVertex3d(points[0][0], points[0][1], points[0][2]);

			glVertex3d(points[1][0], points[1][1], points[1][2]);
			glVertex3d(points[5][0], points[5][1], points[5][2]);

			glVertex3d(points[6][0], points[6][1], points[6][2]);
			glVertex3d(points[2][0], points[2][1], points[2][2]);

			glVertex3d(points[3][0], points[3][1], points[3][2]);
			glVertex3d(points[7][0], points[7][1], points[7][2]);

			glEnd();
		}


		VEC3D normals[6];
		normals[0] = points[1] - points[0]; normals[0].Normalize(); normals[3] = -normals[0];
		normals[1] = points[3] - points[0]; normals[1].Normalize(); normals[4] = -normals[1];
		normals[2] = points[4] - points[0]; normals[2].Normalize(); normals[5] = -normals[2];

		int quads[] = {1,2,6,5,2,3,7,6,5,6,7,4,0,4,7,3,0,1,5,4,0,3,2,1};

		glColor4f(r,g,b,a);
		for (int i=0;i<6;i++)
		{
			glBegin(GL_POLYGON);
			//glNormal3d(normals[i][0],normals[i][1],normals[i][2]);
			for (int j=0;j<4;j++)
			{
				glVertex3d(points[quads[4*i+j]][0],points[quads[4*i+j]][1],points[quads[4*i+j]][2]);
			}
			glEnd();
		}

	}


	SEG2F build_SEG(const VEC2F &negPt, const VEC2F &posPt)
	{
		SEG2F seg;
		seg.Origin = (negPt+posPt)*0.5f;
		seg.Direction = posPt-negPt;
		seg.Extent = seg.Direction.Length()*0.5f;
		seg.Direction.Normalize();
		return seg;
	}

	SEG3F build_SEG(const VEC3F &negPt, const VEC3F &posPt)
	{
		SEG3F seg;
		seg.Origin = (negPt+posPt)*0.5f;
		seg.Direction = posPt-negPt;
		seg.Extent = seg.Direction.Length()*0.5f;
		seg.Direction.Normalize();
		return seg;
	}

	SEG2D build_SEG(const VEC2D &negPt, const VEC2D &posPt)
	{
		SEG2D seg;
		seg.Origin = (negPt+posPt)*0.5f;
		seg.Direction = posPt-negPt;
		seg.Extent = seg.Direction.Length()*0.5f;
		seg.Direction.Normalize();
		return seg;
	}

	SEG3D build_SEG(const VEC3D &negPt, const VEC3D &posPt)
	{
		SEG3D seg;
		seg.Origin = (negPt+posPt)*0.5f;
		seg.Direction = posPt-negPt;
		seg.Extent = seg.Direction.Length()*0.5f;
		seg.Direction.Normalize();
		return seg;
	}


	float pt_line_dist(const VEC3F &pt, const LINE3F &line)
	{
		VEC3F tmp = pt - line.Origin;
		VEC3F dist_vec = tmp - tmp.Dot(line.Direction) * line.Direction;
		return dist_vec.Length();
	}
	double pt_line_dist(const VEC3D &pt, const LINE3D &line)
	{
		VEC3D tmp = pt - line.Origin;
		VEC3D dist_vec = tmp - tmp.Dot(line.Direction) * line.Direction;
		return dist_vec.Length();
	}

	VEC3F proj_pt_2_line(const VEC3F &pt, const LINE3F &line)
	{
		return line.Origin + (pt - line.Origin).Dot(line.Direction) * line.Direction;
	}

	VEC3D proj_pt_2_line(const VEC3D &pt, const LINE3D &line)
	{
		return line.Origin + (pt - line.Origin).Dot(line.Direction) * line.Direction;
	}

	VEC3D proj_pt_2_plane(const VEC3D &pt, const VEC4D &plane)
	{
		double sd = signed_dist_pt_2_plane(plane,pt);
		return pt - VEC3D(plane[0],plane[1],plane[2])*sd;
	}

	VEC2F project(const VEC3F &origin, const VEC3F &pt,const VEC3F &axis1, const VEC3F &axis2)
	{
		VEC2F proj;
		proj[0] = (pt-origin).Dot(axis1);
		proj[1] = (pt-origin).Dot(axis2);
		return proj;
	}

	VEC3F unproject(const VEC3F &origin, const VEC2F &proj, const VEC3F &axis1, const VEC3F &axis2)
	{
		VEC3F pt = origin + axis1*proj[0] + axis2*proj[1];
		return pt;
	}

	VEC2D project(const VEC3D &origin, const VEC3D &pt,const VEC3D &axis1, const VEC3D &axis2)
	{
		VEC2D proj;
		proj[0] = (pt-origin).Dot(axis1);
		proj[1] = (pt-origin).Dot(axis2);
		return proj;
	}

	VEC3D unproject(const VEC3D &origin, const VEC2D &proj, const VEC3D &axis1, const VEC3D &axis2)
	{
		VEC3D pt = origin + axis1*proj[0] + axis2*proj[1];
		return pt;
	}

	void gatherLines(const std::vector<SEG2F> &segs, LINE2F lines[2])
	{
		int idx1 = rand()%segs.size();
		lines[0].Origin = segs[idx1].Origin;
		lines[0].Direction = segs[idx1].Direction;
		int idx2;
		while(1)
		{
			idx2 = rand()%segs.size();
			if(idx2!=idx1) break;
		}
		lines[1].Origin = segs[idx2].Origin;
		lines[1].Direction = segs[idx2].Direction;
	}


	LINE2F perturbLine(const LINE2F &src, float delta_angle)
	{
		float cosine = cos(delta_angle);
		float sine = sin(delta_angle);
		LINE2F tgt;
		tgt.Origin = src.Origin;
		tgt.Direction[0] = src.Direction[0] * cosine - src.Direction[1] * sine;
		tgt.Direction[1] = src.Direction[0] * sine + src.Direction[1] * cosine;
		tgt.Direction.Normalize();
		return tgt;
	}

	void link_drawing_cam_proxy(CDrawing *pDrawing, CProxyCamera *pCam, CProxyPrim *pProxy)
	{
		pDrawing->setCam(pCam);
		pDrawing->setProxy(pProxy);
		pProxy->setCam(pCam);
		pProxy->setDrawing(pDrawing);
		pCam->addDrawingProxy(pDrawing,pProxy);
	}

	void link_drawing_proxy(CDrawing *pDrawing, CProxyPrim *pProxy)
	{
		pDrawing->setCam(pProxy->getCam());
		pDrawing->setProxy(pProxy);
		pProxy->setDrawing(pDrawing);
		//pCam->addDrawingProxy(pDrawing,pProxy);
		pProxy->getCam()->addDrawing(pDrawing);
	}

	void real_pt_2_render_pt(const double real_pt[2], double render_pt[2],
		const double real_vp[4], const int vp[4])
	{
		render_pt[0] = (real_pt[0] - real_vp[0]) / real_vp[2] * vp[2];
		render_pt[1] = (real_vp[3] - (real_pt[1] - real_vp[1])) / real_vp[3] * vp[3];
	}

	LINE2D build_line(const VEC2D &negPt, const VEC2D &posPt)
	{
		LINE2D tmp;
		tmp.Origin = negPt;
		tmp.Direction = posPt-negPt;
		tmp.Direction.Normalize();
		return tmp;
	}

	//NOTE: the order must be the same as the cube_p8 in myconsts.h
	void computeCorners(const VEC3D &c, const VEC3D axis[3], const double hl[3],VEC3D corners[8])
	{
		corners[0] = c + axis[0]*hl[0] - axis[1]*hl[1] - axis[2]*hl[2];
		corners[1] = c + axis[0]*hl[0] + axis[1]*hl[1] - axis[2]*hl[2];
		corners[2] = c - axis[0]*hl[0] + axis[1]*hl[1] - axis[2]*hl[2];
		corners[3] = c - axis[0]*hl[0] - axis[1]*hl[1] - axis[2]*hl[2];
		corners[4] = c + axis[0]*hl[0] - axis[1]*hl[1] + axis[2]*hl[2];
		corners[5] = c + axis[0]*hl[0] + axis[1]*hl[1] + axis[2]*hl[2];
		corners[6] = c - axis[0]*hl[0] + axis[1]*hl[1] + axis[2]*hl[2];
		corners[7] = c - axis[0]*hl[0] - axis[1]*hl[1] + axis[2]*hl[2];
	}

	void calcProxyNewPara(const CProxyPrim *pProxy, const double vars[PROXY_VAR_NUM],
		VEC3D &new_center, VEC3D new_axes[3], double new_hls[3])
	{
		const double *ori_hls = pProxy->getInitFrame().getHLs();
		const VEC3D &ori_center = pProxy->getInitFrame().center(); 
		const VEC3D *ori_axes = pProxy->getInitFrame().getAxes();
		if (pProxy->isFixed())
		{
			memcpy(new_hls,ori_hls,sizeof(double)*3);
			new_center = ori_center;
			memcpy(new_axes,ori_axes,sizeof(VEC3D)*3);
		}else
		{
			new_hls[0] = ori_hls[0]*vars[SX];
			new_hls[1] = ori_hls[1]*vars[SY];
			new_hls[2] = ori_hls[2]*vars[SZ];


			new_center[0] = ori_center[0] + vars[DELTA_CX];
			new_center[1] = ori_center[1] + vars[DELTA_CY];
			new_center[2] = ori_center[2] + vars[DELTA_CZ];

			MAT3D rot_mat;
			rot_mat.FromEulerAnglesXYZ(vars[EULER_X],vars[EULER_Y],vars[EULER_Z]);
			for (int i=0;i<3;i++)
			{
				new_axes[i] = rot_mat * ori_axes[i];
			}
			//new_axes[0].Normalize();
			//new_axes[1] = new_axes[2].Cross(new_axes[0]);
			//new_axes[1].Normalize();
			//new_axes[2] = new_axes[0].Cross(new_axes[1]);
			//new_axes[2].Normalize();
		}
	}

	void calcArticulatedNewPara(const CArticulatedProxies *pArt, const double vars[PROXY_VAR_NUM],
		VEC3D &new_center, VEC3D new_axes[3], double new_hls[3])
	{
		const double *ori_hls = pArt->getInitFrame().getHLs();
		const VEC3D &ori_center = pArt->getInitFrame().center(); 
		const VEC3D *ori_axes = pArt->getInitFrame().getAxes();
		if (pArt->isFixed())
		{
			memcpy(new_hls,ori_hls,sizeof(double)*3);
			new_center = ori_center;
			memcpy(new_axes,ori_axes,sizeof(VEC3D)*3);
		}else
		{
			new_hls[0] = ori_hls[0]*vars[SX];
			new_hls[1] = ori_hls[1]*vars[SY];
			new_hls[2] = ori_hls[2]*vars[SZ];


			new_center[0] = ori_center[0] + vars[DELTA_CX];
			new_center[1] = ori_center[1] + vars[DELTA_CY];
			new_center[2] = ori_center[2] + vars[DELTA_CZ];

			MAT3D rot_mat;
			rot_mat.FromEulerAnglesXYZ(vars[EULER_X],vars[EULER_Y],vars[EULER_Z]);
			for (int i=0;i<3;i++)
			{
				new_axes[i] = rot_mat * ori_axes[i];
			}
			//new_axes[0].Normalize();
			//new_axes[1] = new_axes[2].Cross(new_axes[0]);
			//new_axes[1].Normalize();
			//new_axes[2] = new_axes[0].Cross(new_axes[1]);
			//new_axes[2].Normalize();
		}
	}

	void drawCircle(float x, float y, float r, bool bFill) 
	{
		static const float inc = M_PI / 12.f;
		float pi_2 = M_PI * 2.f;
		if (bFill)
		{
			glBegin(GL_POLYGON);
			for(float d = 0; d < pi_2; d += inc) 
			{
				glVertex2f(cos(d) * r + x, sin(d) * r + y);
			}
			glEnd();
		}else
		{
			glBegin(GL_LINE_LOOP);
			for(float d = 0; d < pi_2; d += inc) 
			{
				glVertex2f(cos(d) * r + x, sin(d) * r + y);
			}
			glEnd();
		}
	}

	void drawCircle(const VEC3D &c, const VEC3D &n, double r, bool bFill) 
	{
		const double inc = M_PI / 12.f;
		double pi_2 = M_PI * 2.f;


		VEC3D v = n.Cross(VEC3D(1,0,0));
		if (v.SquaredLength()<1e-6f)
		{
			v = n.Cross(VEC3D(0,1,0));
		}
		if (v.SquaredLength()<1e-6f)
		{
			v = n.Cross(VEC3D(0,0,1));
		}
		v.Normalize();
		VEC3D u = v.Cross(n);
		u.Normalize();



		VEC3D pt;
		if (bFill)
		{
			glBegin(GL_POLYGON);
			for(double d = 0; d < pi_2; d += inc) 
			{
				pt = c+ r*(cos(d)*u + sin(d)*v);
				glVertex3dv((const double*)pt);
			}
			glEnd();
		}else
		{
			glBegin(GL_LINE_LOOP);
			for(double d = 0; d < pi_2; d += inc) 
			{
				pt = c + r*(cos(d)*u + sin(d)*v);
				glVertex3dv((const double*)pt);
			}
			glEnd();
		}
	}

	VEC3D relLocal_2_global(const VEC3D &rel_local, const VEC3D &c, const VEC3D axes[3], const double hls[3])
	{
		VEC3D local_pt = rel_local;
		local_pt[0] *= hls[0];
		local_pt[1] *= hls[1];
		local_pt[2] *= hls[2];

		VEC3D global_pt;
		global_pt = c + axes[0]*local_pt[0] + axes[1]*local_pt[1] + axes[2]*local_pt[2];

		return global_pt;
	}

	void getPQ(VEC2D *q,VEC2D *p,int ptNum,MAT2D &PQ)
	{
		PQ.MakeZero();
		MAT2D temp;
		for (int i=0;i<ptNum;i++)
		{
			temp = MAT2D(p[i].X()*q[i].X(),p[i].X()*q[i].Y(),
				p[i].Y()*q[i].X(),p[i].Y()*q[i].Y());
			PQ += temp;
		}
	}


	void getInvQ(const VEC2D pts[],int ptNum, MAT2D &invQ)
	{
		double val[3] = {0};	
		double m11,m12,m22;
		for (int i=0;i<ptNum;i++)
		{
			m11 = pts[i].X() * pts[i].X();
			m12 = pts[i].X() * pts[i].Y();
			m22 = pts[i].Y() * pts[i].Y();
			val[0] += m11;
			val[1] += m12;
			val[2] += m22;
		}
		double inv_det = 1.f / (val[0] * val[2] - val[1]*val[1]);
		double invV[3];
		invV[0] = val[2] * inv_det;
		invV[1] = -val[1] * inv_det;
		invV[2] = val[0] * inv_det;
		invQ = MAT2D(invV[0],invV[1],invV[1],invV[2]);
	}


	void getAffineMat(const VEC2D &oa,const VEC2D &ob,const VEC2D &oc,	//source three points
		const VEC2D &na,const VEC2D &nb,const VEC2D &nc,	//target three points
		MAT2D &outA, VEC2D &outT)	//the affine transformation is outA*x + outT
	{
		const double inv3 = 0.3333333333f;
		VEC2D q_center = (oa+ob+oc)*inv3;
		VEC2D local_q[3] = {oa-q_center,ob-q_center,oc-q_center};
		VEC2D p_center = (na+nb+nc)*inv3;
		VEC2D local_p[3] = {na-p_center,nb-p_center,nc-p_center};

		//translation
		MAT2D PQ;
		getPQ(local_q,local_p,3,PQ);
		MAT2D inv_Q;
		getInvQ(local_q,3,inv_Q);
		outA = PQ*inv_Q;	//affine mat,

		//new point will be A(x-q_center) + p_center
		//so translation is p - Aq
		outT = p_center - outA*q_center;
	}

	void cluster_segs_dirBased(const std::vector<SEG2D> &vSegs, std::vector<std::vector<SEG2D>> &vvSegGrps, float angle_thresh)
	{
		vvSegGrps.resize(0);
		std::vector<int> flags(vSegs.size(),0);
		std::vector<SEG2D> cur_grp;
		for (int i=0;i<(int)vSegs.size();i++)
		{
			if (flags[i])
			{
				continue;
			}
			const SEG2D &self_seg = vSegs[i];
			cur_grp.resize(0);
			cur_grp.push_back(self_seg);
			flags[i] = 1;
			for (int j=i+1;j<(int)vSegs.size();j++)
			{
				if (flags[j])
				{
					continue;
				}
				const SEG2D &other_seg = vSegs[j];
				if (fabs(self_seg.Direction.Dot(other_seg.Direction))>angle_thresh)
				{
					cur_grp.push_back(other_seg);
					flags[j] = 1;
				}
			}
			vvSegGrps.push_back(cur_grp);
		}
	}


	void cluster_segs_dirLengthBased(const std::vector<SEG2D> &vSegs, std::vector<std::vector<SEG2D>> &vvSegGrps, 
		float angle_thresh, float length_thresh)
	{
		vvSegGrps.resize(0);
		std::vector<int> flags(vSegs.size(),0);
		std::vector<SEG2D> cur_grp;
		for (int i=0;i<(int)vSegs.size();i++)
		{
			if (flags[i])
			{
				continue;
			}
			const SEG2D &self_seg = vSegs[i];
			cur_grp.resize(0);
			cur_grp.push_back(self_seg);
			flags[i] = 1;
			for (int j=i+1;j<(int)vSegs.size();j++)
			{
				if (flags[j])
				{
					continue;
				}
				const SEG2D &other_seg = vSegs[j];
				if (fabs(self_seg.Direction.Dot(other_seg.Direction))>angle_thresh &&
					fabs(self_seg.Extent-other_seg.Extent)<length_thresh*0.5f)
				{
					cur_grp.push_back(other_seg);
					flags[j] = 1;
				}
			}
			vvSegGrps.push_back(cur_grp);
		}
	}


	void cluster_segs_dirLengthBased(const std::vector<SEG2D> &vSegs, 
		const std::vector<int> &vSegIndices,
		std::vector<std::vector<SEG2D>> &vvSegGrps, 
		std::vector<std::vector<int>> &vvIndiceGrps, float angle_thresh, float len_thresh)
	{
		vvSegGrps.resize(0);
		vvIndiceGrps.resize(0);
		std::vector<int> flags(vSegs.size(),0);
		std::vector<SEG2D> cur_grp;
		std::vector<int> cur_idx_grp;
		for (int i=0;i<(int)vSegs.size();i++)
		{
			if (flags[i])
			{
				continue;
			}
			const SEG2D &self_seg = vSegs[i];
			const int &self_idx = vSegIndices[i];
			cur_grp.resize(0);
			cur_idx_grp.resize(0);
			cur_grp.push_back(self_seg);
			cur_idx_grp.push_back(self_idx);
			flags[i] = 1;
			for (int j=i+1;j<(int)vSegs.size();j++)
			{
				if (flags[j])
				{
					continue;
				}
				const SEG2D &other_seg = vSegs[j];
				const int &other_idx = vSegIndices[j];
				if (fabs(self_seg.Direction.Dot(other_seg.Direction))>angle_thresh &&
					fabs(self_seg.Extent-other_seg.Extent)<len_thresh*0.5f)
				{
					cur_grp.push_back(other_seg);
					cur_idx_grp.push_back(other_idx);
					flags[j] = 1;
				}
			}
			vvSegGrps.push_back(cur_grp);
			vvIndiceGrps.push_back(cur_idx_grp);
		}
	}

	float aver_seg_grp_len(const std::vector<SEG2D> &vGrpSegs)
	{
		float aver_len = 0.f;
		if (!vGrpSegs.size())
		{
			return 0;
		}
		for (int i=0;i<(int)vGrpSegs.size();i++)
		{
			aver_len+=(float)vGrpSegs[i].Extent*2.f;
		}
		return aver_len/(float)vGrpSegs.size();
	}
	
	LINE2D perturbLine(const LINE2D &src, float delta_angle)
	{
		float cosine = cos(delta_angle);
		float sine = sin(delta_angle);
		LINE2D tgt;
		tgt.Origin = src.Origin;
		tgt.Direction[0] = src.Direction[0] * cosine - src.Direction[1] * sine;
		tgt.Direction[1] = src.Direction[0] * sine + src.Direction[1] * cosine;
		tgt.Direction.Normalize();
		return tgt;
	}

	VEC3D calcDirFromVanishPt(const MAT3D &K, const VEC2D &vpt)
	{
		MAT3D invK = MAT3D::IDENTITY;
		invK(2,2) = 1.0 / K(2,2);
		invK(0,0) = 1/K(0,0); invK(0,2) = -invK(2,2)*K(0,2)/K(0,0);
		invK(1,1) = 1/K(1,1); invK(1,2) = -invK(2,2)*K(1,2)/K(1,1);

		VEC3D dir;
		dir = invK * VEC3D(vpt[0],vpt[1],1.0);
		dir.Normalize();
		return dir;
	}

	VEC3D pixelOnPlane(const VEC2D &pixel, const VEC4D &plane, const CProxyCamera *pCam)
	{
		VEC3D pixel_homo(pixel[0],pixel[1],1.0);
		VEC4D pt_homo = pCam->unproject(pixel_homo);
		VEC4D intr_pt = pCam->intersect(pt_homo,plane);
		MyUtil::homo_normalize(intr_pt);
		return VEC3D(intr_pt[0],intr_pt[1],intr_pt[2]);
	}

	VEC3D calcCenter(const VEC3D *pts, int n)
	{
		VEC3D c(0,0,0);
		if (n<=0)
		{
			return c;
		}
		for (int i=0;i<n;i++)
		{
			c += pts[i];
		}
		c/=(float)n;
		return c;
	}

	inline void minmax(double &mn, double &mx, double v)
	{
		if (v < mn) mn = v;
		if (v > mx) mx = v;
	}


	void proj_pts_onto_axis(const VEC3D *pts, int n, const VEC3D axes[3], VEC3D &global_c, double hls[3])
	{
		VEC3D proj_p;
		proj_p.X() = axes[0].Dot(pts[0]);
		proj_p.Y() = axes[1].Dot(pts[0]);
		proj_p.Z() = axes[2].Dot(pts[0]);
		double minval[3],maxval[3];
		minval[0] = maxval[0] = proj_p.X();
		minval[1] = maxval[1] = proj_p.Y();
		minval[2] = maxval[2] = proj_p.Z();
		
		for(int i=1; i<n; i++)
		{
			proj_p.X() = axes[0].Dot(pts[i]);
			proj_p.Y() = axes[1].Dot(pts[i]);
			proj_p.Z() = axes[2].Dot(pts[i]);
			minmax(minval[0], maxval[0], proj_p.X());
			minmax(minval[1], maxval[1], proj_p.Y());
			minmax(minval[2], maxval[2], proj_p.Z());
		}

		// With the max and min data, determine the center point and dimensions
		// of the parent box.
		VEC3D c;
		c[0] = (minval[0] + maxval[0])*0.5;
		c[1] = (minval[1] + maxval[1])*0.5;
		c[2] = (minval[2] + maxval[2])*0.5;

		//get pT in global coordinates
		global_c = c[0]*axes[0] + c[1]*axes[1] + c[2]*axes[2];
		hls[0] = (maxval[0] - minval[0])*0.5;
		hls[1] = (maxval[1] - minval[1])*0.5;
		hls[2] = (maxval[2] - minval[2])*0.5;
	}


	TriMesh *clone_mesh(const TriMesh *pMesh)
	{
		TriMesh *pTgt = new TriMesh();
		pTgt->vertices.assign(pMesh->vertices.begin(),pMesh->vertices.end());
		pTgt->normals.assign(pMesh->normals.begin(),pMesh->normals.end());
		pTgt->faces.assign(pMesh->faces.begin(),pMesh->faces.end());
		pTgt->need_tstrips();
		return pTgt;
	}


	void build_plane_UV(const VEC3D &plane_normal, VEC3D &plane_U, VEC3D &plane_V)
	{
		if (fabs(plane_normal[2])>1e-3f)
		{
			plane_U[0] = -plane_normal[2]; plane_U[1] = 0; plane_U[2] = plane_normal[0]; 
		}else if (fabs(plane_normal[1])>1e-3f)
		{
			plane_U[0] = -plane_normal[1]; plane_U[1] = plane_normal[0]; plane_U[2] = 0; 
		}else
		{
			plane_U[0] = 0;	plane_U[1] = -plane_normal[2]; plane_U[2] = plane_normal[1];
		}
		plane_U.Normalize();
		plane_V = plane_normal.Cross(plane_U);
		plane_V.Normalize();
	}

	VEC2D getUV(const VEC3D &plane_pt, const VEC3D &U_dir, const VEC3D &V_dir, const VEC3D &pt)
	{
		VEC2D uv_para;
		VEC3D dir_3d = pt - plane_pt;
		uv_para[0] = dir_3d.Dot(U_dir);
		uv_para[1] = dir_3d.Dot(V_dir);
		return uv_para;
	}

	void removeDuplicatedPts(std::vector<VEC3D> &vPts)
	{
		for(int i=0;i<(int)vPts.size()-1;i++)
		{
			for (int j=i+1;j<(int)vPts.size();j++)
			{
				if ((vPts[j]-vPts[i]).SquaredLength()<1e-8f)
				{
					//throw
					vPts[j] = vPts.back();
					vPts.pop_back();
					j--;	//re-check
				}
			}
		}
	}

	double signed_dist_pt_2_plane(const VEC4D &plane, const VEC3D pt)
	{
		return plane.Dot(VEC4D(pt[0],pt[1],pt[2],1));
	}

	void adjust_plane_pts(VEC3D *pts, int n, const VEC3D &new_plane_normal)
	{
		if (n<=0)
		{
			return;
		}
		VEC3D c(0,0,0);
		for (int i=0;i<n;i++)
		{
			c += pts[i];
		}
		c /= (double)n;
		VEC4D plane(new_plane_normal[0],new_plane_normal[1],new_plane_normal[2],-new_plane_normal.Dot(c));
		for (int i=0;i<n;i++)
		{
			pts[i] = proj_pt_2_plane(pts[i],plane);
		}
	}

	VEC3D intersect_line_plane(const VEC3D &pt1, const VEC3D &pt2, const VEC4D& abcd)
	{
		VEC4D center(pt1[0],pt1[1],pt1[2],1.f);
		VEC4D coord(pt2[0],pt2[1],pt2[2],1.f);
		VEC4D ray = center - coord;

		const double A = coord.Dot(abcd);
		const double B = ray.Dot(abcd);

		if (fabs(B)<1e-8)
			return VEC3D(0.0f, 0.0f, 0.0f);
		
		VEC4D intr_homo = coord - A / B * ray;
		homo_normalize(intr_homo);
		return VEC3D(intr_homo[0],intr_homo[1],intr_homo[2]);
	}

	void render3DArrow(const VEC3F& world_pos,const VEC3F& axis,float length,float width)
	{
		glDisable(GL_LIGHTING);
		glLineWidth(width);
		glColor3f(0.0f,0.0f,1.0f);
		glBegin(GL_LINES);
		glVertex3f(world_pos[0],world_pos[1],world_pos[2]);
		glVertex3f(world_pos[0]+axis[0]*length,world_pos[1]+axis[1]*length,world_pos[2]+axis[2]*length);
		glEnd();
	}
}