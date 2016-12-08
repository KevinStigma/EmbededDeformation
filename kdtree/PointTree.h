#pragma once
#include "nanoflann.hpp"
#include "../ldpMat/ldp_basic_vec.h"
namespace ldp
{

	namespace kdtree
	{
		template <typename T>
		class PointTree
		{
		public:
			class Point
			{
			public:
				int idx;
				ldp::ldp_basic_vec3<T> p;

				Point() :idx(-1), p(0){}
				Point(const Point& r) :idx(r.idx), p(r.p){}
				Point(ldp::ldp_basic_vec3<T>& v, int i = -1) : p(v), idx(i){}
			};
		private:
			template <typename T>
			struct PointCloud
			{
				std::vector<Point>  pts;

				// Must return the number of data points
				inline size_t kdtree_get_point_count() const { return pts.size(); }

				// Returns the distance between the vector "p1[0:size-1]" 
				// and the data point with index "idx_p2" stored in the class:
				inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t size) const
				{
					const T d0 = p1[0] - pts[idx_p2].p[0];
					const T d1 = p1[1] - pts[idx_p2].p[1];
					const T d2 = p1[2] - pts[idx_p2].p[2];
					return d0*d0 + d1*d1 + d2*d2;
				}

				inline T kdtree_get_pt(const size_t idx, int dim) const
				{
					return pts[idx].p[dim];
				}

				// Optional bounding-box computation: return false to default to a standard bbox computation loop.
				//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
				//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
				template <class BBOX>
				bool kdtree_get_bbox(BBOX &bb) const { return false; }

			};
			typedef nanoflann::KDTreeSingleIndexAdaptor<
				nanoflann::L2_Simple_Adaptor<T, PointCloud<T> >,
				PointCloud<T>,
				3 /* dim */
			> my_kd_tree_simple_t;

			PointCloud<T> m_points;
			my_kd_tree_simple_t* m_tree;
		public:
			PointTree()
			{
				m_tree = 0;
			}
			~PointTree()
			{
				clear();
			}
			void clear()
			{
				if (m_tree)
					delete m_tree;
				m_tree = 0;
				m_points.pts.clear();
			}

			void build(const std::vector<Point>& points)
			{
				clear();
				m_points.pts.assign(points.begin(), points.end());
				m_tree = new my_kd_tree_simple_t(3 /*dim*/, m_points,
					nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
				m_tree->buildIndex();
			}

			bool isValid()
			{
				return m_points.pts.size() != 0;
			}

			Point nearestPoint(Point p, T& dist)const
			{
				size_t index;
				T out_dist_sqr = T(0);
				nanoflann::KNNResultSet<T> resultSet(1);
				resultSet.init(&index, &out_dist_sqr);
				m_tree->findNeighbors(resultSet, p.p.ptr(), nanoflann::SearchParams(10));

				dist = sqrt(out_dist_sqr);
				return m_points.pts[index];
			}

			// k is the size of pts
			void kNearestPoints(Point p, std::vector<Point>& pts)const
			{
				std::vector<size_t> index(pts.size());
				std::vector<T> out_dist_sqr(pts.size());
				nanoflann::KNNResultSet<T> resultSet(pts.size());
				resultSet.init(index.data(), out_dist_sqr.data());
				m_tree->findNeighbors(resultSet, p.p.ptr(), nanoflann::SearchParams(10));

				for (int i = 0; i < pts.size(); i++)
					pts[i] = m_points.pts[index[i]];
			}

			void pointInSphere(Point c, T radius, std::vector<std::pair<size_t, T>>& indices_dists)
			{
				nanoflann::RadiusResultSet<T> resultSet(radius, indices_dists);
				m_tree->findNeighbors(resultSet, c.p.ptr(), nanoflann::SearchParams(10));
			}
		};

		template <typename T>
		class PointNormalTree
		{
		public:
			class Point
			{
			public:
				int idx;
				ldp::ldp_basic_vec<T, 6> p;

				Point() :idx(-1), p(T(0)){}
				Point(const Point& r) :idx(r.idx), p(r.p){}
				Point(const ldp::ldp_basic_vec3<T>& v, const ldp::ldp_basic_vec3<T>& n, int i = -1)
				{
					p = ldp::ldp_basic_vec<T, 6>(v, n);
					idx = i;
				}
				void set(const ldp::ldp_basic_vec3<T>& v, const ldp::ldp_basic_vec3<T>& n)
				{
					p = ldp::ldp_basic_vec<T, 6>(v, n);
				}
				ldp::ldp_basic_vec3<T> getPos()const { return ldp::ldp_basic_vec3<T>(p[0], p[1], p[2]); }
				ldp::ldp_basic_vec3<T> getNormal()const { return ldp::ldp_basic_vec3<T>(p[3], p[4], p[5]); }
			};
		private:
			template <typename T>
			struct PointCloud
			{
				std::vector<Point>  pts;
				T w_normal;

				// Must return the number of data points
				inline size_t kdtree_get_point_count() const { return pts.size(); }

				// Returns the distance between the vector "p1[0:size-1]" 
				// and the data point with index "idx_p2" stored in the class:
				inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t size) const
				{
					const T d0 = p1[0] - pts[idx_p2].p[0];
					const T d1 = p1[1] - pts[idx_p2].p[1];
					const T d2 = p1[2] - pts[idx_p2].p[2];
					const T d3 = p1[3] - pts[idx_p2].p[3];
					const T d4 = p1[4] - pts[idx_p2].p[4];
					const T d5 = p1[5] - pts[idx_p2].p[5];
					return d0*d0 + d1*d1 + d2*d2 + w_normal * (d3*d3 + d4*d4 + d5*d5);
				}

				inline T kdtree_get_pt(const size_t idx, int dim) const
				{
					return pts[idx].p[dim];
				}

				// Optional bounding-box computation: return false to default to a standard bbox computation loop.
				//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
				//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
				template <class BBOX>
				bool kdtree_get_bbox(BBOX &bb) const { return false; }

			};
			typedef nanoflann::KDTreeSingleIndexAdaptor<
				nanoflann::L2_Simple_Adaptor<T, PointCloud<T> >,
				PointCloud<T>,
				3 /* dim */
			> my_kd_tree_simple_t;

			PointCloud<T> m_points;
			my_kd_tree_simple_t* m_tree;
		public:
			PointNormalTree()
			{
				m_tree = 0;
			}
			~PointNormalTree()
			{
				clear();
			}
			void clear()
			{
				if (m_tree)
					delete m_tree;
				m_tree = 0;
				m_points.pts.clear();
			}

			void build(const std::vector<Point>& points, T weight_normals = T(1))
			{
				clear();
				m_points.pts.assign(points.begin(), points.end());
				m_points.w_normal = weight_normals;
				m_tree = new my_kd_tree_simple_t(6 /*dim*/, m_points,
					nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
				m_tree->buildIndex();
			}

			bool isValid()
			{
				return m_points.pts.size() != 0;
			}

			Point nearestPoint(Point p, T& dist)const
			{
				size_t index;
				T out_dist_sqr = T(0);
				nanoflann::KNNResultSet<T> resultSet(1);
				resultSet.init(&index, &out_dist_sqr);
				m_tree->findNeighbors(resultSet, p.p.ptr(), nanoflann::SearchParams(10));

				dist = sqrt(out_dist_sqr);
				return m_points.pts[index];
			}

			// k is the size of pts
			void kNearestPoints(Point p, std::vector<Point>& pts)const
			{
				std::vector<size_t> index(pts.size());
				std::vector<T> out_dist_sqr(pts.size());
				nanoflann::KNNResultSet<T> resultSet(pts.size());
				resultSet.init(index.data(), out_dist_sqr.data());
				m_tree->findNeighbors(resultSet, p.p.ptr(), nanoflann::SearchParams(10));

				for (int i = 0; i < pts.size(); i++)
					pts[i] = m_points.pts[index[i]];
			}

			void pointInSphere(Point c, T radius, std::vector<std::pair<size_t, T>>& indices_dists)
			{
				nanoflann::RadiusResultSet<T> resultSet(radius, indices_dists);
				m_tree->findNeighbors(resultSet, c.p.ptr(), nanoflann::SearchParams(10));
			}
		};
	}
}