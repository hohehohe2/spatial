#ifndef hohehohe2_KdTree_H
#define hohehohe2_KdTree_H

#include <ostream>
#include <vector>
#include "Point.h"
#include "KdTreeNode.h"

namespace hohehohe2
{

    //! Kd-tree node.
    /**
       Based on FindKNN, see Algorithm 1 in

       Accelerating kd-tree searches for all k-nearest neighbours
       Bruce Merry, James Gain and Patrick Marais EUROGRAPHICS 2013

       See the paper for more details.
    **/
    class KdTree
    {

	public:

        //! Constructor.
		/**
		@param bucketSize Bucket size (max number of points each leaf node can have).
		**/
		KdTree(unsigned int bucketSize=24) : m_bucketSize(bucketSize){}

        //! Construct the tree which may take some time.
		/**
		@param points Container of points to be searched.
		**/
		void construct(const std::vector < Point > & points);

		//! Clear the tree.
		void clear();

        //! Kd-tree query.
        //! This method is thread safe.
		/**
		Due to floating point rounding error, Some points may miss being detected even
		when the distance from the query point is smaller than maxDist.
		Depending on the use case, you may want to set the error bound parameter eps to some small number.

		@param queryPoint Point to query the nearest neighbor.
		@param maxDist Max search distance. Points outside this distance will not be detected as nearest.
		@param eps Error bound.
		@retval Nearest point. If not found within maxDist, it returns a point which equals to POINT_NOT_FOUND.
		**/
		Point query(const Point& queryPoint, float maxDist, float eps=0.0f) const;

		///Print the tree info.
		void printTree(std::ostream& os) const;

    private:

        //Kd-tree.
        std::vector < KdTreeNode >  m_tree;

        //NOTE: It can be an array of uint id to be versatile.
        //NOTE: Here we store points directory for efficiency.
        //! Buckets array. KdTreeNodeLeaf::getBucketIndex() returns an index to this array.
        std::vector < Point >  m_buckets;

		//! Bucket size.
		unsigned int m_bucketSize;

		typedef std::vector < const Point* > PointPtrs_;

	private:

        //! Get the root node.
        KdTreeNode* getRoot_(){assert(m_tree.size() && "Tree size is zero."); return m_tree.data();}

		//! Get the root node.
		const KdTreeNode* getRoot_() const{assert(m_tree.size() && "Tree size is zero."); return m_tree.data();}

        //! Query implementation. (find1NN stands for 'find 1 nearest neighbor', i.e. closest neighbor).
        /**
		Argument naming is compatible with Algorithm 1 of the paper.
		@param p Query point.
		@param N Subtree root node
		@param a Per-axis squared distances
		@param d sigma axis: distance to the region
		@param D Squared distance to the nearest region.
		@param eps error bound.
        **/
        void find1NN_(const Point*& result, const Point& p, const KdTreeNode* N, Point a, float d, float& D, float eps) const;

		//! Construct the tree recursively. Returns the index of the newly created node in m_tree.
		unsigned int constructTree_(PointPtrs_::iterator begin, PointPtrs_::iterator end);

		//! Find the split axis of a node by taking the min/max of each component.
		KdTreeNodeInternal::Axis findSplitAxis_(PointPtrs_::iterator begin, PointPtrs_::iterator end);

	};

}

#endif
