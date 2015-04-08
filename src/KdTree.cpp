#include "KdTree.h"
#include <ostream>
#include <algorithm>

using namespace hohehohe2;


//-------------------------------------------------------------------
//-------------------------------------------------------------------
//Function for comparing Points based on a single component (x:index=0, y:index=1, z:index=2).
template < unsigned int index >
static bool componentComp_(const Point* left, const Point* right)
{
    return (*left)(index) < (*right)(index);
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void KdTree::construct(const std::vector < Point > & points)
{
	clear();
	PointPtrs_ pointPtrs(points.size());
	const Point* start = points.data();
	const Point** startPtr = pointPtrs.data();
	size_t size = points.size();
    for (size_t index = 0; index < size; ++index)
    {
		*startPtr++ = start++;
	}

	constructTree_(pointPtrs.begin(), pointPtrs.end());
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void KdTree::clear()
{
	m_tree.clear();
	m_buckets.clear();
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
Point KdTree::query(const Point& queryPoint, float maxDist, float eps) const
{
	assert(eps >= 0.0f && "eps must be positive");
	const Point* result = NULL; //Will pointer to the nearest point found in m_buckets. It is updated whenever a nearer point is found during search.
	const KdTreeNode* root = getRoot_();
	float D = maxDist * maxDist;
    find1NN_(result, queryPoint, root, Point::Zero(), 0.0f, D, eps);

	return (result == NULL)? POINT_NOT_FOUND : *result;

}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void KdTree::printTree(std::ostream& os) const
{
	for (unsigned int i = 0; i < m_tree.size(); ++i)
	{
		bool isLeaf = m_tree[i].isLeaf();
		os << i << ": " << ((isLeaf)? "Leaf " : "Internal ");

		if (isLeaf)
		{
			const KdTreeNodeLeaf* leaf = reinterpret_cast < const KdTreeNodeLeaf* > (&m_tree[i]);
			os << "bucketIndex=" << leaf->getBucketIndex()
			   << " bucketSize=" << leaf->getBucketSize() << std::endl;
		}
		else
		{
			const KdTreeNodeInternal* internal = reinterpret_cast < const KdTreeNodeInternal* > (&m_tree[i]);
			os << "axis=" << internal->getAxis()
				<< " coordinate=" << internal->getSplitCoordinate()
				<< " left=" << internal->getLeftChild() - m_tree.data()
				<< " right=" << internal->getRightChild() - m_tree.data() << std::endl;
		}
	}

	os << std::endl;

	for (unsigned int i = 0; i < m_buckets.size(); ++i)
	{
		os << m_buckets[i].transpose() << std::endl;
	}

}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void KdTree::find1NN_(const Point*& result, const Point& p, const KdTreeNode* N, Point a, float d, float& D, float eps) const
{
    if (N->isLeaf())
    {
		//Check every Point in the bucket of this leaf one by one, and find the closest.
        const KdTreeNodeLeaf* node = static_cast < const KdTreeNodeLeaf* > (N);
        const unsigned int bucketIndex = node->getBucketIndex();
        const unsigned int bucketSize = node->getBucketSize();
        for (unsigned int i = bucketIndex; i < bucketIndex + bucketSize; ++i)
        {
			const float squaredDistance = (m_buckets[i] - p).squaredNorm();
            if (squaredDistance < D)
            {
                D = squaredDistance;
                result = &m_buckets[i];
            }
        }
    }
    else
    {
		const KdTreeNodeInternal* node = static_cast < const KdTreeNodeInternal* > (N);
		const KdTreeNode* N1; //Near child.
		const KdTreeNode* N2; //Far child.
		float pToSplitPlaneSignedDistance = p(node->getAxis()) - node->getSplitCoordinate();
		if (pToSplitPlaneSignedDistance > 0)
		{
			//p is on the plus side of the area, i.e. closer to the right node.
			N1 = node->getRightChild();
			N2 = node->getLeftChild();
		}
		else
		{
			//p is on the minus side of the area, i.e. closer to the left node.
			N1 = node->getLeftChild();
			N2 = node->getRightChild();
		}

		find1NN_(result, p, N1, a, d, D, eps);

		float u = pToSplitPlaneSignedDistance * pToSplitPlaneSignedDistance;
		d += - a(node->getAxis()) + u;
		a(node->getAxis()) = u;

		if (d < D + eps) ////If N2 and a sphere with radius=D, center=p overwrap.
		{
			find1NN_(result, p, N2, a, d, D, eps);
		}
	}
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
unsigned int KdTree::constructTree_(PointPtrs_::iterator begin, PointPtrs_::iterator end)
{
	unsigned int size = (unsigned int)(end - begin);
	if (size <= m_bucketSize)
	{
		//----No need to expand the tree anymore. Make a leaf node here.

		//Append leaf node.
		m_tree.push_back(KdTreeNode(true));
		//Since the sizeof(KdTreeNode) and sizeof(KdTreeNodeLeaf) are the same, you can reinterpret them.
		KdTreeNodeLeaf* newLeaf = reinterpret_cast < KdTreeNodeLeaf* > (&m_tree.back());
		newLeaf->setBucketIndex((unsigned int)m_buckets.size());
		newLeaf->setBucketSize(size);

		//Copy the points to the bucket.
		for (PointPtrs_::iterator it = begin; it != end; ++it)
		{
			m_buckets.push_back(**it);
		}

		return (unsigned int)m_tree.size() - 1;
	}
	else
	{
		//----Expand the tree. Make an internal node here.

		//Append internal node.
		m_tree.push_back(KdTreeNode(false));
		KdTreeNodeInternal* newInternal = reinterpret_cast < KdTreeNodeInternal* > (&m_tree.back());
		unsigned int newInternalIndex = (unsigned int)m_tree.size() - 1;

		//Find the split axis.
		KdTreeNodeInternal::Axis splitAxis = findSplitAxis_(begin, end);
		newInternal->setAxis(splitAxis);

		//Construct children...

		//Split the points into two groups.
		//NOTE: std::nth_element is an STL implementation of selection http://en.wikipedia.org/wiki/Selection_algorithm.
		PointPtrs_::iterator medianPtr = begin + size / 2;
		if (splitAxis == KdTreeNodeInternal::AXIS_X)
		{
			std::nth_element(begin, medianPtr, end, componentComp_ < 0 > );
		}
		else if (splitAxis == KdTreeNodeInternal::AXIS_Y)
		{
			std::nth_element(begin, medianPtr, end, componentComp_ < 1 > );
		}
		else if (splitAxis == KdTreeNodeInternal::AXIS_Z)
		{
			std::nth_element(begin, medianPtr, end, componentComp_ < 2 > );
		}

		newInternal->setSplitCoordinate((**medianPtr)(splitAxis));

		//Construct left tree.
		constructTree_(begin, medianPtr);
		unsigned int rightChildIndex = constructTree_(medianPtr, end); //Median point goes to the right child.

		//Construct right tree.
		newInternal = reinterpret_cast < KdTreeNodeInternal* > (&m_tree[newInternalIndex]); //m_tree may be copied due to the reallocation. Get the internal object again.
		newInternal->setRightChildOffset(rightChildIndex - newInternalIndex);

		return newInternalIndex;

	}
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
KdTreeNodeInternal::Axis KdTree::findSplitAxis_(PointPtrs_::iterator begin, PointPtrs_::iterator end)
{
	Point min;
	Point max;
	for (PointPtrs_::iterator it = begin; it != end; ++it)
	{
		min = min.cwiseMin(**it);
		max = max.cwiseMax(**it);
	}

	Point cwiseLength = max - min;
	if (cwiseLength.x() < cwiseLength.y())
	{
		return (cwiseLength.y() < cwiseLength.z())? KdTreeNodeInternal::AXIS_Z : KdTreeNodeInternal::AXIS_Y;
	}
	else
	{
		return (cwiseLength.x() < cwiseLength.z())? KdTreeNodeInternal::AXIS_Z : KdTreeNodeInternal::AXIS_X;
	}
}
