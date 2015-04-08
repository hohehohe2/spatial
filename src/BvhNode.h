#ifndef hohehohe2_BvhNode_H
#define hohehohe2_BvhNode_H

#include <assert.h>
#include "Point.h"
#include "Aabb.h"

namespace hohehohe2
{
	struct BvhNodeInternal;

    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Base class of the Bvh node. Use isLeaf() to see if leaf node or internal node.
    struct BvhNode
    {

		//! Bounding box of the node.
		Aabb m_bbox;

		//! True if leaf node.
		bool m_isLeaf;

		//! Constructor.
		BvhNode(){}

		//! Constructor.
		BvhNode(bool isLeaf) : m_isLeaf(isLeaf){}
	};


    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Bvh internal node.
    struct BvhNodeInternal : public BvhNode
    {

		//! Pointer to the left child.
		BvhNode* m_leftChild;

		//! Pointer to the right child.
		BvhNode* m_rightChild;

		//! Constructor.
		BvhNodeInternal() : BvhNode(false){}

		//! Constructor.
		BvhNodeInternal(BvhNode* leftChild, BvhNode* rightChild) : BvhNode(false), m_leftChild(leftChild), m_rightChild(rightChild) {}


		//! Update Bounding box.
		void update()
		{
			m_bbox.m_bboxMin = m_leftChild->m_bbox.m_bboxMin.cwiseMin(m_rightChild->m_bbox.m_bboxMin);
			m_bbox.m_bboxMax = m_leftChild->m_bbox.m_bboxMax.cwiseMax(m_rightChild->m_bbox.m_bboxMax);
		}
};


    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Bvh leaf node.
    struct BvhNodeLeaf : public BvhNode
    {

		//! Vertex ids.
		unsigned int m_vertexIds[3];

		//! Constructor.
		BvhNodeLeaf() : BvhNode(true){}

		//! Constructor.
		BvhNodeLeaf(unsigned int vtxId0, unsigned int vtxId1, unsigned int vtxId2) : BvhNode(true)
		{
			m_vertexIds[0] = vtxId0;
			m_vertexIds[1] = vtxId1;
			m_vertexIds[2] = vtxId2;
		}

		//! Morton code of the triangle this leaf represents.
		unsigned int m_mortonCode;

		//! Update Bounding box.
		void update(const std::vector < Point > & vertices)
		{
			m_bbox.m_bboxMin = vertices[m_vertexIds[0]].cwiseMin(vertices[m_vertexIds[1]]).cwiseMin(vertices[m_vertexIds[2]]);
			m_bbox.m_bboxMax = vertices[m_vertexIds[0]].cwiseMax(vertices[m_vertexIds[1]]).cwiseMax(vertices[m_vertexIds[2]]);
		}

		//! < operator uses morton code.
		bool operator < (const BvhNodeLeaf& rhs) const{return m_mortonCode < rhs.m_mortonCode;}
	};
}

#endif
