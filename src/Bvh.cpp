#include "Bvh.h"
#include <float.h>
#include <ostream>
#include <list>
#include <algorithm>
#include "CellCodeCalculator.h"

using namespace hohehohe2;


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void Bvh::construct(const std::vector < Point > & vertices, const std::vector < unsigned int > & faces)
{
	unsigned int numFaces = (unsigned int)faces.size() / 3;

	m_vertices = &vertices;
	m_leafs.resize(numFaces);

	//Numbver of internal nodes are exactly numFaces - 1.
	//See http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-iii-tree-construction-gpu/.
	m_internals.resize(numFaces - 1);

	//Every triangle center position and its AAbb is needed to calculate triangle moton codes.
	std::vector < Point > centers(numFaces);
	Point bboxMin;
	Point bboxMax;

	//Fill vertex data to the leafs and calculate center/Aabb.
	for (unsigned int i = 0; i < faces.size(); i += 3)
	{
		m_leafs[i / 3] = BvhNodeLeaf(faces[i], faces[i + 1], faces[i + 2]);

		Point vPos0 = vertices[faces[i]];
		Point vPos1 = vertices[faces[i + 1]];
		Point vPos2 = vertices[faces[i + 2]];
		Point center = (vPos0 + vPos1 + vPos2) / 3;
		centers[i / 3] = center;
		bboxMin = bboxMin.cwiseMin(center);
		bboxMax = bboxMax.cwiseMax(center);
	}

	//Calculate the morton codes of triangle faces.
	CellCodeCalculator ccCalculator; //A utility class to calculate the morton codes of triangle faces.
	ccCalculator.reset(Aabb(bboxMin, bboxMax));
	for (unsigned int i = 0; i < faces.size() / 3; ++i)
	{
		m_leafs[i].m_mortonCode = ccCalculator.getCode32(centers[i].x(), centers[i].y(), centers[i].z());
	}

	//Sort by morton code ascending order. BvhNodeLeaf's < operator is defined so that it uses the morton code.
	std::sort(m_leafs.begin(), m_leafs.end());

	//Construct the Bvh hierarchy recursively.
	if (numFaces == 1)
	{
		m_root = &m_leafs[0];
		return;
	}
	else
	{
		m_root = &m_internals[0];
	}

	unsigned int nextAvailableIntenral = 1;
	construct_(*static_cast < BvhNodeInternal* > (m_root), 0, numFaces - 1, nextAvailableIntenral);

	update();
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void Bvh::update()
{
	for (unsigned int i = 0; i < m_internals.size(); ++i)
	{
		m_internals[i].m_bbox.m_bboxMin.setConstant(FLT_MAX);
		m_internals[i].m_bbox.m_bboxMax.setConstant(-FLT_MAX);
	}

	//Update leaf Aabb.
	for (unsigned int i = 0; i < m_leafs.size(); ++i)
	{
		m_leafs[i].update(*m_vertices);

	}

	//Update internal nodes. Reverse order since higher index node is closer to the leaf in the BVH.
	if (m_internals.size() == 0)
	{
		return;
	}

	for (int i = (int)m_internals.size() - 1; i >= 0; --i)
	{
		m_internals[i].update();
	}

}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void Bvh::queryAabbOverwrap(std::vector < const BvhNodeLeaf* > & result, const Aabb& testBbox) const
{
	std::list < BvhNode* > childQueue;
	childQueue.push_back(m_root);

	while (childQueue.size())
	{
		BvhNode* child = childQueue.back();
		childQueue.pop_back();
		if (child->m_bbox.isOverwrap(testBbox))
		{
			if (child->m_isLeaf)
			{
				result.push_back(static_cast < const BvhNodeLeaf* > (child));
			}
			else
			{
				const BvhNodeInternal* asInternal = static_cast < const BvhNodeInternal* > (child);
				childQueue.push_back(asInternal->m_rightChild);
				childQueue.push_back(asInternal->m_leftChild);
			}
		}
	}
}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void Bvh::print(std::ostream& os) const
{

	if ( ! m_root)
	{
		return;
	}

	std::list < BvhNode* > childQueue;
	childQueue.push_back(m_root);

	while (childQueue.size())
	{
		BvhNode* child = childQueue.back();
		childQueue.pop_back();

		os << "---- " << child << " " << ((child->m_isLeaf)? "L " : "I ") << "min=" << child->m_bbox.m_bboxMin.transpose() << " max=" << child->m_bbox.m_bboxMax.transpose() << " ";

		if (child->m_isLeaf)
		{
			const BvhNodeLeaf* asLeaf = static_cast < const BvhNodeLeaf* > (child);
			os << asLeaf->m_vertexIds[0] << " " << asLeaf->m_vertexIds[1] << " " << asLeaf->m_vertexIds[2] << std::endl;
		}
		else
		{
			const BvhNodeInternal* asInternal = static_cast < const BvhNodeInternal* > (child);
			os << asInternal->m_leftChild << " " << asInternal->m_rightChild << std::endl;
			childQueue.push_back(asInternal->m_rightChild);
			childQueue.push_back(asInternal->m_leftChild);
		}

	}

	os << std::endl;

}


//-------------------------------------------------------------------
//-------------------------------------------------------------------
void Bvh::construct_(BvhNodeInternal& internalNode, unsigned int left, unsigned int right, unsigned int& nextAvailableIntenral)
{
	BvhNodeLeaf& leftLeaf = m_leafs[left];
	BvhNodeLeaf& rightLeaf = m_leafs[right];

	unsigned int mid;
	if (leftLeaf.m_mortonCode == rightLeaf.m_mortonCode)
	{
		mid = (left + right) / 2;
	}
	else
	{
		unsigned int rightZeroBits = BitOperations::countLeadingZeros32(rightLeaf.m_mortonCode);
		mid = left + 1;
		while (BitOperations::countLeadingZeros32(m_leafs[mid].m_mortonCode) != rightZeroBits)
		{
			++mid;
		}
		--mid;

		//Now mid points to the last node which morton code's clz(count leading zeros) is different from the right node's clz.
	}

	if (left == mid)
	{
		internalNode.m_leftChild = &m_leafs[left];
	}
	else
	{
		BvhNodeInternal* child = &m_internals[nextAvailableIntenral++];
		internalNode.m_leftChild = child;
		construct_(*child, left, mid, nextAvailableIntenral);
	}
	if (right == mid + 1)
	{
		internalNode.m_rightChild = &m_leafs[right];
	}
	else
	{
		BvhNodeInternal* child = &m_internals[nextAvailableIntenral++];
		internalNode.m_rightChild = child;
		construct_(*child, mid + 1, right, nextAvailableIntenral);
	}

}
