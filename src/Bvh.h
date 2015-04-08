#ifndef hohehohe2_Bvh_H
#define hohehohe2_Bvh_H

#include <vector>
#include "BvhNode.h"

namespace hohehohe2
{

    //! Simple BVH node.
    class Bvh
    {

	public:

        //! Constructor.
		Bvh() : m_root(NULL), m_vertices(NULL){}

        //! Construct the bvh, which may take some time. It calls update().
		/**
		Don't modify vertices while using this object.

		@param vertices Vertex positions.
		@param faces Container of triangle faces. A single face has three vertex ids.
		**/
		void construct(const std::vector < Point > & vertices, const std::vector < unsigned int > & faces);

        //! Update the bvh's bounding box.
		void update();

        //! Bvh query. This method is thread safe.
		/**
		@param testBbox Bounding box to test.
		@param result Leaf nodes that overwraps the testBbox.
		**/
		void queryAabbOverwrap(std::vector < const BvhNodeLeaf* > & result, const Aabb& testBbox) const;

		///Print the BVH info.
		void print(std::ostream& os) const;

	private:

		//! The root node.
		BvhNode* m_root;

		//! Leaf nodes.
		std::vector < BvhNodeLeaf > m_leafs;

		//! Internal nodes.
		std::vector < BvhNodeInternal > m_internals;

		//! Vertex positions.
		const std::vector < Point > * m_vertices;

	private:

		void construct_(BvhNodeInternal& internalNode, unsigned int left, unsigned int right, unsigned int& nextAvailableIntenral);

	};


}

#endif
