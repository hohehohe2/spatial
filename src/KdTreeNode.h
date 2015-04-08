#ifndef hohehohe2_KdTreeNode_H
#define hohehohe2_KdTreeNode_H

#include <assert.h>

namespace hohehohe2
{

    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Base class of the kd-tree node. Use isLeaf() to see if leaf node or internal node.
    /**
       The node structure closely follows

       Accelerating kd-tree searches for all k-nearest neighbours
       Bruce Merry, James Gain and Patrick Marais EUROGRAPHICS 2013

       See the paper for more details.

       It has the followinng limitation;
       - Maximum number of nodes and bucket size is 1073741823 (30 bits).
       - Maximum number of objects is 4294967295(32 bits).
    **/
    class KdTreeNode
    {
    public:

		//! Constructor.
		KdTreeNode(bool isLeaf) {m_data = (isLeaf)? 3 : 0;}

		//! Returns true if the node is leaf.
        inline bool isLeaf() const {return (m_data & 3) == 3;}

    protected:

        //! 4 byte data. It has information on isLeaf, rightChildOffset(if internal) or bucketSize(if leaf).
        unsigned int m_data;

		union
		{
			//! Coordinate of the split plane for internal node.
			float m_splitCoordinate;

			//! Index in the bucket array for leaf node.
			unsigned int m_bucketIndex;
		};
    };


    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Kd-tree internal node.
    class KdTreeNodeInternal : public KdTreeNode
    {
    public:

		///Split plane axis. You can cast it to int to get the index of Point, e.g. when p is a point and N is a KdTreeNodeInternal instance, and when N.getAxis() is AXIS_Y,
		/// p(N.getAxis()) means the p's y coordinate value.
        enum Axis
        {
            AXIS_X = 0,
            AXIS_Y,
            AXIS_Z,
        };

        //DO NOT ADD MEMBERS!

    public:

        //! Set the split coordinate.
        inline void setSplitCoordinate(float coord) {m_splitCoordinate = coord;}

        //! Get the split coordinate.
        inline float getSplitCoordinate() const {return m_splitCoordinate;}

        //! Get the split axis.
        inline void setAxis(Axis axis) {m_data |= axis;}

        //! Get the split axis.
        inline Axis getAxis() const {return (Axis)(m_data & 3);}

        //! Set the offset to the right child. Child node represents the area on the split plane's plus side.
        inline void setRightChildOffset(unsigned int offset) {m_data = (offset << 2) + (m_data & 3);}

        //! Get the left child node. Child node represents the area on the split plane's minus side.
        inline const KdTreeNode* getLeftChild() const {return this + 1;}

        //! Get the right child node. Child node represents the area on the split plane's plus side.
        inline const KdTreeNode* getRightChild() const {return this + (m_data >> 2);}

    };


    //-------------------------------------------------------------------
    //-------------------------------------------------------------------
    //! Kd-tree leaf node.
    class KdTreeNodeLeaf : public KdTreeNode
    {
    public:

		//! Split plane axis.
        enum Axis
        {
            AXIS_X = 1,
            AXIS_Y,
            AXIS_Z,
        };

        //DO NOT ADD MEMBERS!

    public:

        //! Set the start index of the bucket in the buckets array.
        inline void setBucketIndex(unsigned int index) {m_bucketIndex = index;}

        //! Get the start index of the bucket in the buckets array.
        inline unsigned int getBucketIndex() const {return m_bucketIndex;}

        //! Set the bucket size.
        inline void setBucketSize(unsigned int size) {m_data = (size << 2) + 3;} //3 indicates leaf.

        //! Get the bucket size.
        inline unsigned int getBucketSize() const {return m_data >> 2;}

    };
}

#endif
