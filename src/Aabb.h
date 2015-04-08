#ifndef hohehohe2_Aabb_H
#define hohehohe2_Aabb_H

#include "Point.h"

namespace hohehohe2
{

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//! Axis aligned bounding box.
struct Aabb
{

	//! (x min, y min, z min) of the AABB.
	Point m_bboxMin;

	//! (x max, y max, z max) of the AABB.
	Point m_bboxMax;

	//! Constructor.
	Aabb(){}

	//! Constructor.
	Aabb(Point bboxMin, Point bboxMax) : m_bboxMin(bboxMin), m_bboxMax(bboxMax){}

	//! Test if two bounding boxes overwrap.
	inline bool isOverwrap(const Aabb& other) const
	{
		return ! (
			m_bboxMin(0) > other.m_bboxMax(0) ||
			m_bboxMin(1) > other.m_bboxMax(1) ||
			m_bboxMin(2) > other.m_bboxMax(2) ||
			other.m_bboxMin(0) > m_bboxMax(0) ||
			other.m_bboxMin(1) > m_bboxMax(1) ||
			other.m_bboxMin(2) > m_bboxMax(2)
			);
	}

};

}

#endif
