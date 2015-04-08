#ifndef hohe_CellCodeCalculator_H
#define hohe_CellCodeCalculator_H

#include "BitOperations.h"
#include "Aabb.h"


namespace hohehohe2
{

//-------------------------------------------------------------------
//-------------------------------------------------------------------
///Calculate the morton code of a point.
class CellCodeCalculator
{

	Point m_bboxMin;
	Point m_cellSize;

public:

	//! Reset the calculator.
	void reset(const Aabb& bbox)
	{
		m_bboxMin = bbox.m_bboxMin;
		m_cellSize = (bbox.m_bboxMax - m_bboxMin) / 1023; // Not 1024 so that max cell id will be less than 1024.
	}

	//! Position -> morton code of the cell containing the position.
	unsigned int getCode32(float x, float y, float z) const
	{
		const unsigned int cellIdX = static_cast < unsigned int > ((x - m_bboxMin.x()) / m_cellSize.x());
		const unsigned int cellIdY = static_cast < unsigned int > ((y - m_bboxMin.y()) / m_cellSize.y());
		const unsigned int cellIdZ = static_cast < unsigned int > ((z - m_bboxMin.z()) / m_cellSize.z());

		return BitOperations::calcMortonCode32(cellIdX, cellIdY, cellIdZ);
	}

};


}

#endif
