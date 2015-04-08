#ifndef hohehohe2_Point_H
#define hohehohe2_Point_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace hohehohe2
{

	//! Point type for this application.
	typedef Eigen::Vector3f Point;

	//! Point object indicating no point is found.
	static Point POINT_NOT_FOUND;

}

#endif
