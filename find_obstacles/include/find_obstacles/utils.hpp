#ifndef UTILS_HPP
#define UTILS_HPP



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <vector>

#include <find_obstacles/core.hpp>



constexpr float get_polar_distance(const pcl::PointXYZI& point)
{
    return static_cast<float>(
        sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2))
    );
}



constexpr float get_polar_angle(const pcl::PointXYZI& point)
{
    return static_cast<float>(
        asin(point.z / get_polar_distance(point))
    );
}



constexpr float get_rotation_angle(const pcl::PointXYZI& point)
{
    return static_cast<float>(
        acos(point.y / sqrt(pow(point.x, 2) + pow(point.y, 2)))
    );
}



constexpr float get_abs_derivative(const pcl::PointXYZI& prev_point, const pcl::PointXYZI& next_point)
{
    return static_cast<float>(
        abs((get_polar_distance(next_point) - get_polar_distance(prev_point) / 2))
    );
}



constexpr float get_distance(const pcl::PointXYZI& l, const pcl::PointXYZI& r)
{
    return static_cast<float>(
        sqrt(pow((l.x - r.x), 2) + pow((l.y - r.y), 2) + pow((l.z - r.z), 2))
    );
}



struct PointPolarAngleComp
{
    constexpr bool operator()(const pcl::PointXYZI& left, const pcl::PointXYZI& right)
    {
        return (get_polar_angle(left) < get_polar_angle(right) - POLAR_ANGLE_ERROR);
    }
};



#endif // UTILS_HPP
