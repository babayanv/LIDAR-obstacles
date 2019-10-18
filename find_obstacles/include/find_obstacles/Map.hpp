#ifndef MAP_HPP
#define MAP_HPP



#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



class Map
{
    using PCLPointCloudIterator = typename std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::iterator;

public:
    Map();
    explicit Map(const std::string& filename);
    ~Map() = default;

    void load(const std::string& filename);
    void write_to_file(const std::string& filename);

    void identify_obstacles();

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;

private:
    Map(const Map& other) = delete;
    Map(Map&& other) = delete;

    void distribute_points(std::vector<size_t>& dest);
    void divide_ring(size_t ring_begin, size_t ring_end, std::vector<size_t>& dest);
    void colorize_sector(size_t sector_begin, size_t sector_end);

    bool sector_condition(size_t ring_begin, size_t ring_end, size_t idx, float rel_rot);
};



#endif // MAP_HPP
