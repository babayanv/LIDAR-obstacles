#include <exception>
#include <algorithm>
#include <vector>

#include <find_obstacles/Map.hpp>
#include <find_obstacles/core.hpp>
#include <find_obstacles/utils.hpp>



Map::Map() :
    m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
}



Map::Map(const std::string& filename) :
    m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *m_cloud) == -1)
    {
        throw std::invalid_argument("Couldn't read file\n");
    }
}



void Map::load(const std::string& filename)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *m_cloud) == -1)
    {
        throw std::invalid_argument("Couldn't read file\n");
    }
}



void Map::write_to_file(const std::string& filename)
{
    pcl::io::savePCDFileASCII(filename, *m_cloud);
}



void Map::identify_obstacles()
{
    std::stable_sort<PCLPointCloudIterator, PointPolarAngleComp>(
        m_cloud->points.begin(),
        m_cloud->points.end(),
        PointPolarAngleComp()
    );

    std::vector<size_t> rings_distibution;
    distribute_points(rings_distibution);

    for(auto&& i = rings_distibution.begin(); i != rings_distibution.end() - 1; ++i)
    {
        const size_t& ring_begin = *i;
        const size_t& ring_end = *(i + 1);

        std::vector<size_t> ring_sectors;
        divide_ring(ring_begin, ring_end, ring_sectors);

        for(auto&& j = ring_sectors.begin(); j != ring_sectors.end() - 1; ++j)
        {
            const size_t& sector_begin = *j;
            const size_t& sector_end = *(j + 1);

            colorize_sector(sector_begin, sector_end);
        }
    }

    for(auto& i : m_cloud->points)
    {
        if(i.intensity > 10)
        {
            i.intensity = 10;
        }
        if(i.intensity < 2)
        {
            i.intensity = 2;
        }
        i.intensity /= 10;
    }
}



void Map::distribute_points(std::vector<size_t>& dest)
{
    dest.clear();
    dest.push_back(0);

    double curr_rad = get_polar_angle(m_cloud->points.front());

    for(size_t i = 0; i < m_cloud->points.size(); ++i)
    {
        if(get_polar_angle(m_cloud->points[i]) > curr_rad + POLAR_ANGLE_ERROR)
        {
            curr_rad = get_polar_angle(m_cloud->points[i]);
            dest.push_back(i);
        }
    }

    dest.push_back(m_cloud->points.size() - 1);
}



void Map::divide_ring(size_t ring_begin, size_t ring_end, std::vector<size_t>& dest)
{
    dest.clear();
    dest.push_back(ring_begin);

    float rel_rot = get_rotation_angle(m_cloud->points[ring_begin]);

    for(size_t i = ring_begin; i < ring_end; ++i)
    {
        if(sector_condition(ring_begin, ring_end, i, rel_rot))
        {
            dest.push_back(i);

            rel_rot = get_rotation_angle(m_cloud->points[i]);
        }
    }

    if(dest.back() != ring_end)
    {
        dest.push_back(ring_end);
    }
}



void Map::colorize_sector(size_t sector_begin, size_t sector_end)
{
    float sum = 0;

    for(size_t i = sector_begin; i < sector_end; ++i)
    {
        const size_t prev = (i == sector_begin ? sector_end : i - 1);
        const size_t next = (i == sector_end ? sector_begin : i + 1);

        sum += get_abs_derivative(m_cloud->points[prev], m_cloud->points[next]);
    }

    const float average = sum / static_cast<float>(sector_end - sector_begin);

    for(size_t i = sector_begin; i < sector_end; ++i)
    {
        m_cloud->points[i].intensity = average;
    }
}



bool Map::sector_condition(size_t ring_begin, size_t ring_end, size_t idx, float rel_rot)
{
    const size_t prev = (idx == ring_begin ? ring_end : idx - 1);
    const size_t next = (idx == ring_end ? ring_begin : idx + 1);

    const float& prev_rot = get_rotation_angle(m_cloud->points[prev]);
    const float& curr_rot = get_rotation_angle(m_cloud->points[idx]);
    const float& next_rot = get_rotation_angle(m_cloud->points[next]);

    return curr_rot > rel_rot + SECTOR_ANGLE ||
        curr_rot < rel_rot - SECTOR_ANGLE ||
        (curr_rot <= prev_rot && curr_rot <= next_rot) ||
        (curr_rot >= prev_rot && curr_rot >= next_rot);

}