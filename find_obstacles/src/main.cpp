#include <find_obstacles/Map.hpp>

#include <iostream>
#include <vector>


int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        return 0;
    }

    Map terrain(argv[1]);
    
    terrain.identify_obstacles();

    terrain.write_to_file("output.pcd");
}
