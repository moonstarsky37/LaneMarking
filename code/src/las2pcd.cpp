#include <iostream>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "data_io.h"
#include "utility.h"

// pcl io
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


// *.las io
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>


// using namespace std;
int las2pcd(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input.las> <output.pcd>" << std::endl;
        return -1;
    }

    // Load LAS file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: cannot read input file " << argv[1] << std::endl;
        return -1;
    }

    // Convert to PCD file
    pcl::io::savePCDFileASCII(argv[2], *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " << argv[2] << std::endl;

    return 0;
}
// int main(int argc, char **argv)
// {
//     if (argc != 3)
//     {
//         std::cerr << "Usage: " << argv[0] << " <input.las> <output.pcd>" << std::endl;
//         return -1;
//     }

//     // Load LAS file
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     if (pcl::io::loadLASFile(argv[1], *cloud) == -1)
//     {
//         std::cerr << "Error: cannot read input file " << argv[1] << std::endl;
//         return -1;
//     }

//     // Convert to PCD file
//     pcl::io::savePCDFileASCII(argv[2], *cloud);
//     std::cerr << "Saved " << cloud->points.size() << " data points to " << argv[2] << std::endl;

//     return 0;
// }