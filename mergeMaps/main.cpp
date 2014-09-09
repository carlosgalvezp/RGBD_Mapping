#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>

void readTransform(const char* filename, Eigen::Matrix4f& transform);

int main(int argc, char* argv[])
{
    std::cout<<"Merging maps...\n";

    // *** Check input
    if(argc < 4)
    {
        std::cout<<"Usage: ./mergeMaps <map1.pcd> <map2.pcd> <mergedMap.pcd> [transform-2-to-1.txt]";
        return -1;
    }

    // *** Read data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());

    if (pcl::io::loadPCDFile (argv[1], *cloud1) < 0)
    {
          std::cout << "Error loading point cloud " << argv[1] << std::endl;
          return -1;
    }

    if (pcl::io::loadPCDFile (argv[2], *cloud2) < 0)
    {
          std::cout << "Error loading point cloud " << argv[2] << std::endl;
          return -1;
    }

    // *** Transformation (optional)
    Eigen::Matrix4f transform;

    if(argc > 4)
        readTransform(argv[4], transform);
    else
        transform.setIdentity();

    pcl::transformPointCloud(*cloud1, *cloud1, transform);

    // *** Merging
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    *finalCloud = *cloud1 + *cloud2;

    // *** Save to file
    if (pcl::io::savePCDFile(argv[3], *finalCloud) < 0)
    {
        std::cout << "Error saving the point cloud in "<<argv[4] << std::endl;
        return -1;
    }

    std::cout << "Successfully merged maps" << std::endl;
    return 0;
}


void readTransform(const char* filename, Eigen::Matrix4f& transform)
{
    std::cout << "Reading transformation..." <<std::endl;
    // *** Read file
    std::ifstream file;
    file.open (filename);

    // *** Fill in transformation matrix
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            file>>transform(i,j);
        }
    }
}
