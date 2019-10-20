#include <iostream>
#include <algorithm>


#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include "realsense/realsense.hpp"
#include "matching/matching.cpp"
#include "convert.hpp"
#include "viewer/viewer.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using namespace std;
bool stop_flag = false;

int main() {


    int count = 0;
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    RealSense camera = RealSense();

    PointViewer pointViewer = PointViewer();

    realsense::Matching ndt_match = realsense::Matching();

    cloud = points_to_pcl(camera.get_point_frame());




    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::Matrix4f init_position = (init_translation * init_rotation).matrix();


    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan_org.pcd", *map_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }


    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(0.05f, 0.05f, 0.05f);


    // voxel grid filter
    grid.setInputCloud(cloud);
    grid.filter(*map_cloud);

    while (!stop_flag) {


        cloud = points_to_pcl(camera.get_point_frame());

        // voxel grid filter
        grid.setLeafSize(0.05f, 0.05f, 0.05f);
        grid.setInputCloud(cloud);
        grid.filter(*cloud_filtered);
        //ndt

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trance(new pcl::PointCloud<pcl::PointXYZ>());

        std::tie(init_position, cloud_trance)= ndt_match.Aline(*cloud_filtered, *map_cloud, init_position);


        cout << init_position << endl;
        *map_cloud += *cloud_trance;

        // voxel grid filter
        grid.setLeafSize(0.1f, 0.1f, 0.1f);
        grid.setInputCloud(map_cloud);
        grid.filter(*map_cloud);


        Eigen::Affine3f ret;
        ret.matrix() = init_position;
        pointViewer.Update(cloud_trance, map_cloud, ret);

        count += 1;

        if (stop_flag == true) {
            break;
        }
    }


    pcl::io::savePCDFileBinary("room_scan_org.pcd", *map_cloud);

}