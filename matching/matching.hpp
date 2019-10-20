//
// Created by Akifumi Ohata on 2019/10/20.
//

#ifndef LRS_PCL_MATCHING_HPP
#define LRS_PCL_MATCHING_HPP

#include <tuple>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>


namespace realsense {
    class Matching {
    private:
        float voxel_leaf_size = 0.1;

        float epsilon = 0.01;
        float step_size = 0.04;
        float resolusiton = 0.5;
        int maximum_iteration = 100;

        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    public:
        Matching();

        std::tuple<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>
        Aline(const pcl::PointCloud<pcl::PointXYZ> source_pt, const pcl::PointCloud<pcl::PointXYZ> map_pt,
              Eigen::Matrix4f init_guess);

    };
}
#endif //LRS_PCL_MATCHING_HPP
