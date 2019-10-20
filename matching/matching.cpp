//
// Created by Akifumi Ohata on 2019/10/20.
//

#include "matching.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
namespace realsense {

    Matching::Matching() {


        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon(0.01);
        // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize(0.04);
        //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt.setResolution(0.5);

        ndt.setMaximumIterations(100);

        approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);


    }


    std::tuple<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    Matching::Aline(const pcl::PointCloud<pcl::PointXYZ> source_pt, const pcl::PointCloud<pcl::PointXYZ> map_pt,
                    Eigen::Matrix4f init_guess) {

        // Loading first scan of room.
        pcl_ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        *target_cloud = map_pt;

        if (map_pt.width == 0) {
            PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        }

        // Loading second scan of room from new perspective.
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *input_cloud = source_pt;
        if (source_pt.width == 0) {
            PCL_ERROR ("Couldn't read source scan  \n");
        }

        // Filtering input scan to roughly 10% of original size to increase speed of registration.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        approximate_voxel_filter.setInputCloud(input_cloud);
        approximate_voxel_filter.filter(*filtered_cloud);


        ndt.setInputSource(filtered_cloud);
        ndt.setInputTarget(target_cloud);


        // Calculating required rigid transform to align the input cloud to the target cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud, init_guess);

        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
                  << " score: " << ndt.getFitnessScore() << std::endl;

        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());


        return std::forward_as_tuple(ndt.getFinalTransformation(), output_cloud);
    }
}

//Eigen::Matrix4f ndt_matching_for_map(const pcl::PointCloud<pcl::PointXYZ> source_points,
//                                     const pcl::PointCloud<pcl::PointXYZ> map_points, Eigen::Matrix4f init_guess,
//                                     pcl_ptr output_cloud) {
//
//    // Loading first scan of room.
//    pcl_ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    *target_cloud = map_points;
//
//    if (map_points.width == 0) {
//        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
//    }
//
//    // Loading second scan of room from new perspective.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *input_cloud = source_points;
//    if (source_points.width == 0) {
//        PCL_ERROR ("Couldn't read source scan  \n");
//    }
//
//    // Filtering input scan to roughly 10% of original size to increase speed of registration.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//    approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
//    approximate_voxel_filter.setInputCloud(input_cloud);
//    approximate_voxel_filter.filter(*filtered_cloud);
//
//
//    // Initializing Normal Distributions Transform (NDT).
//    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
//
//
//    ndt.setInputSource(filtered_cloud);
//    ndt.setInputTarget(target_cloud);
//
//
//    // Calculating required rigid transform to align the input cloud to the target cloud.
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    ndt.align(*output_cloud, init_guess);
//
//    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
//              << " score: " << ndt.getFitnessScore() << std::endl;
//
//    // Transforming unfiltered, input cloud using found transform.
//    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
//
//
//    return ndt.getFinalTransformation();
//}
