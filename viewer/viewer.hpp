//
// Created by Akifumi Ohata on 2019/10/20.
//

#ifndef LRS_PCL_VIEWER_HPP
#define LRS_PCL_VIEWER_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

class PointViewer {
private:
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer;

    boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>> color_handler;
    boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> single_color;
    const std::string cood_name = "realsense";

public:
    PointViewer();
    ~PointViewer();

    void Update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trance,pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,Eigen::Affine3f ret);
};


#endif //LRS_PCL_VIEWER_HPP

