#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class processPointCloud:public rclcpp::Node
{
    public:
    processPointCloud() : Node("process_point_cloud"), count_(0)
    {
        processedPointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_pointcloud_data",1);
        pointcloudSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/transformed_pointcloud_data", 10, std::bind(&processPointCloud::pcCallback, this, _1));
    }

    private:
    void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        /** This function is used to process point cloud data
         *
         * It performs voxel downsampling, distance thresholding and planar segmentation
         * It publishes the processed point cloud data
         * It subscribes to the transformed point cloud data
         *
         *  @param msg - input point cloud data
         *  @return void
        */

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2::Ptr voxel_cloud_filtered(new pcl::PCLPointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::fromROSMsg(*msg,*input_cloud);
        pcl::toPCLPointCloud2(*input_cloud, *cloud);

        // Voxel Downsampling, decrease the number of points
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.001f,0.001f,0.001f);
        voxel_filter.filter(*voxel_cloud_filtered);
        pcl::fromPCLPointCloud2(*voxel_cloud_filtered, *cloud_filtered);

        // Distance Thresholding along z-axis, remove unwanted points
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.0, 0.5);
        pass.setNegative (true);
        pass.filter (*cloud_passthrough);

        // Planar segmentation, remove table surface
        pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(0.001);
        plane_seg.setInputCloud(cloud_passthrough);
        plane_seg.segment(*inliers,*coefficients);

        // Extracting Points
        pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
        extract_indicies.setInputCloud(cloud_passthrough);
        extract_indicies.setIndices(inliers);
        extract_indicies.setNegative(true);
        extract_indicies.filter(*plane_seg_cloud);

        // Publishing point cloud data
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*plane_seg_cloud, point_cloud_msg);
        point_cloud_msg.header.frame_id = "world";
        point_cloud_msg.header.stamp = now();
        processedPointcloudPublisher->publish(point_cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudSubscriber;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processedPointcloudPublisher;
    size_t count_;
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<processPointCloud>());
	rclcpp::shutdown();
    return 0;
}