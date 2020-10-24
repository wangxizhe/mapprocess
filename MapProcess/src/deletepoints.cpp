/*************************************
 * 文件名：ugv_position.cpp
 * 创建人：东北大学-王希哲
 * 描 述：室内、SLAM、定位、A*、Frenet
 * 日 期：2020-5-19
 * 版 本：1.0.0
 *************************************/
#include <ros/ros.h>
#include <string.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>//NDT配准类对应头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化头文件
#include <boost/thread/thread.hpp>//多线程相关头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>              //标准C++库中的输入输出的头文件
#include <pcl/visualization/cloud_viewer.h>  //点类型相关定义
#include "stdlib.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter.h>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <thread>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <vector>
#include <list>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class ugv_position
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_Start, sub_Goal;
    pcl::PointXYZI initial_point;
    pcl::PointCloud<pcl::PointXYZI>::Ptr range;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_final;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pub;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pre;
    std_msgs::Header point_cloud_header_;
    ros::Publisher mapPublisher;
    int LOCK_setgoal=0;
  public:
      ugv_position()
      {
         sub_Start = nh.subscribe("/initialpose", 1, &ugv_position::Startcallback, this);
         sub_Goal = nh.subscribe("/move_base_simple/goal", 1, &ugv_position::Goalcallback, this);

         mapPublisher=nh.advertise<sensor_msgs::PointCloud2>("/map", 10);
         pcl::PointCloud<pcl::PointXYZI>::Ptr temp (new pcl::PointCloud<pcl::PointXYZI>);
         map_final = temp;
         pcl::PointCloud<pcl::PointXYZI>::Ptr temp1 (new pcl::PointCloud<pcl::PointXYZI>);
         range=temp1;
         pcl::PointCloud<pcl::PointXYZI>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZI>);
         map_pub=temp2;
         pcl::PointCloud<pcl::PointXYZI>::Ptr temp3(new pcl::PointCloud<pcl::PointXYZI>);
         map_pre=temp2;
         pcl::PointCloud<pcl::PointXYZ>::Ptr map_temp (new pcl::PointCloud<pcl::PointXYZ>);
         pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
         pcl::io::loadPCDFile("/home/wxz/catkin_workspace2/src/ugv_position/include/map.pcd", *map_temp);

         pcl::copyPointCloud(*map_temp, *map_final);

//         for(int i=0; i < map->points.size(); i++)
//         {
//             pcl::PointXYZI temp;
//             temp.x=map->points[i].z;
//             temp.y=map->points[i].x;
//             temp.z=map->points[i].y;
//             map_final->points.push_back(temp);
//         }
      }
    void Startcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
    void pub_cloud();
    void Goalcallback(const geometry_msgs::PoseStamped::ConstPtr& end);
    ~ugv_position(){
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_final_temp;
        map_final_temp = map_final;
        map_final_temp->height = 1;
        map_final_temp->width = map_final->size() ;
        pcl::io::savePCDFileASCII ("/home/wxz/Desktop/map.pcd", *map_final_temp);
    }
};

void ugv_position::Startcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial)
{
  cout<<"in_start"<<endl;
  initial_point.x=initial->pose.pose.position.x;
  initial_point.y=initial->pose.pose.position.y;
  range->points.push_back(initial_point);
  if(range->points.size()>1)
  {
      double smallx, smally, bigx, bigy;
      if(range->points[0].x>range->points[1].x)
      {
          smallx=range->points[1].x;
          bigx=range->points[0].x;
      }
      else
      {
          smallx=range->points[0].x;
          bigx=range->points[1].x;
      }
      if(range->points[0].y>range->points[1].y)
      {
          smally=range->points[1].y;
          bigy=range->points[0].y;
      }
      else
      {
          smally=range->points[0].y;
          bigy=range->points[1].y;
      }
      range->points.clear();
      map_pre->points.clear();
      pcl::PointCloud<pcl::PointXYZI>::Ptr maTT(new pcl::PointCloud<pcl::PointXYZI>);
      for(int i=0; i<map_final->points.size(); i++)
      {
         if(map_final->points[i].x>smallx&&map_final->points[i].x<bigx&&map_final->points[i].y>smally&&map_final->points[i].y<bigy)
         {
             map_pre->points.push_back(map_final->points[i]);
             continue;
         }
         else
         {
             maTT->points.push_back(map_final->points[i]);
         }
      }
      map_final->points.clear();
      pcl::copyPointCloud(*maTT, *map_final);
  }
}
void ugv_position::Goalcallback(const geometry_msgs::PoseStamped::ConstPtr& end)
{
  cout<<"Goalcallback"<<endl;
  for(int i=0; i<map_pre->points.size(); i++)
     map_final->points.push_back(map_pre->points[i]);
}

void ugv_position::pub_cloud()
{
  while(ros::ok())
  {
    ros::Rate rate(10);
    sensor_msgs::PointCloud2 ros_msgs;
    cout<<map_final->points.size()<<endl;

    pcl::toROSMsg(*map_final, ros_msgs);
    ros_msgs.header =point_cloud_header_ ;
    ros_msgs.header.frame_id = "/velodyne";
    mapPublisher.publish(ros_msgs);
    rate.sleep();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ugv_position");
  ugv_position start_detec;
  ugv_position MO;
  std::thread loopthread(&ugv_position::pub_cloud, &MO);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  loopthread.detach();
  return 0;
}
