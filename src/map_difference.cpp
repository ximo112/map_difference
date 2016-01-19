#include <ros/ros.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#define allowable_error 0.2
#define static_error 0.01

class Map_Difference{
public:
  Map_Difference(){
    map_sub = nh.subscribe("map", 10, &Map_Difference::mapCallBack, this);
    scan_sub = nh.subscribe("scan", 10, &Map_Difference::scanCallBack, this);
    static_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("static_obstacle", 100);
    dynamic_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("dynamic_obstacle", 100);
    check = false;
  }

  void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map){
    float x = map->info.origin.position.x + map->info.resolution / 2;
    float y = map->info.origin.position.y + map->info.resolution / 2;
    int i , j = 0, static_obstacle_num = 0;
    check = true;
    for(i = 0; i < map->info.width * map->info.height; i++){
      if(map->data[i] == 100){
        static_obstacle_num += 1;
      }
    }
    static_obstacle_.points.resize(static_obstacle_num);

    for(i = 0; i < map->info.width * map->info.height; i++){
      if(map->data[i] == 100){
        static_obstacle_.points[j].x = x;
        static_obstacle_.points[j].y = y;
        j += 1;
      }
      x += map->info.resolution;
      if(x > map->info.width * map->info.resolution + map->info.origin.position.x){
        x = map->info.origin.position.x + map->info.resolution / 2;
        y += map->info.resolution;
      }
    }
    static_obstacle_.header.frame_id = map->header.frame_id;
    static_obstacle_.header.stamp = ros::Time::now();

    static_obstacle_pub.publish(static_obstacle_);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    float distance;
    int i, j, count, k, dynamic_obstacle_num;

   if(check == false){
      ROS_WARN("map is not call");
    }else{
      //差分の処理を入れる
      dynamic_obstacle_num = 0;
      dynamic_obstacle_.points.clear();

      projector.projectLaser(*scan, scan_cloud);

      float scan_cloud_x[(int)scan_cloud.points.size()], scan_cloud_y[(int)scan_cloud.points.size()], scan_cloud_intensity[(int)scan_cloud.channels[0].values.size()];
      count = 0;
      for(i = 0; i < (int)scan_cloud.points.size(); i++){
        if((float)hypotf(scan_cloud.points[i].x, scan_cloud.points[i].y) < 29){
          scan_cloud_x[count] = scan_cloud.points[i].x;
          scan_cloud_y[count] = scan_cloud.points[i].y;
          scan_cloud_intensity[count] = scan_cloud.channels[0].values[i];
          count += 1;
        }
      }
      scan_cloud.points.resize(count);
      scan_cloud.channels[0].values.resize(count);
      for(i = 0; i < count; i++){
        scan_cloud.points[i].x = scan_cloud_x[i];
        scan_cloud.points[i].y = scan_cloud_y[i];
        scan_cloud.channels[0].values[i] = scan_cloud_intensity[i];
      }

      //error処理
      if(!listener.waitForTransform(scan->header.frame_id, static_obstacle_.header.frame_id, scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment), ros::Duration(1.0))){
        return;
      }

      listener.transformPointCloud(static_obstacle_.header.frame_id, scan_cloud, scan_cloud);

      for(i = 0; i < (int)scan_cloud.points.size(); i++){
        int dynamic_obstacle_check = true;
        float x_min = scan_cloud.points[i].x - allowable_error;
        float x_max = scan_cloud.points[i].x + allowable_error;
        float y_min = scan_cloud.points[i].y - allowable_error;
        float y_max = scan_cloud.points[i].y + allowable_error;
        for(j = 0; j < (int)static_obstacle_.points.size(); j++){
          if(x_min < static_obstacle_.points[j].x && static_obstacle_.points[j].x < x_max && y_min < static_obstacle_.points[j].y && static_obstacle_.points[j].y < y_max){
            dynamic_obstacle_check = false;
          }
        }
        if(dynamic_obstacle_check == true){
          dynamic_obstacle_num += 1;
        }
      }

      if(dynamic_obstacle_num >= (int)scan_cloud.points.size() * 0.9){
        ROS_WARN("Self-location is not accurate, or landmarks is less");
        dynamic_obstacle_.points.clear();
        dynamic_obstacle_.channels.clear();
      }else{
        ROS_INFO("ok");
        dynamic_obstacle_.points.resize(dynamic_obstacle_num);
        dynamic_obstacle_.channels.resize(1);
        dynamic_obstacle_.channels[0].name = "intensity";
        dynamic_obstacle_.channels[0].values.resize(dynamic_obstacle_num);
        k = 0;

        int dynamic_obstacle_check[(int)scan_cloud.points.size()];
        for(i = 0; i < (int)scan_cloud.points.size(); i++){
          dynamic_obstacle_check[i] = true;
          float x_min = scan_cloud.points[i].x - allowable_error;
          float x_max = scan_cloud.points[i].x + allowable_error;
          float y_min = scan_cloud.points[i].y - allowable_error;
          float y_max = scan_cloud.points[i].y + allowable_error;
          for(j = 0; j < (int)static_obstacle_.points.size(); j++){
            if(x_min < static_obstacle_.points[j].x && static_obstacle_.points[j].x < x_max && y_min < static_obstacle_.points[j].y && static_obstacle_.points[j].y < y_max){
              dynamic_obstacle_check[i] = false;
            }
          }
          if(dynamic_obstacle_check[i] == true){
            dynamic_obstacle_.points[k].x = scan_cloud.points[i].x;
            dynamic_obstacle_.points[k].y = scan_cloud.points[i].y;
            dynamic_obstacle_.channels[0].values[k] = scan_cloud.channels[0].values[i];
            k += 1;
          }
        }
      }
    }

    dynamic_obstacle_.header.stamp = scan->header.stamp;
    dynamic_obstacle_.header.frame_id = static_obstacle_.header.frame_id;

    sensor_msgs::PointCloud2 dynamic_obstacle2_;
    sensor_msgs::convertPointCloudToPointCloud2(dynamic_obstacle_, dynamic_obstacle2_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dynamic_obstacle(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(dynamic_obstacle2_, *pcl_dynamic_obstacle);

    ROS_INFO("%d", (int)dynamic_obstacle_.points.size());///
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pcl_dynamic_obstacle);
    sor.setMeanK (2);
    sor.setStddevMulThresh (0.8);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dynamic_obstacle_filter(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*pcl_dynamic_obstacle_filter);

    sensor_msgs::PointCloud2 dynamic_obstacle2_filter_;
    pcl::toROSMsg(*pcl_dynamic_obstacle_filter, dynamic_obstacle2_filter_);
    sensor_msgs::PointCloud dynamic_obstacle_filter_;
    sensor_msgs::convertPointCloud2ToPointCloud(dynamic_obstacle2_filter_, dynamic_obstacle_filter_);
    dynamic_obstacle_filter_.header.stamp = ros::Time::now();
    dynamic_obstacle_filter_.header.frame_id = static_obstacle_.header.frame_id;

    dynamic_obstacle_pub.publish(dynamic_obstacle_filter_);
  }
  
private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Subscriber scan_sub;
  ros::Publisher static_obstacle_pub;
  ros::Publisher dynamic_obstacle_pub;
  sensor_msgs::PointCloud static_obstacle_;
  laser_geometry::LaserProjection projector;
  tf::TransformListener listener;
  sensor_msgs::PointCloud scan_cloud;
  sensor_msgs::PointCloud dynamic_obstacle_;
  int check;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "map_difference");
  ROS_INFO("hallo");
  Map_Difference obstacle_cloud;
  obstacle_cloud;

  ros::spin();

  return 0;
}
