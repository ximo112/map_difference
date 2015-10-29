#include <ros/ros.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#define allowable_error 0.5

class Map_Difference{
public:
  Map_Difference(){
    map_sub = nh.subscribe("map", 10, &Map_Difference::mapCallBack, this);
    scan_sub = nh.subscribe("scan", 10, &Map_Difference::scanCallBack, this);
    static_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("static_obstacle", 100);
    dynamic_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("dynamic_obstacle", 100);
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

  void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map){
    float x = map->info.origin.position.x + map->info.resolution / 2;
    float y = map->info.origin.position.y + map->info.resolution / 2;
    int i , j = 0, static_obstacle_num = 0;
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

    static_obstacle_pub.publish(static_obstacle_);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    //差分の処理を入れる
    float distance;
    int i, j, count, k = 0, dynamic_obstacle_num = 0;

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
      count = 0;
      for(j = 0; j < (int)static_obstacle_.points.size(); j++){
        distance = hypotf(static_obstacle_.points[j].x - scan_cloud.points[i].x, static_obstacle_.points[j].y - scan_cloud.points[i].y);
        if(distance > allowable_error){
          count += 1;
        }
      }
      if(count == (int)static_obstacle_.points.size()){
        dynamic_obstacle_num += 1;
      }
    }

    if(dynamic_obstacle_num >= (int)scan_cloud.points.size() * 0.8){
      ROS_WARN("Self-location is not accurate, or landmarks is less");
    }else{
      dynamic_obstacle_.points.resize(dynamic_obstacle_num);
      dynamic_obstacle_.channels.resize(1);
      dynamic_obstacle_.channels[0].name = "intensity";
      dynamic_obstacle_.channels[0].values.resize(dynamic_obstacle_num);

      for(i = 0; i < (int)scan_cloud.points.size(); i++){
        count = 0;
        for(j = 0; j < (int)static_obstacle_.points.size(); j++){
          distance = hypotf(static_obstacle_.points[j].x - scan_cloud.points[i].x, static_obstacle_.points[j].y - scan_cloud.points[i].y);
          if(distance > allowable_error){
            count += 1;
          }
        }
        if(count == (int)static_obstacle_.points.size()){
          dynamic_obstacle_.points[k].x = scan_cloud.points[i].x;
          dynamic_obstacle_.points[k].y = scan_cloud.points[i].y;
          dynamic_obstacle_.channels[0].values[k] = scan_cloud.channels[0].values[i];
          k += 1;
        }
      }
    }

    dynamic_obstacle_.header.stamp = scan->header.stamp;
    dynamic_obstacle_.header.frame_id = static_obstacle_.header.frame_id;

    dynamic_obstacle_pub.publish(dynamic_obstacle_);

    dynamic_obstacle_.points.clear();
  }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "map_difference");
  ROS_INFO("hallo");
  Map_Difference obstacle_cloud;
  obstacle_cloud;

  ros::spin();

  return 0;
}
