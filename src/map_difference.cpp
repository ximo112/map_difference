#include <ros/ros.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

class Map_Difference{
public:
  Map_Difference(){
    map_sub = nh.subscribe("map", 10, &Map_Difference::mapCallBack, this);
    scan_sub = nh.subscribe("scan", 10, &Map_Difference::scanCallBack, this);
    static_obstacles_pub = nh.advertise<sensor_msgs::PointCloud>("static_obstacles", 100);
    mapcall = false;
    ros::Rate loop_rate(10);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  sensor_msgs::PointCloud static_obstacles;
  ros::Publisher static_obstacles_pub;
  int mapcall;


  void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map){
    mapcall = true;
    while(ros::ok()){
      float x = map->info.origin.position.x + map->info.resolution / 2;
      float y = map->info.origin.position.y + map->info.resolution / 2;
      int i , j = 0, static_obstacles_num = 0;
      for(i = 0; i < map->info.width * map->info.height; i++){
        if(map->data[i] == 100){
          static_obstacles_num += 1;
        }
      }
      static_obstacles.points.resize(static_obstacles_num);

      for(i = 0; i < map->info.width * map->info.height; i++){
        if(map->data[i] == 100){
          static_obstacles.points[j].x = x;
          static_obstacles.points[j].y = y;
          j += 1;
        }
        x += map->info.resolution;
        if(x > map->info.width * map->info.resolution + map->info.origin.position.x){
          x = map->info.origin.position.x + map->info.resolution / 2;
          y += map->info.resolution;
        }
      }
      static_obstacles.header.frame_id = map->header.frame_id;

      static_obstacles_pub.publish(static_obstacles);
    }
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(mapcall != true){
      ROS_WRAN("there is no map");
    }else{
      //差分の処理を入れる
    }
  }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "map_difference");
  ROS_INFO("hallo");
  Map_Difference static_obstacles;
  static_obstacles;

  ros::spin();

  return 0;
}
