#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <pose_handler.h>
#include <chrono>
#include <frontier_search.h>

class RobotExploration{

public:

  RobotExploration(){}

  void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);

  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid frontiers_map;
  std::vector<std::pair<int, int> > centroids;

  std::vector<std::vector<std::pair<int, int> > > frontiers;

  ros::NodeHandle nh;

  PoseHandlers::PoseHandler pose_handler;

private:

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "robot_exploration");

  RobotExploration robot_exploration;

  ROS_INFO("Hello world!");

  ros::Subscriber map_sub = robot_exploration.nh.subscribe("/move_base/local_costmap/costmap", 1, &RobotExploration::map_callback, &robot_exploration);
  ros::Publisher frontier_pub = robot_exploration.nh.advertise<nav_msgs::OccupancyGrid>("frontiers", 10);
  ros::Publisher goal_pub = robot_exploration.nh.advertise<geometry_msgs::PoseStamped>("/frontier_goal", 1);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){

    for(int i = 0; i<robot_exploration.map.data.size(); i++ ){
      robot_exploration.frontiers_map.data[i] = -1;
    }
    int data_index = 0;
    int darkness = 120;
    for(std::vector<std::pair<int, int> > frontier : robot_exploration.frontiers){
      for(std::pair<int, int> coords: frontier){
        for(int i=0; i<robot_exploration.map.info.height; i++){
          for(int j=0; j<robot_exploration.map.info.width; j++){
            if(coords.first == i && coords.second == j){
              robot_exploration.frontiers_map.data[data_index] = darkness;
            }
            if(i == 288 && j == 288)robot_exploration.frontiers_map.data[data_index] = 99;
            data_index++;
          }
        }
        data_index=0;
      }
      darkness=(darkness+100);
    }
    data_index = 0;
    for(int i=0; i<robot_exploration.map.info.height; i++){
      for(int j=0; j<robot_exploration.map.info.width; j++){
        for(std::pair<int, int> coords: robot_exploration.centroids){
          if(coords.first == i && coords.second == j){
            robot_exploration.frontiers_map.data[data_index] = 0;
          }
        }
        data_index++;
      }
    }

    frontier_pub.publish(robot_exploration.frontiers_map);

    if(!robot_exploration.centroids.empty())
    {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "robot0/map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = (robot_exploration.centroids[0].second*robot_exploration.map.info.resolution)+robot_exploration.map.info.origin.position.x;
    goal.pose.position.y = (robot_exploration.centroids[0].first*robot_exploration.map.info.resolution)+robot_exploration.map.info.origin.position.y;
    goal.pose.position.z = 0;
    //ROS_INFO("%d, %f, %f", robot_exploration.centroids[0].first, robot_exploration.map.info.resolution, robot_exploration.map.info.origin.position.x);
    ROS_INFO("Centroid (x,y) map: %d, %d \t (x,y) rw: %f, %f  ", robot_exploration.centroids[0].first, robot_exploration.centroids[0].second, goal.pose.position.x, goal.pose.position.y);

    goal_pub.publish(goal);
    }

    ros::spinOnce();

    loop_rate.sleep();

  }

  ros::shutdown();

}

void RobotExploration::map_callback(const nav_msgs::OccupancyGridConstPtr &msg){

  map = *msg;
  // auto start = std::chrono::high_resolution_clock::now();
  FrontierSearches::FrontierSearch frontier_search(map, pose_handler);
  frontiers = frontier_search.buildBidimensionalMap();
  centroids = frontier_search.getCentroids(frontiers);
  frontiers_map = map;
  // auto end = std::chrono::high_resolution_clock::now();
  // auto timespan=std::chrono::duration<double>(end - start);
  // std::cout<<"constructing time:"<<timespan.count()<<std::endl;
  // std::cout<<"constructing time:"<<timespan.count()<<std::endl;
  // std::cout<<"constructing time:"<<timespan.count()<<std::endl;
  // std::cout<<"constructing time:"<<timespan.count()<<std::endl;
}

