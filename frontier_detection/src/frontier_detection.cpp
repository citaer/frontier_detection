#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <unordered_map>
// #include <boost>
#include <time.h>

#define INF INT_MAX
using namespace std;

namespace frontier_detection
{

  
    class frontierdetection
    {
        public:
            frontierdetection(ros::NodeHandle private_nh)
            {
                getCostmap = false;
                NODE = private_nh;
                costmap_sub_global_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &frontierdetection::CostmapSubCallback_global, this);
                frontier_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/frontier/goal", 1);
                robot_position_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/robot_position", 1);
            }


        private:
            void CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map)
            {
                ros::Time timestamp = map->header.stamp;
                global_costmap = *map;
                GetRobotpose("/PandarQT",global_pose, timestamp);
                int mapHeight=global_costmap.info.height;
                int mapWidth=global_costmap.info.width;
                int a[3];
                a[0]=(global_pose.pose.position.x+global_costmap.info.width*global_costmap.info.resolution/2)*100000;
                a[1]=(global_pose.pose.position.y+global_costmap.info.height*global_costmap.info.resolution/2)*100000;
                a[2]=global_costmap.info.resolution*100000;
                robot_index=(a[1]/a[2])*global_costmap.info.width+a[0]/a[2];
                robot_position_pub_.publish(global_pose);
                find_closest_frontier(global_costmap,robot_index,closest_frontier_idx);
                closest_frontier_point.header.frame_id = "/robot0/map";
                closest_frontier_point.header.stamp = ros::Time::now();
                closest_frontier_point.pose.orientation.w = 1;
                closest_frontier_point.pose.position.x=closest_frontier_idx%global_costmap.info.width*global_costmap.info.resolution+global_costmap.info.origin.position.x;
                closest_frontier_point.pose.position.y=closest_frontier_idx/global_costmap.info.width*global_costmap.info.resolution+global_costmap.info.origin.position.y;
                closest_frontier_point.pose.position.z=0;
                frontier_pub_.publish(closest_frontier_point);
            }



    void find_closest_frontier(const nav_msgs::OccupancyGrid& occupancy_map,int start_idx,int &frontier_idx) 
    {
        int a[4]={1,(int)occupancy_map.info.width,-1,-(int)occupancy_map.info.width};
        std::unordered_map<int, int> visited, unvisited;
        std::queue<int> id;
        int current;
        if(occupancy_map.data[start_idx]<97)
        {
        id.push(start_idx);
        while (id.size()) 
        {
            current = id.front();
            id.pop();
            for(int i=0;i<4;i++)
            {
                int neighbor_id=current+a[i];
                if((occupancy_map.data[neighbor_id]<97)&&!(visited[neighbor_id]))
                {
                    if(occupancy_map.data[current]==-1)
                    {
                        frontier_idx=current;
                        id=queue<int>();
                        break;
                    }
                        id.push(neighbor_id);
                        ++visited[neighbor_id];
                }
            }
        }
        }
    }
            
            void GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_pose, ros::Time timestamp)
            {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped iden_pose;
            iden_pose.header.frame_id = iden_frame_id;
            iden_pose.header.stamp = ros::Time::now(); 
            iden_pose.pose.orientation.w = 1;
            tf_listener_.waitForTransform("/robot0/map",iden_frame_id, ros::Time(0), ros::Duration(2.0));
            tf_listener_.lookupTransform( "/robot0/map",iden_frame_id, ros::Time(0), transform);
            global_pose.pose.position.x=transform.getOrigin().x();
            global_pose.pose.position.y=transform.getOrigin().y();
            global_pose.pose.position.z=transform.getOrigin().z();
            }

            bool getCostmap;
            ros::NodeHandle NODE;
            ros::Subscriber costmap_sub_global_;
            ros::Publisher frontier_pub_;
            ros::Publisher robot_position_pub_;
            nav_msgs::OccupancyGrid global_costmap;
            nav_msgs::OccupancyGrid local_costmap;
            geometry_msgs::PoseStamped global_pose;
            geometry_msgs::PoseStamped closest_frontier_point;

            std::mutex lock_costmap;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            int robot_index;
            int closest_frontier_idx;
            int resolution=0.2;

            // protected:

    };
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_detection");
    ros::NodeHandle private_nh1("~");
    frontier_detection::frontierdetection ta(private_nh1);
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;
}



