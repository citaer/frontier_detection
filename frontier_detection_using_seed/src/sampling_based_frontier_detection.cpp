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
#include <random>

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
                costmap_sub_global_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &frontierdetection::map_sub, this);
                costmap_pub_global_=private_nh.advertise<nav_msgs::OccupancyGrid>("/sampling_partles_map", 1);
                costmap_sub_global_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/sampling_partles_map", 1, &frontierdetection::CostmapSubCallback_global, this);
                // frontier_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/frontier/goal", 1);
                // robot_position_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/robot_position", 1);
            }


        private:
            void map_sub(const nav_msgs::OccupancyGrid::ConstPtr& map)
            {
                cout<<"get map"<<endl;
                nav_msgs::OccupancyGrid map_temp=*map;
                int x_step=10;
                int y_step=10;
                for(int i=0;i<map_temp.info.height;i++)
                {
                    if(i%y_step==0)
                    {
                    for(int j=0;j<map_temp.info.width;j++)
                        {
                            if(j%x_step==0&&map_temp.data.at(i*map_temp.info.width+j)==0)
                                map_temp.data.at(i*map_temp.info.width+j)=80;
                                particle_indexes.push_back(i*map_temp.info.width+j);
                        }
                    }
                }
                while(1)
                {
                    sleep(1);
                    costmap_pub_global_.publish(map_temp);
                }
            }

            void CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map)
            {
                nav_msgs::OccupancyGrid map_temp=*map;
                partle_move(map_temp,particle_indexes);
            }
            void partle_move(const nav_msgs::OccupancyGrid& map, std::vector<int> indexes)
            {
                for(auto index:indexes)
                {
                    int mx,my;
                    indexTomap(map,index,mx,my);
                    
                }
            }

            void collion_check()
            {

            }
            void indexTomap(const nav_msgs::OccupancyGrid& map,int index,int& x,int& y)
            {
                y=index/map.info.width;
                x=index%map.info.width;
            }




    // void find_closest_frontier(const nav_msgs::OccupancyGrid& occupancy_map,int start_idx,int &frontier_idx) 
    // {
    //     int a[4]={1,(int)occupancy_map.info.width,-1,-(int)occupancy_map.info.width};
    //     std::unordered_map<int, int> visited, unvisited;
    //     std::queue<int> id;
    //     int current;
    //     if(occupancy_map.data[start_idx]<97)
    //     {
    //         id.push(start_idx);
    //         while (id.size()) 
    //         {
    //             current = id.front();
    //             id.pop();
    //             for(int i=0;i<4;i++)
    //             {
    //                 int neighbor_id=current+a[i];
    //                 if(neighbor_id>=0&&neighbor_id<occupancy_map.info.width*occupancy_map.info.height)
    //                     {
    //                             if((occupancy_map.data[neighbor_id]<97)&&!(visited[neighbor_id]))
    //                             {
    //                                 if(occupancy_map.data[current]==-1)
    //                                 {
    //                                     frontier_idx=current;
    //                                     id=queue<int>();
    //                                     break;
    //                                 }
    //                                     id.push(neighbor_id);
    //                                     ++visited[neighbor_id];
    //                             }
    //                     }
    //             }
    //         }
    //     }
    // }
            
    //         void GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_pose, ros::Time timestamp)
    //         {
    //         tf::StampedTransform transform;
    //         geometry_msgs::PoseStamped iden_pose;
    //         iden_pose.header.frame_id = iden_frame_id;
    //         iden_pose.header.stamp = ros::Time::now(); 
    //         iden_pose.pose.orientation.w = 1;
    //         tf_listener_.waitForTransform("/robot0/map",iden_frame_id, ros::Time(0), ros::Duration(2.0));
    //         tf_listener_.lookupTransform( "/robot0/map",iden_frame_id, ros::Time(0), transform);
    //         global_pose.pose.position.x=transform.getOrigin().x();
    //         global_pose.pose.position.y=transform.getOrigin().y();
    //         global_pose.pose.position.z=transform.getOrigin().z();
    //         }

            bool getCostmap;
            std::vector<int> particle_indexes;
            ros::NodeHandle NODE;
            ros::Subscriber costmap_sub_global_;
            ros::Publisher frontier_pub_;
            ros::Publisher robot_position_pub_;
            // ros::Publisher frontier_map_pub_;
            ros::Publisher costmap_pub_global_;
            nav_msgs::OccupancyGrid global_costmap;
            nav_msgs::OccupancyGrid local_costmap;
            geometry_msgs::PoseStamped global_pose;
            geometry_msgs::PoseStamped closest_frontier_point;
            

            std::mutex lock_costmap;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            int robot_index;
            int seed;
            int frontier_temp=0;
            int frontier_count=0;
            int closest_frontier_idx=0;
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



