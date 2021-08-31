#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>


namespace frontier_detection
{
    class frontierdetection
    {
        public:
            frontierdetection(ros::NodeHandle private_nh);
            void map_sub(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void particle_move(nav_msgs::OccupancyGrid& map, std::vector<int> indexes,int step);
            // void check_frontier(const nav_msgs::OccupancyGrid& occupancy_map,int start_idx,int &frontier_idx);
            void collion_check();
            void indexTomap(const nav_msgs::OccupancyGrid& map,int index,int& x,int& y);
            bool frontier_index_exist(const int frontier_point_index);//check whether the index of frontier is found before
            bool frontier_exist(const int candidate_frontier_index,const nav_msgs::OccupancyGrid& map);//check whether the point in candidate_frontier_index is frontier
            void frontier_find(const nav_msgs::OccupancyGrid& map,const int frontier_point_index);//find the frontier cells from frontier_point_index;
        private:
            bool getCostmap;
            std::vector<int> particle_indexes;
            std::vector<int> frontier;//store the frontier found
            std::vector<std::vector<int>> frontier_list;//store the frontier found
            std::vector<int> frontier_index_sum;//store all the index of frontier cells found
            std::vector<std::vector<int>> frontier_sum;
            ros::NodeHandle NODE;
            ros::Subscriber costmap_sub_global_;
            ros::Subscriber costmap_sub_temp_;
            ros::Publisher costmap_pub_temp_;
            ros::Publisher frontier_map_pub_;
            ros::Publisher robot_position_pub_;
            // ros::Publisher frontier_map_pub_;
            ros::Publisher costmap_pub_global_;
            nav_msgs::OccupancyGrid global_costmap;
            nav_msgs::OccupancyGrid local_costmap;
            geometry_msgs::PoseStamped global_pose;
            geometry_msgs::PoseStamped closest_frontier_point;
            std::pair<int,int> frontier_to_index;
            

            std::mutex lock_costmap;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            int robot_index;
            int frontier_temp=0;
            int frontier_count=0;
            int x_width=20;//sampling partle width in x axis
            int y_width=20;//sampling partle width in y axis
            int step=50;//particle movement step
            int free_value=0;
            int frontier_value=20;
            int obstacle_value=100;
            int particle_value=60;//particle value in costmap
            int unexplored_value=-1;
            int N=99;//using for generate 0-1 random num
            int min_frontier_size_=5;//minal cells in a frontier
            int closest_frontier_idx=0;
            int resolution=0.2;
            int frontier_index_sum_temp;//store the number of frontier cells
    };
}