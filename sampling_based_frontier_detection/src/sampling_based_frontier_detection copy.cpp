//todo: frontier classifaction
//time:0.01s
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
#include <stack>
#include <sampling_based_frontier_detection.h>
#include <boost/foreach.hpp>
#include <costmap_tools.h>
#include <chrono>

#define INF INT_MAX
using namespace std;
namespace frontier_detection
{
    auto start = std::chrono::high_resolution_clock::now();
    int count=0;
    frontierdetection::frontierdetection(ros::NodeHandle private_nh)
    {
        getCostmap = false;
        NODE = private_nh;
        costmap_sub_global_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &frontierdetection::map_sub, this);
        costmap_pub_temp_=private_nh.advertise<nav_msgs::OccupancyGrid>("/origin_map", 1);
        costmap_sub_temp_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/origin_map", 1, &frontierdetection::CostmapSubCallback_global, this);
        frontier_map_pub_=private_nh.advertise<nav_msgs::OccupancyGrid>("/frontier_map", 1);

        // frontier_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/frontier/goal", 1);
        // robot_position_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/robot_position", 1);
    }


    void frontierdetection::map_sub(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        cout<<"get map"<<endl;
        nav_msgs::OccupancyGrid map_temp=*map;
        while(1)
        {
            costmap_pub_temp_.publish(map_temp);
            sleep(1);
        }
    }

    void frontierdetection::CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        nav_msgs::OccupancyGrid map_temp=*map;
        for(int i=0;i<map_temp.info.height;i++)
        {
            if(i%y_width==0)
            {
            for(int j=0;j<map_temp.info.width;j++)
                {
                    if(j%x_width==0&&map_temp.data.at(i*map_temp.info.width+j)==0)
                    {
                        map_temp.data.at(i*map_temp.info.width+j)=particle_value;
                        particle_indexes.push_back(i*map_temp.info.width+j);
                    }
                }
            }
        }
        auto start = std::chrono::high_resolution_clock::now();
        while(count<3)
        {
            particle_move(map_temp,particle_indexes,step);
            vector <int>().swap(particle_indexes);
            cout<<"comparing frontier num:"<<frontier_index_sum_temp<<" "<<frontier_index_sum.size()<<endl;
            if(frontier_index_sum_temp==frontier_index_sum.size())
                count++;
            else
                continue;
        }   
        
        count=0;
        for(auto i:frontier_index_sum)
        {
            map_temp.data.at(i)=frontier_value;
        }
        frontier_map_pub_.publish(map_temp);
        auto end = std::chrono::high_resolution_clock::now();
        auto timespan=std::chrono::duration<double>(end - start);
        cout<<"                consuming time sum:"<<timespan.count()<<endl;

        // for(auto x:frontier_index_sum)
        //     cout<<x<<endl;
    }
    void frontierdetection::particle_move(nav_msgs::OccupancyGrid& map, std::vector<int> indexes,int step)
    {
        
        frontier_index_sum_temp=frontier_index_sum.size();
        auto start2 = std::chrono::high_resolution_clock::now();
        for(auto index:indexes)
        {
            auto start1 = std::chrono::high_resolution_clock::now();
            int mx,my;//store the (x,y) of index in map
            int temp_x,temp_y,temp_index;//store the temp point from index in map
            indexTomap(map,index,mx,my);
            // std::cout<<"mx:"<<mx<<std::endl;
            // std::cout<<"my:"<<my<<std::endl;

            // std::cout<<"k_x:"<<k_x<<"k_y:"<<k_y<<std::endl;
            std::vector<int> frontier;
            // std::cout<<"random:"<< rand() % (N + 1) / (float)(N + 1)-0.5<<std::endl;
            for(int i=0;i<step;i++)
            {
                float k_x=rand() % (N + 1) / (float)(N + 1)-0.5;
                float k_y=rand() % (N + 1) / (float)(N + 1)-0.5;
                temp_x=mx+i*k_x;
                temp_y=my+i*k_y;
                temp_index=temp_y*map.info.width+temp_x;
                if(map.data.at(temp_index)==free_value)
                    // continue;
                    map.data.at(temp_index)=particle_value;
                else if(map.data.at(temp_index)==unexplored_value)
                {
                    // map.data.at(temp_index)=frontier_value;
                    // std::cout<<"enter frontier_find1"<<endl;
                    if(!frontier_index_exist(temp_index))
                    {
                        // std::cout<<"enter frontier_find2"<<endl;
                        frontier_find(map,temp_index);
                    }
                    // std::vector<unsigned int> x=nhood4(temp_index,map);
                    break;
                }
            }
            auto end1 = std::chrono::high_resolution_clock::now();
            auto timespan=std::chrono::duration<double>(end1 - start1);
            cout<<"per loop consuming time:"<<timespan.count()<<endl;
        }
            auto end2 = std::chrono::high_resolution_clock::now();
            auto timespan=std::chrono::duration<double>(end2 - start2);
            cout<<"loop consuming time sum:"<<timespan.count()<<endl;

        // vector <int>().swap(frontier_index_sum);//check the single epid of frontiers
    }

    void frontierdetection::frontier_find(const nav_msgs::OccupancyGrid& map,const int frontier_point_index)
    {
        // std::cout<<"enter frontier_find"<<endl;
        std::queue<int> bfs;
        bfs.push(frontier_point_index);
        while (!bfs.empty())
        {
            int idx = bfs.front();
            bfs.pop();
            // std::cout<<"frontier_exist(idx,map):"<<frontier_exist(idx,map)<<endl;
            // std::cout<<"frontier_index_exist(idx)"<<frontier_index_exist(idx)<<endl;
            if(frontier_exist(idx,map)&&!frontier_index_exist(idx))
            {
                // std::cout<<"frontier_exist"<<std::endl;
                // std::cout<<"idx_value"<<map.data.at(idx)<<std::endl;
                frontier_index_sum.push_back(idx);
                frontier.push_back(idx);
                BOOST_FOREACH(int nbr, nhood8(idx, map))
                {
                    if(map.data.at(nbr)==unexplored_value&&!frontier_index_exist(nbr))
                        bfs.push(nbr);
                }
            }
            // else
            //     continue;
        }
            if (frontier.size() > min_frontier_size_)
            {
                frontier_list.push_back(frontier);
            }
        // std::cout<<"frontier_find_end"<<std::endl;
    }

    bool frontierdetection::frontier_index_exist(const int frontier_point_index)
    {
        auto exist=find(frontier_index_sum.begin(),frontier_index_sum.end(),frontier_point_index);
        if(exist==frontier_index_sum.end())
        {
            return false;
        }
        else
            return true;
    }
    bool frontierdetection::frontier_exist(const int candidate_frontier_index,const nav_msgs::OccupancyGrid& map)
    {
        BOOST_FOREACH(int nbr, nhood8(candidate_frontier_index, map))
        {
            // std::cout<<"nbr:"<<nbr<<std::endl;
            if(map.data.at(nbr)==free_value&&map.data.at(candidate_frontier_index)==unexplored_value)
                return true;
        }
        return false;
    }
    void frontierdetection::collion_check()
    {

    }
    void frontierdetection::indexTomap(const nav_msgs::OccupancyGrid& map,int index,int& x,int& y)
    {
        x=index%map.info.width;
        y=index/map.info.width;
    }
}

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_detection");
    ros::NodeHandle private_nh1("~");
    frontier_detection::frontierdetection ta(private_nh1);
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;
}



