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

namespace frontier_detection
{
    class node
    {
    public:
        int obstacle, start;
        int id;
        int width;
        int height;
        int value;
        int x, y;
        int cost;

        std::vector<node*> neighbors;
        node* parent;

        node()
        {
            this->obstacle = 0;
            this->start = 0;
            this->parent = nullptr;
            this->cost = INF;
        }
        node(int id,int width,int height,int value)
        {
            this->id=id;
            this->height=height;
            this->width=width;
            this->value=value;
            this->x=id%width;
            this->y=id/width;
        }
        ~node()
        {

        }

    };


    int getCost(node* one, node* two) {
        int dist = abs(one->x - two->x) + abs(one->y - two->y);

        return ((one->obstacle || two->obstacle) ? INF : dist);
    }



    void updateNodes(std::vector<std::vector<node*> >& map, node* startNode,int &frontier_idx) 
    {
        std::queue<node*> nodeQ;
        std::unordered_map<node*, int> visited, unvisited;
        node* current;
        // std::cout<<current->
        if(!startNode->obstacle)
        {
        nodeQ.push(startNode);
        while (nodeQ.size()) 
        {
            current = nodeQ.front();
            nodeQ.pop();
            for (node* n : current->neighbors) {
                if (!(n->obstacle) && !(visited[n])) {
                    if (current->cost + getCost(current, n) < n->cost) {
                        n->cost = current->cost + getCost(current, n);
                        n->parent = current;
                    }
                    if (!unvisited[n]) {
                        nodeQ.push(n);
                        ++unvisited[n];
                    }
                    if (n->value==-1)
                    {
                        frontier_idx=current->id;
                        nodeQ=std::queue<node*>();
                        break;
                    }
            }
            }
            if (visited[current] == 0) {
                ++visited[current];
            }
        }
        }
        // for(auto p:map)
        // {
        //     for(auto q:p)
        //     {
        //     delete q;
        //     p.clear();
        //     }
        // }
        // map.clear(); 

    }
    void initMap(std::vector<std::vector<node*>>& map, int mapWidth, int mapHeight,const nav_msgs::OccupancyGrid& occupancy_map, int start_id) 
    {
        for (int i = 0; i < mapWidth*mapHeight; ++i) 
        {
                int y=i/mapWidth;
                int x=i%mapHeight;
                map[y][x] = new node(i,mapWidth,mapHeight,occupancy_map.data[i]);
        }

        for (int x = 0; x < mapWidth; ++x) 
        {

            for (int y = 0; y < mapHeight; ++y) 
            {
                if(map[y][x]->value>97)
                {
                    map[y][x]->obstacle = 1;
                }
                else
                {
                    map[y][x]->obstacle = 0;
                }
                if (y < mapHeight - 1) {
                    map[y][x]->neighbors.push_back(map[y + 1][x]);
                }
                if (x < mapWidth - 1) {
                    map[y][x]->neighbors.push_back(map[y][x + 1]);
                }
                if (y > 0) {
                    map[y][x]->neighbors.push_back(map[y - 1][x]);
                }
                if (x > 0) {
                    map[y][x]->neighbors.push_back(map[y][x - 1]);
                }
            }
        }
        int start_x=start_id/mapWidth;
        int start_y=start_id/mapHeight;
        map[start_y][start_x]->start = 1;
        map[start_y][start_x]->cost = 0;
    }

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
                std::vector<std::vector<node* > > map1 (mapHeight, std::vector<node*> (mapWidth));
                int a[3];
                a[0]=(global_pose.pose.position.x+global_costmap.info.width*global_costmap.info.resolution/2)*100000;
                a[1]=(global_pose.pose.position.y+global_costmap.info.height*global_costmap.info.resolution/2)*100000;
                a[2]=global_costmap.info.resolution*100000;
                robot_index=(a[1]/a[2])*global_costmap.info.width+a[0]/a[2];
                // std::cout<<"global_pose:"<<global_pose.pose.position.x<<" "<<global_pose.pose.position.y<<std::endl;
                robot_position_pub_.publish(global_pose);
                find_closest_frontier(map1,global_costmap,robot_index,closest_frontier_idx);

                // std::cout<<"robot_idx:"<<robot_index<<std::endl;
                // std::cout<<"global_costmap.info.origin.position:"<<global_costmap.info.origin.position.x<<" "<<global_costmap.info.origin.position.y<<std::endl;
                // std::cout<<"global_pose:"<<global_pose.pose.position.x<<" "<<global_pose.pose.position.y<<std::endl;
                // std::cout<<"closest_frontier_idx:"<<closest_frontier_idx<<std::endl;
                closest_frontier_point.header.frame_id = "/robot0/map";
                closest_frontier_point.header.stamp = ros::Time::now();
                closest_frontier_point.pose.orientation.w = 1;
                closest_frontier_point.pose.position.x=closest_frontier_idx%global_costmap.info.width*global_costmap.info.resolution+global_costmap.info.origin.position.x;
                closest_frontier_point.pose.position.y=closest_frontier_idx/global_costmap.info.width*global_costmap.info.resolution+global_costmap.info.origin.position.y;
                closest_frontier_point.pose.position.z=0;
                // std::cout<<"frontier_pose:"<<closest_frontier_point.pose.position.x<<" "<<closest_frontier_point.pose.position.y<<std::endl;
                frontier_pub_.publish(closest_frontier_point);
                for(auto p:map1)
                {
                    for(auto q:p)
                    {
                    delete q;
                    p.clear();
                    // q=nullptr;
                    }
                    // p=nullptr;
                    p.clear(); 
                    // std::vector<node*>().swap(p);
                }
                map1.clear();

            }



            void find_closest_frontier(std::vector<std::vector<node* > > &map,const nav_msgs::OccupancyGrid& global_costmap,const int idx,int& closest_frontier_idx)
            {
                clock_t start, finish;//timer
                double duration;
                start = clock();
                int mapWidth=global_costmap.info.width;
                int mapHeight=global_costmap.info.height;
                int start_idx=idx;
                
                initMap(map, mapWidth, mapHeight,global_costmap, start_idx);
                finish = clock();
                duration = (double)(finish - start) / CLOCKS_PER_SEC;
                std::cout<<"The run time is:"<<duration<<std::endl;
                int start_x=start_idx%mapWidth;
                int start_y=start_idx/mapWidth;
                node* startNode = map[start_y][start_x];
                // std::cout<<"x:"<<startNode->x<<"y:"<<startNode->y<<" sum:"<<startNode->x+startNode->y*global_costmap.info.width<<std::endl;
                // std::cout<<"map[[start_y][start_x]:"<<map[start_y][start_x]->obstacle<<std::endl;
                // std::cout<<"id:"<<start_idx<<std::endl;
                // node* goalNode = map[goal[1]][goal[0]];
                updateNodes(map, startNode,closest_frontier_idx);
                // for(auto p:map)
                // {
                //     for(auto q:p)
                //     {
                //     delete q;
                //     }
                //     p.clear();
                //     // std::vector<node*>().swap(p);
                // }
                // std::vector<std::vector<node* > > temp (mapHeight, std::vector<node*> (mapWidth));
                // map.swap(temp);
                // map.clear(); 
                // std::vector<std::vector<node*>>().swap(map);


                
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

            // Costmap2DROS* cmap_;
            bool getCostmap;
            ros::NodeHandle NODE;
            // tf::TransformListener tf_listener_;
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



