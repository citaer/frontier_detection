#include <ros/ros.h> 
#include <nav_msgs/OccupancyGrid.h>

int main (int argc, char **argv) 
{ 
    // 初始化ROS节点 节点名字
	ros::init (argc, argv, "mappublish"); 
	
    // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid> ("mappublish", 1);

    // 定义地图对象
    nav_msgs::OccupancyGrid myMap;

    // 地图数据 
    myMap.info.map_load_time = ros::Time(0);
    myMap.info.resolution = 0.5;
    myMap.info.width = 10;
    myMap.info.height = 10;
    myMap.info.origin.position.x = 0;
    myMap.info.origin.position.y = 0;
    myMap.info.origin.position.z = 0;
    myMap.info.origin.orientation.x = 0;
    myMap.info.origin.orientation.y = 0;
    myMap.info.origin.orientation.z = 0;
    // myMap.info.origin.orientation.w = 0;
    for (std::size_t i = 0; i < 10; i++)
    {
        int8_t a;
        if (i <= 6)
            a = 20 * i;
        else if (i == 7)
            a = -10;
        else
            a = i;
        myMap.data.push_back(a);
    }

    // frame id
    myMap.header.frame_id = "map";

    // 消息发布频率
    ros::Rate loop_rate(1);

	while (ros::ok()) 
	{ 
        // 广播
		pub.publish(myMap);
	    ros::spinOnce(); 
		loop_rate.sleep(); 
	 } 
	return 0; 
}
