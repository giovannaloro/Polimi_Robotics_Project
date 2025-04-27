#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "first_project/secotor_times.h"
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

float mean_speed;
float sector_time;

//define publisher
ros::Publisher sector_pub;

//define main track point 
float lat_13



void sectorCallback(const geometry_msgs::PointStamped::ConstPtr& msg1, const  sensor_msgs::NavSatFix::ConstPtr& msg2){
 ROS_INFO("Message arrived");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle n;
    sector_pub = n.advertise<first_project::secotor_times>("/sector_times", 10);
    message_filters::Subscriber<geometry_msgs::PointStamped> sector_sub1(n, "/speedsteer", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sector_sub2(n, "/swiftnav/front/gps_pose", 10);
   
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,  sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sector_sub1, sector_sub2);
    sync.registerCallback(boost::bind(&sectorCallback, _1, _2));

    ros::spin();
  	return 0;
}