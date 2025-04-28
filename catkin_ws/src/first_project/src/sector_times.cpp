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

//define publisher
ros::Publisher sector_pub;

//define time granularity 
float dt = 0.05;


// Section 1/3
float lat_Sec1_3 = 45.613760;
float lon_Sec1_3 = 9.280715;

// Section 1/2
float lat_Sec1_2 = 45.630132;
float lon_Sec1_2 = 9.290356;

// Section 2/3
float lat_Sec2_3 = 45.623147;
float lon_Sec2_3 = 9.286618;

// PC
float lat_PC = 45.611905;
float lon_PC = 9.282614;

//define sectors
enum sector{
  SECTOR_1 = 0,
  SECTOR_2 = 1,
  SECTOR_3 = 2
};

//define return callback persistent values
float current_sector_time = 0;
float current_sector_mean_speed = 0; 
int count = 0;
int current_sector = SECTOR_1;
int previous_sector = SECTOR_1;


void sectorCallback(const geometry_msgs::PointStamped::ConstPtr& msg1, const  sensor_msgs::NavSatFix::ConstPtr& msg2){

  //define output message 
  first_project::secotor_times sector_msg;

  //current_sector identification;
  float current_lat = msg2.latitude;
  float current_lon = msg2->longitude;
  if (current_lon >= lon_Sec1_2) {current_sector = SECTOR_2;}
  else if (current_lat <= lat_Sec1_3 ) {current_sector = SECTOR_3;}
  else if (current_lat >= lat_Sec1_3 && current_lon <= lon_PC) {current_sector = SECTOR_1;}
  else if (current_lat <= lat_Sec2_3 && current_lon >= lon_PC ) {current_sector = SECTOR_3;}
  else if (current_lat <= lat_Sec1_2 && current_lat >= lat_Sec2_3 && current_lon >= lon_Sec2_3) {current_sector = SECTOR_2;}
  else {current_sector = SECTOR_1;}

  //check sector changes and update persistent values 
  if (current_sector == previous_sector){
    //update time, average and count 
    current_sector_time += dt;
    current_sector_mean_speed = ((current_sector_mean_speed*count)+(msg1->point.y*0.28)) / (count+1);
    ++count;
  }
  else{
    //reset persistent variables
    previous_sector = current_sector;
    count = 1;
    current_sector_mean_speed = msg1->point.y*0.28;
    current_sector_time = dt;
  }

  //debug
  ROS_INFO("Time: %.2f | Current Sector: %d | Previous Sector: %d | Mean Speed: %.2f", current_sector_time, current_sector, previous_sector, current_sector_mean_speed);

  //send message
  sector_msg.current_sector = current_sector;
  sector_msg.current_sector_mean_speed = current_sector_mean_speed;
  sector_msg.current_sector_time = current_sector_time;
  sector_pub.publish(sector_msg);
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