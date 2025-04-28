#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sstream>

//define wheelbase 
const double WHEELBASE = 1.765;
//define steering factor
const double STEERING_FACTOR = 0.031;

// Global state
double x = 0.0, y = 0.0, theta = 1.57;
double lat = 0.0, lon = 0.0;
double dt = 0.05;

//define publisher
ros::Publisher odometer_pub;

void odometerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    double velocity = msg->point.y*0.28;       // Forward velocity [m/s]
    double steering_angle = msg->point.x*0.01744444444*STEERING_FACTOR; // Steering angle [rad]
    static tf::TransformBroadcaster tf_broadcaster;

    //get time 
    ros::Time current_time = ros::Time::now();

    // Bicycle model kinematics
    double delta_x = velocity * cos(theta) * dt;
    double delta_y = velocity  * sin(theta) * dt;
    double delta_theta = (velocity / WHEELBASE) * tan(steering_angle) * dt;

    //compute odometry
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    // Quaternion for orientation
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "vehicle";
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;
    tf_broadcaster.sendTransform(odom_tf);

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = current_time;
    odom_msg.child_frame_id = "base_link";

    // Pose
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    // Velocity
    odom_msg.twist.twist.linear.x = velocity;
    odom_msg.twist.twist.angular.z = delta_theta / dt;  // Angular velocity
    odometer_pub.publish(odom_msg);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;
    odometer_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber odometer_sub = n.subscribe("/speedsteer", 10, odometerCallback);
    ros::spin();
  	return 0;
}