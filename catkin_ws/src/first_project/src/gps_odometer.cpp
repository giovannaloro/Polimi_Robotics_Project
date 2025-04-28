#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"


const double a = 6378137;
const double b = 6356752;
double previous_e = 0;
double previous_n = 0;

class GPSOdometer{
public: 
	ros::Subscriber sub;
	ros::Publisher pub;
	tf::TransformBroadcaster tf_broadcaster;
	double lat_r_;
	double lon_r_;
	double alt_r_;
	double x_ref_, y_ref_, z_ref_;
	GPSOdometer() {

	ros::NodeHandle n;
	n.param("lat_r", lat_r_, 0.0);
	n.param("lon_r", lon_r_, 0.0);
	n.param("alt_r", alt_r_, 0.0);
	ROS_INFO("Reference GPS set: lat_r = %.6f, lon_r = %.6f, alt_r = %.2f", lat_r_, lon_r_, alt_r_);
	gpsToECEF(lat_r_* M_PI/180.0, lon_r_* M_PI/180.0, alt_r_, x_ref_, y_ref_, z_ref_);

	sub = n.subscribe("/swiftnav/front/gps_pose", 1, &GPSOdometer::gpsCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
 
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double e, n, u;
    gpsToOdom(msg->latitude, msg->longitude, msg->altitude, e, n, u);

    // Estimate orientation (yaw)
    float slope = (n - previous_n) / (e - previous_e);
    float theta = atan(slope);  // Angle in radians

	//update the previous north-east position
    previous_e = e;
    previous_n = n;

    // Convert to quaternion
    tf::Quaternion q;
    q.setRPY(0, 0, theta);  // Assuming we're working in 2D (yaw around the z-axis)

    // Prepare message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "gps";

    // Position
    odom_msg.pose.pose.position.x = e;
    odom_msg.pose.pose.position.y = n;
    odom_msg.pose.pose.position.z = u;

    // Orientation (set as quaternion)
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Publish Odometry message
    pub.publish(odom_msg);

    // Send transform with the updated orientation
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(e, n, u));
    transform.setRotation(q);  // Set the rotation as the quaternion

    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gps"));
}

void gpsToECEF(double lat, double lon, double alt, double& x, double& y, double& z){

	double e2 = 1-(pow(b,2)/pow(a,2));
	double N = a/sqrt(1-e2*sin(lat)*sin(lat));

	x = (N + alt)*cos(lat)*cos(lon);
	y = (N + alt)*cos(lat)*sin(lon);
	z = (N*(1-e2)+alt) * sin(lat);
}

void ecefToENU(double x_ecef, double y_ecef, double z_ecef, double& x_enu, double& y_enu, double& z_enu){
	double lat_rad = lat_r_ * M_PI/180.0;
	double lon_rad = lon_r_ * M_PI/180.0;

	double dx = x_ecef - x_ref_;
	double dy = y_ecef - y_ref_;
	double dz = z_ecef - z_ref_;

	x_enu = -sin(lon_rad)*dx + cos(lon_rad) * dy;
	y_enu = -sin(lat_rad) * cos(lon_rad) * dx - sin(lat_rad)*sin(lon_rad)*dy+cos(lat_rad)*dz;
	z_enu = cos(lat_rad)*cos(lon_rad)*dx + cos(lat_rad)*sin(lon_rad)*dy+sin(lat_rad)*dz;
}

void gpsToOdom(double lat, double lon, double alt, double& e, double& n, double& u){
	double x_ECEF, y_ECEF, z_ECEF;
	gpsToECEF(lat*M_PI/180.0, lon*M_PI/180.0, alt, x_ECEF, y_ECEF, z_ECEF);
	ecefToENU(x_ECEF, y_ECEF, z_ECEF, e, n, u); 
}

};
int main(int argc, char **argv){
	ros::init(argc, argv, "gps_odometer");
	ros::NodeHandle n;
	GPSOdometer gps_odometer;
  	ros::spin();

  return 0;
};

