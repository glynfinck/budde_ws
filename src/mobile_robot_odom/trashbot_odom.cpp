#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

double qe1 = 0;
double qe2 = 0;
double dt = 0;
double dx = 0;
double dy = 0;
double dth = 0;
double dist = 0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dxp, dyp, vartheta, rcurv;
ros::Time current_time;

ros::Publisher odom_pub;
 
void orbPoseCallback(const geometry_msgs::PoseStamped& msg){

  current_time = ros::Time::now();

  //first, we'll publish the transform over tf
  
  tf::TransformBroadcaster odom_broadcaster;
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x =  msg.pose.position.x;
  odom_trans.transform.translation.y =  msg.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = msg.pose.orientation;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
  
  nav_msgs::Odometry odom; //create nav_msgs::odometry 
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  
  odom.pose.pose.position.x = msg.pose.position.x; //set positions 
  odom.pose.pose.position.y = msg.pose.position.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = msg.pose.orientation;
  
  odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;
  
  odom_pub.publish(odom);  //publish odom message
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  tf::TransformBroadcaster odom_broadcaster;
  ros::NodeHandle nh; 
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber orb_pose_sub = nh.subscribe("orb_slam2_rgbd/pose",50,orbPoseCallback);

  
  ros::spin();
  return 0;
}
