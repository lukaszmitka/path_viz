//
// Created by lukasz on 24.06.16.
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher odom_cov_pub;
double cov_mat[36] = {1, 0.01, 0.01, 0.01, 0.01, 0.01,
                      0.01, 1, 0.01, 0.01, 0.01, 0.01,
                      0.01, 0.01, 1, 0.01, 0.01, 0.01,
                      0.01, 0.01, 0.01, 1, 0.01, 0.01,
                      0.01, 0.01, 0.01, 0.01, 1, 0.01,
                      0.01, 0.01, 0.01, 0.01, 0.01, 1};

nav_msgs::Path path_odom;
ros::Publisher path_odom_publisher;
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

   geometry_msgs::PoseStamped tmp_pose;
   tmp_pose.header.frame_id = "robot_base";
   tmp_pose.pose.position.x = msg->pose.pose.position.x;
   tmp_pose.pose.position.y = msg->pose.pose.position.y;
   tmp_pose.pose.position.z = 0;//tra.z();
   path_odom.poses.insert(path_odom.poses.end(), tmp_pose);
   path_odom_publisher.publish(path_odom);

   /*std::cout << "Odom received" << std::endl;
   nav_msgs::Odometry odom_with_cov;
   odom_with_cov.header = msg->header;
   odom_with_cov.child_frame_id = msg->child_frame_id;
   geometry_msgs::PoseWithCovariance poseWithCovariance;
   poseWithCovariance.covariance.fill(cov_mat[0]);
   //poseWithCovariance.covariance.elems=cov_mat;
   //msg->pose.covariance.elems = cov_mat;
   odom_cov_pub.publish(odom_with_cov);
   //ROS_INFO("I heard: [%s]", msg->data.c_str());*/
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {

}

int main(int argc, char **argv) {
   ros::init(argc, argv, "path_viz");
   ros::NodeHandle n("~");
   ros::Publisher path_map_publisher = n.advertise<nav_msgs::Path>("robot_path_map", 1);
   ros::Publisher path_world_publisher = n.advertise<nav_msgs::Path>("robot_path_world", 1);
   path_odom_publisher = n.advertise<nav_msgs::Path>("robot_path_odom", 1);
   nav_msgs::Path path_map, path_world;
   path_map.header.frame_id = "robot_base";
   path_world.header.frame_id = "robot_base";
   path_odom.header.frame_id = "robot_base";

   //ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);
   ros::Subscriber odom_sub = n.subscribe("/robot_ekf/odom_combined", 1, odomCallback);

   odom_cov_pub = n.advertise<nav_msgs::Odometry>("/odom_cov", 1);
   //ros::Subscriber imu_sub = n.subscribe("/imu_data", 1, imuCallback);
   //ros::Publisher imu_cov_pub = n.advertise<sensor_msgs::Imu>("/imu_cov", 1);

   tf::TransformListener listener;
   ros::Rate rate(10.0);
   while (n.ok()) {
      tf::StampedTransform transform;
      try {
         listener.lookupTransform("/map", "/robot_base", ros::Time(0), transform);
         tf::Vector3 tra = transform.getOrigin();
         //std::cout << "Transform: x=" << tra.x() << ", y=" << tra.y() << ", z=" << tra.z() <<
         //std::endl;
         geometry_msgs::PoseStamped tmp_pose;
         tmp_pose.header.frame_id = "robot_base";
         tmp_pose.pose.position.x = tra.x();
         tmp_pose.pose.position.y = tra.y();
         tmp_pose.pose.position.z = 0; //tra.z();
         path_map.poses.insert(path_map.poses.end(), tmp_pose);
         path_map_publisher.publish(path_map);

      }
      catch (tf::TransformException ex) {
         ROS_ERROR("%s", ex.what());
         ros::Duration(1.0).sleep();
      }

      try {
         listener.lookupTransform("/world", "/robot_base", ros::Time(0), transform);
         tf::Vector3 tra = transform.getOrigin();
         //std::cout << "Transform: x=" << tra.x() << ", y=" << tra.y() << ", z=" << tra.z() <<
         //std::endl;
         geometry_msgs::PoseStamped tmp_pose;
         tmp_pose.header.frame_id = "robot_base";
         tmp_pose.pose.position.x = tra.x();
         tmp_pose.pose.position.y = tra.y();
         tmp_pose.pose.position.z = 0;//tra.z();
         path_world.poses.insert(path_world.poses.end(), tmp_pose);
         path_world_publisher.publish(path_world);

         nav_msgs::Odometry odom_with_cov;
         std_msgs::Header h;
         h.frame_id = "/world";
         h.stamp = ros::Time::now();
         odom_with_cov.header = h;
         odom_with_cov.child_frame_id = "/robot_base";
         geometry_msgs::PoseWithCovariance poseWithCovariance;
         poseWithCovariance.pose.position.x = tra.x();
         poseWithCovariance.pose.position.y = tra.y();
         poseWithCovariance.pose.position.z = 0;
         poseWithCovariance.pose.orientation.x = transform.getRotation().x();
         poseWithCovariance.pose.orientation.y = transform.getRotation().y();
         poseWithCovariance.pose.orientation.z = transform.getRotation().z();
         poseWithCovariance.pose.orientation.w = transform.getRotation().w();
         poseWithCovariance.covariance.fill(0.1);
         //poseWithCovariance.covariance.assign(0.1);
         poseWithCovariance.covariance.at(0) = 1;
         poseWithCovariance.covariance.at(7) = 1;
         poseWithCovariance.covariance.at(14) = 1;
         poseWithCovariance.covariance.at(21) = 1;
         poseWithCovariance.covariance.at(28) = 1;
         poseWithCovariance.covariance.at(35) = 1;

         odom_with_cov.pose = poseWithCovariance;
         odom_cov_pub.publish(odom_with_cov);

         //std::cout << "Odom received " << poseWithCovariance.pose.position.x << " " <<
         //poseWithCovariance.pose.position.y << std::endl;
      }
      catch (tf::TransformException ex) {
         ROS_ERROR("%s", ex.what());
         ros::Duration(1.0).sleep();
      }
      ros::spinOnce();
      rate.sleep();
   }
}