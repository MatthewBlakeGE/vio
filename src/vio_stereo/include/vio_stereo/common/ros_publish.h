#ifndef ROS_PUBLISH_H
#define ROS_PUBLISH_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <fstream>
#include "common/camera_pose_visualization.h"
#include "common/parameters.h"
#include "estimator/estimator.h"

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern ros::Publisher pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle& n);  // 将节点n中的信息发布

void pubLatestOdometry(const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,
                       const Eigen::Vector3d& V, double t);

void pubTrackImage(const cv::Mat& imgTrack, const double t);

void printStatistics(const Estimator& estimator, double t);

void pubOdometry(const Estimator& estimator, const std_msgs::Header& header);

void pubInitialGuess(const Estimator& estimator,
                     const std_msgs::Header& header);

void pubKeyPoses(const Estimator& estimator, const std_msgs::Header& header);

void pubCameraPose(const Estimator& estimator, const std_msgs::Header& header);

void pubPointCloud(const Estimator& estimator, const std_msgs::Header& header);

void pubTF(const Estimator& estimator, const std_msgs::Header& header);

void pubKeyframe(const Estimator& estimator);

void pubRelocalization(const Estimator& estimator);

void pubCar(const Estimator& estimator, const std_msgs::Header& header);

#endif  // ROS_PUBLISH_H