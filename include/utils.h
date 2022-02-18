#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "common.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h> 




std::vector<Eigen::Vector3d> Mat3XdToVec3d(Eigen::Matrix3Xd LidarPoints);
Eigen::Vector3d ToVec3(Eigen::Matrix3d rot);
Eigen::Vector3f ToVec3(Eigen::Matrix3f rot);
Eigen::Matrix3d ToMat33(Eigen::Vector3d rod);
Eigen::Matrix3f ToMat33(Eigen::Vector3f rod);
Vector6d To6DOF(Eigen::Quaterniond q, Eigen::Vector3d t);
Vector6d To6DOF(Eigen::Matrix4d RT);
Eigen::Quaterniond ToQuaternion(const Vector6d Pose);
Eigen::Matrix4f To44RT(Vector6f pose);
Eigen::Matrix4d To44RT(Vector6d pose);
Eigen::Matrix4d To44RT(std::vector<double> pose);
Vector6d RelativePose(std::map< int, Vector6d > Odometry_WPose, int a, int b);
double ToAngle(Eigen::Matrix4d LidarRotation);
Eigen::Vector3d ToAxis(Eigen::Matrix4d LidarRotation);
void PublishPointCloud( const ros::Publisher &publisher, 
                        const std::vector<Eigen::Vector3d> &pointclouds, 
                        const ros::Time &timestamp, 
                        const std::string &frameid);
sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud);
std::vector<Eigen::Vector3d> ConvertFromROSmsg(sensor_msgs::PointCloud2 &PointCloud);
float VerticalAngle(Eigen::Vector3d p);
double PointDistance(Eigen::Vector3d p);
double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);
double CosRaw2(double a, double b, float ang);

