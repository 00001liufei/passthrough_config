#pragma once
#include <ros/ros.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "velodyne_pcl/point_types.h"
#include "yaml.hpp"

void sigintHandler(int sig)
{
    std::cout << "Shutting down!"
              << "\n";
    ros::shutdown();
}

sensor_msgs::PointCloud2Ptr toRos(pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr in, std::string &frame_name)
{
    sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*in, *out);
    out->header.frame_id = frame_name;
    out->header.stamp = ros::Time::now();
    return out;
}

std::string toYaml(velodyne_pcl::PointXYZIRT const &point)
{
    std::string out;
    out += "  - {x: " + std::to_string(point.x);
    out += ", y: " + std::to_string(point.y);
    out += ", z: " + std::to_string(point.z);
    out += ", intensity: " + std::to_string(point.intensity);
    out += ", ring: " + std::to_string(point.ring);
    out += ", time: " + std::to_string(point.time);
    out += "}\n";

    return std::move(out);
}

void fromYaml(const YAML::Node &node, velodyne_pcl::PointXYZIRT &point)
{
    point.x = node["x"].as<float>();
    point.y = node["y"].as<float>();
    point.z = node["z"].as<float>();
    point.intensity = node["intensity"].as<float>();
    point.ring = node["ring"].as<uint16_t>();
    point.time = node["time"].as<float>();
}

std::string toYaml(tf::StampedTransform &t)
{
    tf::Vector3 translation = t.getOrigin();
    tf::Quaternion quaternion = t.getRotation();

    std::string out;
    out += " parent: " + t.frame_id_ + "\n";
    out += " child: " + t.child_frame_id_ + "\n";
    out += " translation: [";
    out += std::to_string(translation.x()) + ", ";
    out += std::to_string(translation.y()) + ", ";
    out += std::to_string(translation.z()) + "]\n";
    out += " rotation: [";
    out += std::to_string(quaternion.w()) + ", ";
    out += std::to_string(quaternion.x()) + ", ";
    out += std::to_string(quaternion.y()) + ", ";
    out += std::to_string(quaternion.z()) + "]\n";

    return std::move(out);
}

void fromYaml(const YAML::Node &node, tf::StampedTransform &tf)
{
    // tf::StampedTransform tf;
    tf.frame_id_ = node["parent"].as<std::string>();
    tf.child_frame_id_ = node["child"].as<std::string>();

    tf::Vector3 translation;
    translation.setZero();
    translation.setX(node["translation"][0].as<double>());
    translation.setY(node["translation"][1].as<double>());
    translation.setZ(node["translation"][2].as<double>());

    tf::Quaternion quaternion;
    quaternion.setW(node["rotation"][0].as<double>());
    quaternion.setX(node["rotation"][1].as<double>());
    quaternion.setY(node["rotation"][2].as<double>());
    quaternion.setZ(node["rotation"][3].as<double>());

    tf.setOrigin(translation);
    tf.setRotation(quaternion);

    tf.stamp_ = ros::Time::now();

    // return std::move(tf);
}

std::string toYaml(const pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr &cloud)
{
    std::string out;
    out += "Clouds: \n";
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        out += " - {x: " + std::to_string(cloud->points[i].x);
        out += ", y: " + std::to_string(cloud->points[i].y);
        out += ", z: " + std::to_string(cloud->points[i].z);
        out += "}\n";
    }

    return out;
}

Eigen::Vector3f toVector3f(velodyne_pcl::PointXYZIRT const &in)
{
    Eigen::Vector3f vec(in.x, in.y, in.z);
    return std::move(vec);
}

bool readCalibrationYml(const std::string &in, sensor_msgs::CameraInfo &cam_info)
{
    try
    {
        YAML::Node doc = YAML::LoadFile(in);

        cam_info.width = doc["image_width"].as<uint32_t>();
        cam_info.height = doc["image_height"].as<uint32_t>();
        cam_info.distortion_model = doc["distortion_model"].as<std::string>();

        const YAML::Node &D_node = doc["camera_matrix"];
        int D_rows, D_cols;
        D_rows = D_node["rows"].as<int>();
        D_cols = D_node["cols"].as<int>();
        const YAML::Node &D_data = D_node["data"];
        cam_info.D.resize(D_rows * D_cols);
        for (int i = 0; i < D_rows * D_cols; ++i)
            cam_info.D[i] = D_data[i].as<double>();

        return true;
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR("Exception parsing YAML camera calibration:\n%s", e.what());
        return false;
    }
}

int to_int(const std::string &s)
{
    int result = 0;
    for (size_t i = 0; i <= s.size() - 1; i++)
    {
        if (s[i] >= '0' && s[i] <= '9')
        {
            result = result * 10 + s[i] - 48;
        }
    }

    return result;
}
