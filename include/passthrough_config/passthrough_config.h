#pragma once
#define PCL_NO_PRECOMPILE
#include <iostream>
#include <string>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/forwards.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <velodyne_pcl/point_types.h>

#include <std_srvs/Empty.h>

#include <passthrough_config/SendRPY.h>
#include <passthrough_config/SendFieldX.h>
#include <passthrough_config/SendFieldY.h>
#include <passthrough_config/SendFieldZ.h>
#include <passthrough_config/LoadConfig.h>

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "yaml.hpp"

namespace passthrough_config
{
    namespace
    {
        void do_passThrough(passthrough_filter::passthroughFieldConfig const &config,
                            pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr cloud)
        {
            pcl::PassThrough<velodyne_pcl::PointXYZIRT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName(config.dim);
            pass.setFilterLimits(config.min, config.max);
            pass.setFilterLimitsNegative(false);
            pass.setKeepOrganized(false);
            pass.filter(*cloud);
        }

        void passThrough(passthrough_filter::passthroughConfig const &config, pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr filtered_cloud)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                do_passThrough(config.field[i], filtered_cloud);
            }
        }

        void computeTransform(const passthrough_filter::passthroughConfig &config, Eigen::Isometry3f &T)
        {
            // Eigen::AngleAxisf rollAngle(config.rotation.roll * M_PI / 180, Eigen::Vector3f::UnitZ());
            // Eigen::AngleAxisf pitchAngle(config.rotation.pitch * M_PI / 180, Eigen::Vector3f::UnitX());
            // Eigen::AngleAxisf yawAngle(config.rotation.yaw * M_PI / 180, Eigen::Vector3f::UnitY());

            Eigen::AngleAxisf rollAngle(config.rotation.roll * M_PI / 180, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(config.rotation.pitch * M_PI / 180, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(config.rotation.yaw * M_PI / 180, Eigen::Vector3f::UnitZ());

            Eigen::Quaternionf qua = rollAngle * yawAngle * pitchAngle;

            T.linear() = qua.toRotationMatrix();
            // std::cout << T.linear().matrix() << "\n";
        }

        inline std::string toYaml(const passthrough_filter::passthroughConfig &config)
        {
            std::string out;
            out += "field: \n";
            for (size_t i = 0; i < 3; ++i)
            {
                out += " - {dim: \"" + config.field[i].dim + "\"";
                out += ", min: ";
                out += std::to_string(config.field[i].min);
                out += ", max: ";
                out += std::to_string(config.field[i].max);
                out += "}\n";
            }

            out += "rotation: \n";
            out += " {roll: ";
            out += std::to_string(config.rotation.roll);
            out += ", pitch: ";
            out += std::to_string(config.rotation.pitch);
            out += ", yaw: ";
            out += std::to_string(config.rotation.yaw);
            out += "}\n";

            return out;
        }

    } // namespace

    class PassthroughConfig
    {
    public:
        PassthroughConfig(ros::NodeHandle &nh) : nh_(nh)
        {
            ROS_INFO_STREAM("Passthrough filter node initialized.");

            std_srvs::Empty::Request req = std_srvs::EmptyRequest();
            std_srvs::Empty::Response res = std_srvs::EmptyResponse();

            reset(req, res);

            std::string package_path = ros::package::getPath("passthrough_config");

            nh_.param<std::string>("topic", topic_, "/velodyne_points");
            nh_.param<std::string>("save_config_path", save_config_path_, package_path + "/config");

            service_rpy_ = nh_.advertiseService("modify_rpy", &PassthroughConfig::modifyRPYValue, this);
            service_field_x_ = nh_.advertiseService("modify_field_x", &PassthroughConfig::modifyFieldxValue, this);
            service_field_y_ = nh_.advertiseService("modify_field_y", &PassthroughConfig::modifyFieldyValue, this);
            service_field_z_ = nh_.advertiseService("modify_field_z", &PassthroughConfig::modifyFieldzValue, this);
            service_save_ = nh_.advertiseService("save", &PassthroughConfig::saveToYaml, this);
            service_reset_ = nh_.advertiseService("reset", &PassthroughConfig::reset, this);
            service_show_param_ = nh_.advertiseService("show", &PassthroughConfig::show, this);
            service_load_ = nh_.advertiseService("load", &PassthroughConfig::load, this);

            // publish a topic showing in rviz
            // pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
            ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
                "raw_cloud",
                1,
                boost::bind(&PassthroughConfig::cloudConnect, this),
                boost::bind(&PassthroughConfig::cloudDisConnect, this),
                ros::VoidPtr(), &callback_queue_);
            pub_cloud_ = nh_.advertise(ao);
            pub_filtered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);

            callback_queue_thread_ = boost::thread(boost::bind(&PassthroughConfig::queueThread, this));
        }

    private:
        ros::NodeHandle nh_;

        passthrough_filter::passthroughConfig config_;
        Eigen::Isometry3f rotation_transform_;

        std::string save_config_path_;
        std::string config_name_;

        std::string topic_;

        ros::ServiceServer service_rpy_;
        ros::ServiceServer service_field_x_;
        ros::ServiceServer service_field_y_;
        ros::ServiceServer service_field_z_;
        ros::ServiceServer service_reset_;
        ros::ServiceServer service_save_;
        ros::ServiceServer service_show_param_;
        ros::ServiceServer service_load_;

        ros::Publisher pub_cloud_;
        ros::Publisher pub_filtered_cloud_;

        ros::Subscriber sub_;

        boost::thread thread_filter_;
        ros::CallbackQueue callback_queue_;
        boost::thread callback_queue_thread_;

        void queueThread()
        {
            static const double TIMEOUT = 0.01;

            while (nh_.ok())
            {
                callback_queue_.callAvailable(ros::WallDuration(TIMEOUT));
            }
        }

        void cloudConnect()
        {
            ROS_INFO("Connect build.");
            // a thread for filtering the cloud and publishing topic

            // TODO: topic
            ROS_INFO("Start filtering ...");
            sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(topic_, 5, boost::bind(&PassthroughConfig::handleTopicCloud, this, _1));
        }

        void cloudDisConnect()
        {
            ROS_INFO("Disconnect");
            // close thread

            // sub_.shutdown();
            sub_.~Subscriber();
        }

        void handleTopicCloud(sensor_msgs::PointCloud2::ConstPtr const &cloud_msg)
        {

            pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr raw_cloud(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
            pcl::fromROSMsg(*cloud_msg, *raw_cloud);

            computeTransform(config_, rotation_transform_);
            pcl::transformPointCloud(*raw_cloud, *raw_cloud, rotation_transform_);

            pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr filtered_cloud(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
            *filtered_cloud = *raw_cloud;

            passThrough(config_, filtered_cloud);

            publishFilteredcloud(raw_cloud, filtered_cloud);
        }

        void publishFilteredcloud(const pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr cloud, const pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr filtered_cloud)
        {
            sensor_msgs::PointCloud2Ptr cloud_ros(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud, *cloud_ros);
            cloud_ros->header.frame_id = "world";
            cloud_ros->header.stamp = ros::Time::now();

            sensor_msgs::PointCloud2Ptr filtered_cloud_ros(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*filtered_cloud, *filtered_cloud_ros);
            filtered_cloud_ros->header.frame_id = "world";
            filtered_cloud_ros->header.stamp = ros::Time::now();

            pub_cloud_.publish(cloud_ros);
            pub_filtered_cloud_.publish(filtered_cloud_ros);
        }

        bool modifyRPYValue(passthrough_config::SendRPY::Request &req, passthrough_config::SendRPY::Response &res)
        {
            config_.rotation.pitch = req.pitch;
            config_.rotation.yaw = req.yaw;
            config_.rotation.roll = req.roll;

            return true;
        }

        bool modifyFieldxValue(passthrough_config::SendFieldX::Request &req, passthrough_config::SendFieldX::Response &res)
        {
            config_.field[0].min = req.x_min;
            config_.field[0].max = req.x_max;

            return true;
        }

        bool modifyFieldyValue(passthrough_config::SendFieldY::Request &req, passthrough_config::SendFieldY::Response &res)
        {
            config_.field[1].min = req.y_min;
            config_.field[1].max = req.y_max;

            return true;
        }

        bool modifyFieldzValue(passthrough_config::SendFieldZ::Request &req, passthrough_config::SendFieldZ::Response &res)
        {
            config_.field[2].min = req.z_min;
            config_.field[2].max = req.z_max;

            return true;
        }

        bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            ROS_INFO("Reseting ...");
            for (size_t i = 0; i < 3; ++i)
            {
                passthrough_filter::passthroughFieldConfig field_limits;
                field_limits.dim = "x";
                field_limits.min = -10.f;
                field_limits.max = 10.f;
                config_.field.push_back(field_limits);
            }
            config_.field[1].dim = "y";
            config_.field[2].dim = "z";
            config_.rotation.pitch = config_.rotation.roll = config_.rotation.yaw = 0.0f;
            // config_.field_x.min = config_.field_y.min = config_.field_z.min = -10.0f;
            // config_.field_x.max = config_.field_y.max = config_.field_z.max = 10.0f;
            rotation_transform_ = Eigen::Isometry3f::Identity();
            ROS_INFO("Reset done.");

            return true;
        }

        bool saveToYaml(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            ROS_INFO("Save parameters to Yaml.");
            std::string yaml_name;
            yaml_name = "/pt_param.yaml";

            save_config_path_ += yaml_name;

            std::ofstream out(save_config_path_);
            out << toYaml(config_);

            return true;
        }

        bool show(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            std::cout << toYaml(config_);
            return true;
        }

        bool load(passthrough_config::LoadConfig::Request &req, passthrough_config::LoadConfig::Response &res)
        {
            std::string path = req.path;
            config_ = YAML::LoadFile(path).as<passthrough_filter::passthroughConfig>();

            return true;
        }
    };

} // namespace passthrough_config
