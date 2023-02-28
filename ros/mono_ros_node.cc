#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <functional>
#include <memory>

#include <opencv2/opencv.hpp>

#include <System.h>
#include <Viewer.h>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Scalar.h"
#include "nav_msgs/msg/path.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "visualization_msgs/msg/image_marker.hpp"
#include "foxglove_msgs/msg/image_marker_array.hpp"

class OrbSlamMasterNode : public rclcpp::Node
{
public:
    OrbSlamMasterNode(ORB_SLAM3::System_ptr pSLAM, const std::string &settings)
        : Node("orbslam"), m_SLAM(pSLAM), m_Viewer(pSLAM, settings)
    {
        this->declare_parameter("camera_feed_topic", "/camera");
        this->declare_parameter("camera_pose_topic", "/camera_pose");
        this->declare_parameter("camera_path_topic", "/camera_path");
        this->declare_parameter("pointcloud_topic", "/pointcloud");
        this->declare_parameter("image_marker_topic", "/keypoints");
        this->declare_parameter("camera_feed_frame", "default_cam");
        this->declare_parameter("path_insert_distance_threshold", 0.1);
        this->declare_parameter("insert_path_time_threshold", 2);
        camera_feed_topic = this->get_parameter("camera_feed_topic").as_string();
        camera_pose_topic = this->get_parameter("camera_pose_topic").as_string();
        camera_path_topic = this->get_parameter("camera_path_topic").as_string();
        pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
        image_marker_topic = this->get_parameter("image_marker_topic").as_string();
        camera_feed_frame = this->get_parameter("camera_feed_frame").as_string();
        path_insert_distance_threshold = this->get_parameter("path_insert_distance_threshold").as_double();
        insert_path_time_threshold = this->get_parameter("insert_path_time_threshold").as_int();

        m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            camera_feed_topic,
            10,
            std::bind(&OrbSlamMasterNode::GrabImage, this, std::placeholders::_1));

        m_camera_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(camera_pose_topic, 10);
        m_path_publisher = this->create_publisher<nav_msgs::msg::Path>(camera_path_topic, 10);
        m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_image_marker_publisher = this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>(image_marker_topic, 10);
        map_world_position = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
    }

    /*
    // shutdown the node
    ~OrbSlamMasterNode()
    {
        m_Viewer.close();
        while (!m_Viewer.isFinished())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(), "(internal) Viewer closed");
    }*/

private:
    using ImageMsg = sensor_msgs::msg::Image;
    void GrabImage(const ImageMsg::SharedPtr msg)
    {
        // Copy the ros image message to cv::Mat.
        try
        {
            m_cvImPtr = cv_bridge::toCvCopy(msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        if (m_cvImPtr->image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty image");
            return;
        }
        else
        {
            camera_pose_or = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);
            m_Viewer.update(camera_pose_or);
            camera_pose = TransformFromMat(toMat(camera_pose_or));
            map_points = m_SLAM->GetAllMapPoints();
            if (!camera_pose_or.matrix().isIdentity())
            {
                this->PublishTf(msg->header.stamp);
                this->PublishCameraPose(msg->header.stamp);
                this->PublishCameraPath(msg->header.stamp);
                this->PublishImageMarker(msg->header.stamp);
                if (map_points.size() > 0)
                {
                    this->PublishPointCloud(map_points, msg->header.stamp);
                }
            }
        }
    }

    cv::Mat toMat(Sophus::SE3f pose)
    {
        Eigen::Matrix4f mat = pose.matrix();
        cv::Mat cv_mat(4, 4, CV_32F);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                cv_mat.at<float>(i, j) = mat(i, j);
            }
        }
        return cv_mat;
    }

    tf2::Transform TransformFromMat(cv::Mat position_mat)
    {
        cv::Mat rotation(3, 3, CV_32F);
        cv::Mat translation(3, 1, CV_32F);

        rotation = position_mat.rowRange(0, 3).colRange(0, 3);
        translation = position_mat.rowRange(0, 3).col(3);

        tf2::Matrix3x3 tf_camera_rotation(rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
                                          rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
                                          rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));

        tf2::Vector3 tf_camera_translation(translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));

        // Coordinate transformation matrix from orb coordinate system to ros coordinate system
        const tf2::Matrix3x3 tf_orb_to_ros(0, 0, 1,
                                           -1, 0, 0,
                                           0, -1, 0);

        // Transform from orb coordinate system to ros coordinate system on camera coordinates
        tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
        tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

        // Inverse matrix
        tf_camera_rotation = tf_camera_rotation.transpose();
        tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

        // Transform from orb coordinate system to ros coordinate system on map coordinates
        tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
        tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

        return tf2::Transform(tf_camera_rotation, tf_camera_translation);
    }

    void PublishImageMarker(rclcpp::Time stamp)
    {
        foxglove_msgs::msg::ImageMarkerArray image_markers = this->ConvertToImageMarker(m_SLAM->GetTrackedKeyPointsUn(), stamp);
        m_image_marker_publisher->publish(image_markers);
    }

    void PublishCameraPose(rclcpp::Time stamp)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose = this->TransformToPose(camera_pose);
        // Check if we have moved more than path_insert_distance_threshold since the last entry
        if (camera_pose_history.size() > 0)
        {
            auto last_pose = camera_pose_history.back().pose;
            tf2::Transform last_pose_tf = tf2::Transform(tf2::Quaternion(last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z, last_pose.orientation.w), tf2::Vector3(last_pose.position.x, last_pose.position.y, last_pose.position.z));
            float distance = (camera_pose.getOrigin() - last_pose_tf.getOrigin()).length();
            int last_pose_time = camera_pose_history.back().header.stamp.sec;
            if (distance > path_insert_distance_threshold or stamp.seconds() - last_pose_time > insert_path_time_threshold)
            {
                camera_pose_history.push_back(pose_msg);
                if (camera_pose_history.size() > 500)
                {
                    camera_pose_history.erase(camera_pose_history.begin());
                }
            }
        }
        else
        {
            camera_pose_history.push_back(pose_msg);
        }

        m_camera_pose_publisher->publish(pose_msg);
    }

    void PublishCameraPath(rclcpp::Time stamp)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";
        path_msg.poses = camera_pose_history;

        m_path_publisher->publish(path_msg);
    }

    void PublishPointCloud(std::vector<ORB_SLAM3::MapPoint *> mvpMapPoints, rclcpp::Time stamp)
    {
        sensor_msgs::msg::PointCloud2 pointcloud_msg = this->ConvertToPointCloud2(mvpMapPoints, stamp);
        m_pointcloud_publisher->publish(pointcloud_msg);
    }

    foxglove_msgs::msg::ImageMarkerArray ConvertToImageMarker(std::vector<cv::KeyPoint> keypoints, rclcpp::Time stamp)
    {
        if (keypoints.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "ConvertToImageMarker: No keypoints provided");
            return foxglove_msgs::msg::ImageMarkerArray();
        }
        foxglove_msgs::msg::ImageMarkerArray image_markers = foxglove_msgs::msg::ImageMarkerArray();
        // Convert vector of open cv keypoints to vector of image markers
        for (long unsigned int i = 0; i < keypoints.size(); i++)
        {
            visualization_msgs::msg::ImageMarker image_marker;
            image_marker.header.stamp = stamp;
            image_marker.header.frame_id = camera_feed_frame;
            image_marker.ns = "keypoints";
            image_marker.id = i;
            image_marker.type = visualization_msgs::msg::ImageMarker::CIRCLE;
            image_marker.action = visualization_msgs::msg::ImageMarker::ADD;
            image_marker.position.x = keypoints[i].pt.x;
            image_marker.position.y = keypoints[i].pt.y;
            image_marker.position.z = 0;
            image_marker.scale = 1;
            image_marker.outline_color.r = 0.0;
            image_marker.outline_color.g = 1.0;
            image_marker.outline_color.b = 0.0;
            image_marker.outline_color.a = 0.8;
            image_markers.markers.push_back(image_marker);
        }

        return image_markers;
    }

    geometry_msgs::msg::Pose TransformToPose(tf2::Transform transform)
    {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = transform.getOrigin().x();
        pose_msg.position.y = transform.getOrigin().y();
        pose_msg.position.z = transform.getOrigin().z();
        pose_msg.orientation.x = transform.getRotation().x();
        pose_msg.orientation.y = transform.getRotation().y();
        pose_msg.orientation.z = transform.getRotation().z();
        pose_msg.orientation.w = transform.getRotation().w();
        return pose_msg;
    }

    void PublishTf(rclcpp::Time stamp)
    {
        // Map -> Camera
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = camera_feed_frame;
        transform_msg.transform = tf2::toMsg(camera_pose);

        m_tf_broadcaster->sendTransform(transform_msg);

        // World -> Map
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = "map";
        transform_msg.transform = tf2::toMsg(map_world_position);
        m_tf_broadcaster->sendTransform(transform_msg);
    }

    sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(std::vector<ORB_SLAM3::MapPoint *> mvpMapPoints, rclcpp::Time stamp)
    {
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.stamp = stamp;
        pointcloud_msg.header.frame_id = "world";
        pointcloud_msg.height = 1;
        pointcloud_msg.width = mvpMapPoints.size();

        pointcloud_msg.fields.resize(3);
        pointcloud_msg.fields[0].name = "x";
        pointcloud_msg.fields[1].name = "y";
        pointcloud_msg.fields[2].name = "z";
        int offset = 0;
        for (size_t d = 0; d < pointcloud_msg.fields.size(); ++d, offset += 4)
        {
            pointcloud_msg.fields[d].offset = offset;
            pointcloud_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pointcloud_msg.fields[d].count = 1;
        }
        pointcloud_msg.point_step = offset;
        pointcloud_msg.row_step = pointcloud_msg.point_step * mvpMapPoints.size();
        pointcloud_msg.is_bigendian = false;
        pointcloud_msg.is_dense = true;
        pointcloud_msg.data.resize(pointcloud_msg.row_step);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
        {
            ORB_SLAM3::MapPoint *pMP = mvpMapPoints[i];
            if (!pMP)
            {
                continue;
            }

            if (pMP->isBad())
            {
                continue;
            }

            Eigen::Vector3f point_pos =
                pMP->GetWorldPos();
            float position[3] = {point_pos(2), -point_pos(0), -point_pos(1)};

            memcpy(&pointcloud_msg.data[i * pointcloud_msg.point_step + pointcloud_msg.fields[0].offset], &position[0], sizeof(float));
            memcpy(&pointcloud_msg.data[i * pointcloud_msg.point_step + pointcloud_msg.fields[1].offset], &position[1], sizeof(float));
            memcpy(&pointcloud_msg.data[i * pointcloud_msg.point_step + pointcloud_msg.fields[2].offset], &position[2], sizeof(float));
        }

        return pointcloud_msg;
    }

    std::string m_vocabulary_path;
    std::string m_settings_path;
    std::string camera_feed_topic;
    std::string camera_pose_topic;
    std::string pointcloud_topic;
    std::string camera_path_topic;
    std::string image_marker_topic;
    std::string camera_feed_frame;
    double path_insert_distance_threshold;
    int insert_path_time_threshold;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_camera_pose_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_publisher;
    rclcpp::Publisher<foxglove_msgs::msg::ImageMarkerArray>::SharedPtr m_image_marker_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    cv_bridge::CvImagePtr m_cvImPtr;
    ORB_SLAM3::System_ptr m_SLAM;
    ORB_SLAM3::Viewer m_Viewer;
    Sophus::SE3f camera_pose_or;
    tf2::Transform camera_pose;
    std::vector<ORB_SLAM3::MapPoint *> map_points;
    std::vector<geometry_msgs::msg::PoseStamped> camera_pose_history;

    tf2::Transform map_world_position;
};

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run moss_rock mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    ORB_SLAM3::System_ptr SLAM = std::make_shared<ORB_SLAM3::System>(argv[1], argv[2], ORB_SLAM3::CameraType::MONOCULAR);

    auto node = std::make_shared<OrbSlamMasterNode>(SLAM, argv[2]);

    rclcpp::spin(node);

    rclcpp::shutdown();
    // cout << "ROS exited, stoping mOS3" << endl;
    // SLAM->Shutdown();
    // cout << "mOS3 stopped, goodbye" << endl;
    return 0;
}
