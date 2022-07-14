#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "lidar_shape_estimation/shape_estimator.hpp"
#include "autoware_msgs/DetectedObjectArray.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define __APP_NAME__ "L-shape Fitting"

class ShapeEstimationNode
{
private: 
  tf::TransformListener *_transform_listener;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh;
  ros::Publisher pub_autoware_object_;
  ros::Publisher pub_jsk_bboxes_;
  ros::Subscriber sub_from_clustering;

  std::string bbox_source_frame_;
  std::string bbox_target_frame_;
  std::string L_shape_input_;
  std::string L_shape_output_;
  std::string L_shape_visualization_;

  float filter_res_;

  tf2_ros::TransformListener tf2_listener;
  tf2_ros::Buffer tf2_buffer;

  jsk_recognition_msgs::BoundingBox jsk_bbox_transform(const autoware_msgs::DetectedObject &autoware_bbox, 
          const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);

  void callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_msg);

private:
  ShapeEstimator estimator_;

public:
  ShapeEstimationNode();

  ~ShapeEstimationNode(){};
};


ShapeEstimationNode::ShapeEstimationNode() : nh_(""), private_nh("~"),tf2_listener(tf2_buffer)
{
  /* Initialize tuning parameter */
  private_nh.param<std::string>("bbox_target_frame", bbox_target_frame_, "base_link");
  ROS_INFO("[%s] bounding box's target frame is: %s", __APP_NAME__, bbox_target_frame_);
  private_nh.param<float>("filter_res", filter_res_, 0.0);
  ROS_INFO("[%s] filter_res is: %f", __APP_NAME__, filter_res_);

  private_nh.param<std::string>("L_shape_input_topic", L_shape_input_, "/segmentation/detected_objects");
  ROS_INFO("[%s] L shape input_topic is: %s", __APP_NAME__, L_shape_input_);
  private_nh.param<std::string>("L_shape_output_topic", L_shape_output_, "/perception/lidar/shape_estimation/objects");
  ROS_INFO("[%s] L shape output_topic is: %s", __APP_NAME__, L_shape_output_);

  private_nh.param<std::string>("L_shape_visualization_topic", L_shape_visualization_, "/perception/lidar/jsk_bbox_array");
  ROS_INFO("[%s] L-shape visualization topic is: %s", __APP_NAME__, L_shape_visualization_);

  sub_from_clustering = nh_.subscribe(L_shape_input_, 1, &ShapeEstimationNode::callback, this);
  pub_autoware_object_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(L_shape_output_, 1, true);
  pub_jsk_bboxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(L_shape_visualization_,1);
}

void ShapeEstimationNode::callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_msg)
{
  // Guard
  if (pub_autoware_object_.getNumSubscribers() < 1)
    return;

  // Create output msg
  auto output_msg = *input_msg;

  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_recognition_msgs::BoundingBoxArray jsk_bbox_array;
  geometry_msgs::TransformStamped transform_stamped;

  auto bbox_header = input_msg->header;
  bbox_source_frame_ = bbox_header.frame_id;
  bbox_header.frame_id = bbox_target_frame_;

  jsk_bbox_array.header = bbox_header;
  output_msg.header = bbox_header;

  try
  {
    transform_stamped = tf2_buffer.lookupTransform(bbox_target_frame_, bbox_source_frame_, ros::Time());
    // ROS_INFO("target_frame is %s",bbox_target_frame_.c_str());
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("Frame Transform Given Up! Outputing obstacles in the original LiDAR frame %s instead...", bbox_source_frame_.c_str());
    bbox_header.frame_id = bbox_source_frame_;
    try
    {
      transform_stamped = tf2_buffer.lookupTransform(bbox_source_frame_, bbox_source_frame_, ros::Time(0));
    }
    catch (tf2::TransformException& ex2)
    {
      ROS_ERROR("%s", ex2.what());
      return;
    }
  }

  const auto start_time = std::chrono::steady_clock::now();
  // Estimate shape for each object and pack msg
  for (auto &object : output_msg.objects)
  {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(object.pointcloud, *cluster);

    if (filter_res_ > 0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cluster);
      vg.setLeafSize(filter_res_, filter_res_, filter_res_);
      vg.filter(*cluster);
    }
    else{
      //ROS_INFO("no filting for the L-shape fitting");
    }

    bool success_fitting = estimator_.getShapeAndPose(object.label, *cluster, object);
    
    if(!success_fitting)
      continue;

    geometry_msgs::Pose pose, pose_transformed;
    pose.position = object.pose.position;
    pose.orientation = object.pose.orientation;

    tf2::doTransform(pose, pose_transformed, transform_stamped);

    object.header = bbox_header;
    object.pose = pose_transformed;

    jsk_bbox = jsk_bbox_transform(object, bbox_header, pose_transformed);
    jsk_bbox_array.boxes.emplace_back(jsk_bbox);
  }

  // Time the whole process
  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "BBOX L-shape fitting took " << elapsed_time.count() << " milliseconds" << std::endl;

  // Publish bounding box information
  pub_autoware_object_.publish(output_msg);
  pub_jsk_bboxes_.publish(jsk_bbox_array);

  return;
}

jsk_recognition_msgs::BoundingBox ShapeEstimationNode::jsk_bbox_transform(const autoware_msgs::DetectedObject &autoware_bbox, 
          const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_bbox.header = header;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions = autoware_bbox.dimensions;
  jsk_bbox.label = autoware_bbox.id;
  jsk_bbox.value = 1.0f;

  return std::move(jsk_bbox);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_shape_estimator");
  ShapeEstimationNode node;
  ros::spin();
  return 0;
}