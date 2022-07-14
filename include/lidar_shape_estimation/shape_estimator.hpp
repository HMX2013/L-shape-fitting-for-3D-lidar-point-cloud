#pragma once

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "autoware_msgs/DetectedObject.h"
#include <numeric>

class ShapeEstimator
{
private:

public:
  ShapeEstimator();

  ~ShapeEstimator(){};

  bool getShapeAndPose(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cluster,
                       autoware_msgs::DetectedObject& output);
  bool estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output);
  bool pca_fitting(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output);
  double best_angle_search(const pcl::PointCloud<pcl::PointXYZ>& cluster);
  double calcClosenessCriterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_var(const std::vector<double>& v);
  double calc_variances_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_nearest_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_area_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_autoware_default_criterion(const std::vector<double>& C_1, const std::vector<double>& C_2);
  double calc_autoware_complex_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);

  enum Criterion
  {
      AREA,
      NEAREST,
      VARIANCE,
      autoware_default,
      autoware_complex
  };

  Criterion criterion_;
};