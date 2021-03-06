#include "lidar_shape_estimation/shape_estimator.hpp"
#include <memory>
#include <iostream>

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>


ShapeEstimator::ShapeEstimator(){}

bool ShapeEstimator::getShapeAndPose(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cluster,
                                     autoware_msgs::DetectedObject& output)
{
  criterion_ = ShapeEstimator::autoware_complex;
  if (cluster.empty())
    return false;
  return estimate(cluster, output);
}


bool ShapeEstimator::estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output)
{
  constexpr double ep = 0.001;
  // calc centroid point for cylinder height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto& pcl_point : cluster)
  {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / (double)cluster.size();
  centroid.y = centroid.y / (double)cluster.size();
  centroid.z = centroid.z / (double)cluster.size();

  // // calc min and max z for cylinder length
  double min_z = 0;
  double max_z = 0;
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    if (cluster.at(i).z < min_z || i == 0)
      min_z = cluster.at(i).z;
    if (max_z < cluster.at(i).z || i == 0)
      max_z = cluster.at(i).z;
  }

  // calc circumscribed circle on x-y plane
  cv::Mat_<float> cv_points((int)cluster.size(), 2);
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    cv_points(i, 0) = cluster.at(i).x;  // x
    cv_points(i, 1) = cluster.at(i).y;  // y
  }

  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(cv::Mat(cv_points).reshape(2), center, radius);


  /* Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners */

  const double angle_reso = 1 * M_PI / 180.0;
  const double max_angle = M_PI / 2.0;
  double theta_star;  // col.10, Algo.2
  double max_q;
  
  std::vector<std::pair<double /*theta*/, double /*q*/>> Q;


  // rough search
  for (double theta = 0; theta < max_angle; theta += angle_reso)
  {
    Eigen::Vector2d e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2d e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<double> C_1;                   // col.5, Algo.2
    std::vector<double> C_2;                   // col.6, Algo.2
    for (const auto& point : cluster)
    {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }

    double q;

    if (criterion_ == Criterion::AREA)
    {
      q = calc_area_criterion(C_1, C_1);
    }
    if (criterion_ == Criterion::NEAREST)
    {
      q = calc_nearest_criterion(C_1, C_1);
    }
    else if (criterion_ == Criterion::VARIANCE)
    {
      q = calc_variances_criterion(C_1, C_1);
    }
    else if (criterion_ == Criterion::autoware_default)
    {
      q = calc_autoware_default_criterion(C_1, C_1);
    }
    else if (criterion_ == Criterion::autoware_complex)
    {
      q = calc_autoware_complex_criterion(C_1, C_1);
    }
    else
    {
      std::cout << "L-Shaped Algorithm Criterion Is Not Supported." << std::endl;
      break;
    }

    Q.push_back(std::make_pair(theta, q));        // col.8, Algo.2
  }

  for (size_t i = 0; i < Q.size(); ++i)
  {
    if (max_q < Q.at(i).second || i == 0)
    {
      max_q = Q.at(i).second;
      theta_star = Q.at(i).first;
    }
  }

  // Paper : Algo.2 Search-Based Rectangle Fitting

  Eigen::Vector2d e_1_star;  // col.11, Algo.2
  Eigen::Vector2d e_2_star;
  e_1_star << std::cos(theta_star), std::sin(theta_star);
  e_2_star << -std::sin(theta_star), std::cos(theta_star);
  std::vector<double> C_1_star;  // col.11, Algo.2
  std::vector<double> C_2_star;  // col.11, Algo.2
  for (const auto& point : cluster)
  {
    C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
    C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
  }

  // col.12, Algo.2
  const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const double a_1 = std::cos(theta_star);
  const double b_1 = std::sin(theta_star);
  const double c_1 = min_C_1_star;
  const double a_2 = -1.0 * std::sin(theta_star);
  const double b_2 = std::cos(theta_star);
  const double c_2 = min_C_2_star;
  const double a_3 = std::cos(theta_star);
  const double b_3 = std::sin(theta_star);
  const double c_3 = max_C_1_star;
  const double a_4 = -1.0 * std::sin(theta_star);
  const double b_4 = std::cos(theta_star);
  const double c_4 = max_C_2_star;

  // calc center of bounding box
  double intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  double intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimention of bounding box
  Eigen::Vector2d e_x;
  Eigen::Vector2d e_y;
  e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  Eigen::Vector2d diagonal_vec;
  diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

  // calc yaw
  tf2::Quaternion quat;
  quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));

  output.pose.orientation = tf2::toMsg(quat);
  // constexpr double ep = 0.001;
  output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
  output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
  output.dimensions.z = std::max((max_z - min_z), ep);
  output.pose_reliable = true;
  output.pose.position.x = (intersection_x_1 + intersection_x_2) / 2.0;
  output.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
  output.pose.position.z = centroid.z;

  // check wrong output
  if (output.dimensions.x < ep && output.dimensions.y < ep)
    return false;
  output.dimensions.x = std::max(output.dimensions.x, ep);
  output.dimensions.y = std::max(output.dimensions.y, ep);
  return true;
}


double ShapeEstimator::calc_area_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  const double c1_min = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c1_max = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c2_min = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double c2_max = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  double alpha = -(c1_max - c1_min) * (c2_max - c2_min);

  return alpha;
}

double ShapeEstimator::calc_nearest_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  const double c1_min = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c1_max = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c2_min = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double c2_max = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  std::vector<double> d1; // col.4, Algo.4
  for (const auto &c_1_element : C_1)
  {
      const double v = std::min(c1_max - c_1_element, c_1_element - c1_min);
      d1.push_back(std::fabs(v));
  }

  std::vector<double> d2; // col.5, Algo.4
  for (const auto &c_2_element : C_2)
  {
      const double v = std::min(c2_max - c_2_element, c_2_element - c2_min);
      d2.push_back(std::fabs(v));
  }

  double beta = 0;
  double min_dist_of_nearest_crit_ = 0.01;
  for (size_t i = 0; i < d1.size(); i++)
  {
      double d = std::max(std::min(d1[i], d2[i]), min_dist_of_nearest_crit_);
      beta += (1.0 / d);
  }

  return beta;
}

double ShapeEstimator::calc_variances_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  const double c1_min = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c1_max = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c2_min = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double c2_max = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  std::vector<double> d1; // col.4, Algo.4
  for (const auto &c_1_element : C_1)
  {
      const double v = std::min(c1_max - c_1_element, c_1_element - c1_min);
      d1.push_back(std::fabs(v));
  }

  std::vector<double> d2; // col.5, Algo.4
  for (const auto &c_2_element : C_2)
  {
      const double v = std::min(c2_max - c_2_element, c_2_element - c2_min);
      d2.push_back(std::fabs(v));
  }

  std::vector<double> e1;
  std::vector<double> e2;

  assert(d1.size() == d2.size());

  // d1.size() || d2.size() Is equals.
  for (size_t i = 0; i < d1.size(); i++)
  {
      if (d1[i] < d2[i])
      {
          e1.push_back(d1[i]);
      }
      else
      {
          e2.push_back(d2[i]);
      }
  }

  double v1 = 0.0;
  if (!e1.empty())
  {
      v1 = (-1.0) * calc_var(e1);
  }

  double v2 = 0.0;
  if (!e2.empty())
  {
      v2 = (-1.0) * calc_var(e2);
  }

  double gamma = v1 + v2;

  return gamma;
}

double ShapeEstimator::calc_autoware_default_criterion(const std::vector<double>& C_1, const std::vector<double>& C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const double min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const double max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
  const double min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
  const double max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

  std::vector<double> D_1;  // col.4, Algo.4
  for (const auto& c_1_element : C_1)
  {
    const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
    D_1.push_back(std::fabs(v));
  }

  std::vector<double> D_2;  // col.5, Algo.4
  for (const auto& c_2_element : C_2)
  {
    const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
    D_2.push_back(v * v);
  }

  const double d_min = 0.05;
  const double d_max = 0.50;
  double beta = 0;  // col.6, Algo.4
  for (size_t i = 0; i < D_1.size(); ++i)
  {
    const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
    beta += 1.0 / d;
  }
  return beta;
}

double ShapeEstimator::calc_autoware_complex_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
    // Paper : Algo.4 Closeness Criterion
    const double min_c_1 = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
    const double max_c_1 = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
    const double min_c_2 = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
    const double max_c_2 = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

    double max_beta = 0; // col.6, Algo.4
    {
        std::vector<double> D_1; // col.4, Algo.4
        for (const auto &c_1_element : C_1)
        {
            const double v = max_c_1 - c_1_element;
            D_1.push_back(v * v);
        }

        std::vector<double> D_2; // col.5, Algo.4
        for (const auto &c_2_element : C_2)
        {
            const double v = max_c_2 - c_2_element;
            D_2.push_back(v * v);
        }

        const double d_min = 0.1 * 0.1;
        const double d_max = 0.5 * 0.5;
        double beta = 0; // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        if (max_beta < beta)
            max_beta = beta;
    }
    {
        std::vector<double> D_1; // col.4, Algo.4
        for (const auto &c_1_element : C_1)
        {
            const double v = max_c_1 - c_1_element;
            D_1.push_back(v * v);
        }

        std::vector<double> D_2; // col.5, Algo.4
        for (const auto &c_2_element : C_2)
        {
            const double v = min_c_2 - c_2_element;
            D_2.push_back(v * v);
        }

        const double d_min = 0.1;
        const double d_max = 0.5;
        double beta = 0; // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        if (max_beta < beta)
            max_beta = beta;
    }
    {
        std::vector<double> D_1; // col.4, Algo.4
        for (const auto &c_1_element : C_1)
        {
            const double v = min_c_1 - c_1_element;
            D_1.push_back(v * v);
        }

        std::vector<double> D_2; // col.5, Algo.4
        for (const auto &c_2_element : C_2)
        {
            const double v = max_c_2 - c_2_element;
            D_2.push_back(v * v);
        }

        const double d_min = 0.1;
        const double d_max = 0.5;
        double beta = 0; // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        if (max_beta < beta)
            max_beta = beta;
    }
    {
        std::vector<double> D_1; // col.4, Algo.4
        for (const auto &c_1_element : C_1)
        {
            const double v = min_c_1 - c_1_element;
            D_1.push_back(v * v);
        }

        std::vector<double> D_2; // col.5, Algo.4
        for (const auto &c_2_element : C_2)
        {
            const double v = min_c_2 - c_2_element;
            D_2.push_back(v * v);
        }

        const double d_min = 0.01;
        const double d_max = 0.25;
        double beta = 0; // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        if (max_beta < beta)
            max_beta = beta;
    }

    return max_beta;
}

double ShapeEstimator::calc_var(const std::vector<double>& v)
{
    double sum  = std::accumulate(std::begin(v), std::end(v), 0.0);
    double mean = sum / v.size();

    double acc_var_num = 0.0;

    std::for_each(std::begin(v), std::end(v), [&](const double d) { acc_var_num += (d - mean) * (d - mean); });

    double var = sqrt(acc_var_num / (v.size() - 1));

    return var;
}



bool ShapeEstimator::pca_fitting(const pcl::PointCloud<pcl::PointXYZ> &cluster, autoware_msgs::DetectedObject &output)
{
  // Compute the bounding box height (to be used later for recreating the box)
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cluster, min_pt, max_pt);
  const float box_height = max_pt.z - min_pt.z;
  const float box_z = (max_pt.z + min_pt.z) / 2;
ROS_INFO("test1");
  // Compute the cluster centroid
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(cluster, pca_centroid);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_project(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cluster, *cluster_project);

  // Squash the cluster to x-y plane with z = centroid z
  for (size_t i = 0; i < cluster_project->size(); ++i)
  {
    cluster_project->points[i].z = pca_centroid(2);
  }
ROS_INFO("test2");
  // Compute principal directions & Transform the original cloud to PCA coordinates
  pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster_project);
  pca.project(*cluster_project, *pca_projected_cloud);

  const auto eigen_vectors = pca.getEigenVectors();
ROS_INFO("test3");
  // Get the minimum and maximum points of the transformed cloud.
  pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
  const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf quaternion(eigen_vectors); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);
ROS_INFO("test4");
  output.pose.position.x = position(0);
  output.pose.position.y = position(1);
  output.pose.position.z = position(2);

  output.pose.orientation.x = quaternion.x();
  output.pose.orientation.y = quaternion.y();
  output.pose.orientation.z = quaternion.z();
  output.pose.orientation.w = quaternion.w();

  output.dimensions.x = dimension(0);
  output.dimensions.y = dimension(1);
  output.dimensions.z = dimension(2);

  output.pose_reliable = true;

  return true;
}