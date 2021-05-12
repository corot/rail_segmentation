/*!
 * \file Segmenter.cpp
 * \brief The main segmentation node object.
 *
 * The segmenter is responsible for segmenting clusters from a point cloud topic. Visualization and data latched topics
 * are published after each request. A persistent array of objects is maintained internally.
 *
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author David Kent, GT - dekent@gatech.edu
 * \date January 12, 2016
 */

#include <omp.h>

// RAIL Segmentation
#include "rail_segmentation/Segmenter.h"

using namespace std;
using namespace rail::segmentation;

//constant definitions (to use in functions with reference parameters, e.g. param())
#if __cplusplus >= 201103L
constexpr bool Segmenter::DEFAULT_DEBUG;
constexpr int Segmenter::DEFAULT_MIN_SURFACE_SIZE;
constexpr int Segmenter::DEFAULT_MIN_CLUSTER_SIZE;
constexpr int Segmenter::DEFAULT_MAX_CLUSTER_SIZE;
constexpr double Segmenter::CLUSTER_TOLERANCE;
#else
const bool Segmenter::DEFAULT_DEBUG;
  const int Segmenter::DEFAULT_MIN_SURFACE_SIZE;
  const int Segmenter::DEFAULT_MIN_CLUSTER_SIZE;
  const int Segmenter::DEFAULT_MAX_CLUSTER_SIZE;
  const double Segmenter::CLUSTER_TOLERANCE;
#endif

Segmenter::Segmenter() : private_node_("~"), tf2_(tf_buffer_)
{
  // flag for using the provided point cloud

  // set defaults
  string point_cloud_topic("/head_camera/depth_registered/points");
  string zones_file(ros::package::getPath("rail_segmentation") + "/config/zones.yaml");

  // grab any parameters we need
  private_node_.param("debug", debug_, DEFAULT_DEBUG);
  private_node_.param("min_surface_size", min_surface_size_, DEFAULT_MIN_SURFACE_SIZE);
  private_node_.param("min_cluster_size", min_cluster_size_, DEFAULT_MIN_SURFACE_SIZE);
  private_node_.param("max_cluster_size", max_cluster_size_, DEFAULT_MAX_CLUSTER_SIZE);
  private_node_.param("cluster_tolerance", cluster_tolerance_, CLUSTER_TOLERANCE);
  private_node_.param("use_color", use_color_, false);
  private_node_.param("crop_first", crop_first_, false);
  private_node_.param("label_markers", label_markers_, false);
  private_node_.param<string>("point_cloud_topic", point_cloud_topic_, "/head_camera/depth_registered/points");
  private_node_.getParam("zones_config", zones_file);

  // setup publishers/subscribers we need
  segment_srv_ = private_node_.advertiseService("segment", &Segmenter::segmentCallback, this);
  segment_objects_srv_ = private_node_.advertiseService("segment_objects", &Segmenter::segmentObjectsCallback, this);
  segment_objects_from_point_cloud_srv_ = private_node_.advertiseService("segment_objects_from_point_cloud", &Segmenter::segmentObjectsFromPointCloudCallback, this);
  clear_srv_ = private_node_.advertiseService("clear", &Segmenter::clearCallback, this);
  remove_object_srv_ = private_node_.advertiseService("remove_object", &Segmenter::removeObjectCallback, this);
  calculate_features_srv_ = private_node_.advertiseService("calculate_features", &Segmenter::calculateFeaturesCallback,
                                                           this);
  segmented_objects_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObjectList>(
    "segmented_objects", 1
  );
  table_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObject>(
    "segmented_table", 1
  );
  markers_pub_ = private_node_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  table_pose_pub_ = private_node_.advertise<geometry_msgs::PoseStamped>("table_pose", 1, true);
  table_marker_pub_ = private_node_.advertise<visualization_msgs::Marker>("table_marker", 1, true);
  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pc1_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc1", 1, true);
    debug_pc2_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc2", 1, true);
    debug_pc3_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc3", 1, true);
    debug_img_pub_ = private_node_.advertise<sensor_msgs::Image>("debug_img", 1, true);
  }

  // check the YAML version
#ifdef YAMLCPP_GT_0_5_0
  // parse the segmentation zones
  YAML::Node zones_config = YAML::LoadFile(zones_file);
  for (size_t i = 0; i < zones_config.size(); i++)
  {
    YAML::Node cur = zones_config[i];
    // create a zone with the frame ID information
    SegmentationZone zone(cur["name"].as<string>(), cur["parent_frame_id"].as<string>(),
                          cur["child_frame_id"].as<string>(), cur["bounding_frame_id"].as<string>(),
                          cur["segmentation_frame_id"].as<string>());

    // check for the remove surface flag
    if (cur["remove_surface"].IsDefined())
    {
      zone.setRemoveSurface(cur["remove_surface"].as<bool>());
    }

    // check for the remove surface flag
    if (cur["require_surface"].IsDefined())
    {
      zone.setRequireSurface(cur["require_surface"].as<bool>());
    }

    // check for any set limits
    if (cur["roll_min"].IsDefined())
    {
      zone.setRollMin(cur["roll_min"].as<double>());
    }
    if (cur["roll_max"].IsDefined())
    {
      zone.setRollMax(cur["roll_max"].as<double>());
    }
    if (cur["pitch_min"].IsDefined())
    {
      zone.setPitchMin(cur["pitch_min"].as<double>());
    }
    if (cur["pitch_max"].IsDefined())
    {
      zone.setPitchMax(cur["pitch_max"].as<double>());
    }
    if (cur["yaw_min"].IsDefined())
    {
      zone.setYawMin(cur["yaw_min"].as<double>());
    }
    if (cur["yaw_max"].IsDefined())
    {
      zone.setYawMax(cur["yaw_max"].as<double>());
    }
    if (cur["x_min"].IsDefined())
    {
      zone.setXMin(cur["x_min"].as<double>());
    }
    if (cur["x_max"].IsDefined())
    {
      zone.setXMax(cur["x_max"].as<double>());
    }
    if (cur["y_min"].IsDefined())
    {
      zone.setYMin(cur["y_min"].as<double>());
    }
    if (cur["y_max"].IsDefined())
    {
      zone.setYMax(cur["y_max"].as<double>());
    }
    if (cur["z_min"].IsDefined())
    {
      zone.setZMin(cur["z_min"].as<double>());
    }
    if (cur["z_max"].IsDefined())
    {
      zone.setZMax(cur["z_max"].as<double>());
    }

    zones_.push_back(zone);
  }
#else
  // parse the segmentation zones
  ifstream fin(zones_file.c_str());
  YAML::Parser zones_parser(fin);
  YAML::Node zones_config;
  zones_parser.GetNextDocument(zones_config);
  for (size_t i = 0; i < zones_config.size(); i++)
  {
    // parse the required information
    string name, parent_frame_id, child_frame_id, bounding_frame_id, segmentation_frame_id;
    zones_config[i]["name"] >> name;
    zones_config[i]["parent_frame_id"] >> parent_frame_id;
    zones_config[i]["child_frame_id"] >> child_frame_id;
    zones_config[i]["bounding_frame_id"] >> bounding_frame_id;
    zones_config[i]["segmentation_frame_id"] >> segmentation_frame_id;

    // create a zone with the frame ID information
    SegmentationZone zone(name, parent_frame_id, child_frame_id, bounding_frame_id, segmentation_frame_id);

    // check for the remove surface flag
    if (zones_config[i].FindValue("remove_surface") != NULL)
    {
      bool remove_surface;
      zones_config[i]["remove_surface"] >> remove_surface;
      zone.setRemoveSurface(remove_surface);
    }
    if (zones_config[i].FindValue("require_surface") != NULL)
    {
      bool require_surface;
      zones_config[i]["require_surface"] >> require_surface;
      zone.setRequireSurface(require_surface);
    }

    // check for any set limits
    if (zones_config[i].FindValue("roll_min") != NULL)
    {
      double roll_min;
      zones_config[i]["roll_min"] >> roll_min;
      zone.setRollMin(roll_min);
    }
    if (zones_config[i].FindValue("roll_max") != NULL)
    {
      double roll_max;
      zones_config[i]["roll_max"] >> roll_max;
      zone.setRollMax(roll_max);
    }
    if (zones_config[i].FindValue("pitch_min") != NULL)
    {
      double pitch_min;
      zones_config[i]["pitch_min"] >> pitch_min;
      zone.setPitchMin(pitch_min);
    }
    if (zones_config[i].FindValue("pitch_max") != NULL)
    {
      double pitch_max;
      zones_config[i]["pitch_max"] >> pitch_max;
      zone.setPitchMax(pitch_max);
    }
    if (zones_config[i].FindValue("yaw_min") != NULL)
    {
      double yaw_min;
      zones_config[i]["yaw_min"] >> yaw_min;
      zone.setYawMin(yaw_min);
    }
    if (zones_config[i].FindValue("yaw_max") != NULL)
    {
      double yaw_max;
      zones_config[i]["yaw_max"] >> yaw_max;
      zone.setYawMax(yaw_max);
    }
    if (zones_config[i].FindValue("x_min") != NULL)
    {
      double x_min;
      zones_config[i]["x_min"] >> x_min;
      zone.setXMin(x_min);
    }
    if (zones_config[i].FindValue("x_max") != NULL)
    {
      double x_max;
      zones_config[i]["x_max"] >> x_max;
      zone.setXMax(x_max);
    }
    if (zones_config[i].FindValue("y_min") != NULL)
    {
      double y_min;
      zones_config[i]["y_min"] >> y_min;
      zone.setYMin(y_min);
    }
    if (zones_config[i].FindValue("y_max") != NULL)
    {
      double y_max;
      zones_config[i]["y_max"] >> y_max;
      zone.setYMax(y_max);
    }
    if (zones_config[i].FindValue("z_min") != NULL)
    {
      double z_min;
      zones_config[i]["z_min"] >> z_min;
      zone.setZMin(z_min);
    }
    if (zones_config[i].FindValue("z_max") != NULL)
    {
      double z_max;
      zones_config[i]["z_max"] >> z_max;
      zone.setZMax(z_max);
    }

    zones_.push_back(zone);
  }
#endif

  // check how many zones we have
  if (zones_.size() > 0)
  {
    ROS_INFO("%d segmenation zone(s) parsed.", (int) zones_.size());
    ROS_INFO("Segmenter Successfully Initialized");
    okay_ = true;
  } else
  {
    ROS_ERROR("No valid segmenation zones defined. Check %s.", zones_file.c_str());
    okay_ = false;
  }
}

bool Segmenter::okay() const
{
  return okay_;
}

const SegmentationZone &Segmenter::getCurrentZone() const
{
  // check each zone
  for (size_t i = 0; i < zones_.size(); i++)
  {
    // get the current TF information
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(zones_[i].getParentFrameID(),
                                                                    zones_[i].getChildFrameID(), ros::Time(0));

    // convert to a Matrix3x3 to get RPY
    tf2::Matrix3x3 mat(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                       tf.transform.rotation.w));
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // check if all the bounds meet
    if (roll >= zones_[i].getRollMin() && pitch >= zones_[i].getPitchMin() && yaw >= zones_[i].getYawMin() &&
      roll <= zones_[i].getRollMax() && pitch <= zones_[i].getPitchMax() && yaw <= zones_[i].getYawMax())
    {
      return zones_[i];
    }
  }

  ROS_WARN("Current state not in a valid segmentation zone. Defaulting to first zone.");
  return zones_[0];
}

bool Segmenter::removeObjectCallback(rail_segmentation::RemoveObject::Request &req,
                                     rail_segmentation::RemoveObject::Response &res)
{
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  // check the index
  if (req.index < object_list_.objects.size() && req.index < markers_.markers.size())
  {
    // remove
    object_list_.objects.erase(object_list_.objects.begin() + req.index);
    // set header information
    object_list_.header.seq++;
    object_list_.header.stamp = ros::Time::now();
    object_list_.cleared = false;
    // republish
    segmented_objects_pub_.publish(object_list_);
    // delete marker
    markers_.markers[req.index].action = visualization_msgs::Marker::DELETE;
    if (label_markers_)
    {
      text_markers_.markers[req.index].action = visualization_msgs::Marker::DELETE;
    }

    if (label_markers_)
    {
      visualization_msgs::MarkerArray marker_list;
      marker_list.markers.reserve(markers_.markers.size() + text_markers_.markers.size());
      marker_list.markers.insert(marker_list.markers.end(), markers_.markers.begin(), markers_.markers.end());
      marker_list.markers.insert(marker_list.markers.end(), text_markers_.markers.begin(), text_markers_.markers.end());
      markers_pub_.publish(marker_list);
    }
    else
    {
      markers_pub_.publish(markers_);
    }

    if (label_markers_)
    {
      text_markers_.markers.erase(text_markers_.markers.begin() + req.index);
    }
    markers_.markers.erase(markers_.markers.begin() + req.index);
    return true;
  } else
  {
    ROS_ERROR("Attempted to remove index %d from list of size %ld.", req.index, object_list_.objects.size());
    return false;
  }
}

bool Segmenter::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // lock for the messages
  boost::mutex::scoped_lock lock(msg_mutex_);
  // empty the list
  object_list_.objects.clear();
  object_list_.cleared = true;
  // set header information
  object_list_.header.seq++;
  object_list_.header.stamp = ros::Time::now();
  // republish
  segmented_objects_pub_.publish(object_list_);
  // delete markers
  for (size_t i = 0; i < markers_.markers.size(); i++)
  {
    markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  if (label_markers_)
  {
    for (size_t i = 0; i < text_markers_.markers.size(); i++)
    {
      text_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }

  if (label_markers_)
  {
    visualization_msgs::MarkerArray marker_list;
    marker_list.markers.reserve(markers_.markers.size() + text_markers_.markers.size());
    marker_list.markers.insert(marker_list.markers.end(), markers_.markers.begin(), markers_.markers.end());
    marker_list.markers.insert(marker_list.markers.end(), text_markers_.markers.begin(), text_markers_.markers.end());
//    markers_pub_.publish(marker_list);
  } else
  {
    //  markers_pub_.publish(markers_);
  }

  markers_.markers.clear();

  if (label_markers_)
  {
    text_markers_.markers.clear();
  }

  table_marker_.action = visualization_msgs::Marker::DELETE;
/// table_marker_pub_.publish(table_marker_);
  return true;
}

bool Segmenter::segmentCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  rail_manipulation_msgs::SegmentedObjectList objects;
  return segmentObjects(objects);
}

bool Segmenter::segmentObjectsCallback(rail_manipulation_msgs::SegmentObjects::Request &req,
                                       rail_manipulation_msgs::SegmentObjects::Response &res)
{
  return segmentObjects(res.segmented_objects, req.only_surface);
}

bool Segmenter::segmentObjectsFromPointCloudCallback(rail_manipulation_msgs::SegmentObjectsFromPointCloud::Request &req,
                                                     rail_manipulation_msgs::SegmentObjectsFromPointCloud::Response &res)
{
  // convert pc from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(req.point_cloud, *pc);

  return executeSegmentation(pc, res.segmented_objects);
}

bool Segmenter::segmentObjects(rail_manipulation_msgs::SegmentedObjectList &objects, bool only_surface)
{
  // get the latest point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Time request_time = ros::Time::now();
  ros::Time point_cloud_time = request_time - ros::Duration(0.1);
  while (point_cloud_time < request_time)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_msg =
      ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZRGB> >(point_cloud_topic_, node_,
                                                                     ros::Duration(10.0));
    if (pc_msg == NULL)
    {
      ROS_INFO("No point cloud received for segmentation.");
      return false;
    }
    else
    {
      *pc = *pc_msg;
    }
    point_cloud_time = pcl_conversions::fromPCL(pc->header.stamp);
  }

  ros::WallTime t0 = ros::WallTime::now();

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc, *pc, indices);
  bool kk= executeSegmentation(pc, objects, only_surface);
 // printf("%f\n", (ros::WallTime::now() - t0).toSec());
  return kk;
}

bool Segmenter::executeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
                                    rail_manipulation_msgs::SegmentedObjectList &objects, bool only_surface)
{
  // clear the objects first
  std_srvs::Empty empty;
  this->clearCallback(empty.request, empty.response);

  // determine the correct segmentation zone
  const SegmentationZone &zone = this->getCurrentZone();
  ROS_INFO("Segmenting %s in zone '%s'.", only_surface ? "only surfaces" : "objects", zone.getName().c_str());

  // transform the point cloud to the fixed frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZRGB>);


  //ROS_INFO("%d", pc->size());
  std::vector<int> mapped;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pc, *transformed_pc, mapped);

  //ROS_INFO("%d  %d", transformed_pc->size(), mapped.size());
  pcl_ros::transformPointCloud(zone.getBoundingFrameID(), ros::Time(0), *pc, pc->header.frame_id,
                               *transformed_pc, tf_);
  transformed_pc->header.frame_id = zone.getBoundingFrameID();
  transformed_pc->header.seq = pc->header.seq;
  transformed_pc->header.stamp = pc->header.stamp;

  // start with every index
  pcl::IndicesPtr filter_indices(new vector<int>);
  filter_indices->resize(transformed_pc->points.size());
  for (size_t i = 0; i < transformed_pc->points.size(); i++)
  {
    filter_indices->at(i) = i;
  }

  // check if we need to remove a surface
  double z_min = zone.getZMin();
  if (!crop_first_)
  {
    if (zone.getRemoveSurface())
    {
      bool surface_found = this->findSurface(transformed_pc, filter_indices, zone, filter_indices, only_surface, table_);
      if (zone.getRequireSurface() && !surface_found)
      {
        objects.objects.clear();
        ROS_INFO("Could not find a surface within the segmentation zone.  Exiting segmentation with no objects found.");
        return true;
      }
      double z_surface = table_.centroid.z;
      // check the new bound for Z
      z_min = max(zone.getZMin(), z_surface + SURFACE_REMOVAL_PADDING);
    }
  }

  // check bounding areas (bound the inverse of what we want since PCL will return the removed indicies)
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr bounds(new pcl::ConditionOr<pcl::PointXYZRGB>);
  if (z_min > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LE, z_min))
    );
  }
  if (zone.getZMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GE, zone.getZMax()))
    );
  }
  if (zone.getYMin() > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LE, zone.getYMin()))
    );
  }
  if (zone.getYMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GE, zone.getYMax()))
    );
  }
  if (zone.getXMin() > -numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LE, zone.getXMin()))
    );
  }
  if (zone.getXMax() < numeric_limits<double>::infinity())
  {
    bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GE, zone.getXMax()))
    );
  }

  //ROS_INFO("%d", filter_indices->size());
  // remove past the given bounds
  this->inverseBound(transformed_pc, filter_indices, bounds, filter_indices);

//  ROS_INFO("%d", filter_indices->size());
//  ROS_INFO("%d", transformed_pc->size());
//  ROS_INFO("%d", min_surface_size_);
  ////pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  this->extract(transformed_pc, filter_indices, transformed_pc);

  //ROS_INFO("%d", transformed_pc->size());
  if (debug_)
    debug_pc1_pub_.publish(transformed_pc);

  if (crop_first_)
  {
    if (zone.getRemoveSurface())
    {
      if (transformed_pc->size() < min_surface_size_)
      {
        objects.objects.clear();
        ROS_INFO("Cropped pointcloud too small to contain a valid surface (%lu < %d). Exiting segmentation.",
                 transformed_pc->size(), min_surface_size_);
        return true;
      }
      bool surface_found = this->findSurface(transformed_pc, filter_indices, zone, filter_indices, only_surface, table_);
      if (zone.getRequireSurface() && !surface_found)
      {
        objects.objects.clear();
        ROS_INFO("Could not find a surface within the segmentation zone.  Exiting segmentation with no objects found.");
        return true;
      }
      double z_surface = table_.centroid.z;
      // check the new bound for Z
      z_min = max(zone.getZMin(), z_surface + SURFACE_REMOVAL_PADDING);

      pcl::ConditionOr<pcl::PointXYZRGB>::Ptr table_bounds(new pcl::ConditionOr<pcl::PointXYZRGB>);
      table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LE, z_min))
      );

      // plane segmentation does adds back in the filtered indices, so we need to re-add the old bounds (this should
      // be faster than conditionally merging the two lists of indices, which would require a bunch of searches the
      // length of the point cloud's number of points)
      if (zone.getZMax() < numeric_limits<double>::infinity())
      {
        table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GE, zone.getZMax()))
        );
      }
      if (zone.getYMin() > -numeric_limits<double>::infinity())
      {
        table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LE, zone.getYMin()))
        );
      }
      if (zone.getYMax() < numeric_limits<double>::infinity())
      {
        table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GE, zone.getYMax()))
        );
      }
      if (zone.getXMin() > -numeric_limits<double>::infinity())
      {
        table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LE, zone.getXMin()))
        );
      }
      if (zone.getXMax() < numeric_limits<double>::infinity())
      {
        table_bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
          new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GE, zone.getXMax()))
        );
      }

      // remove below the table bounds
      this->inverseBound(transformed_pc, filter_indices, table_bounds, filter_indices);
    }
  }

  // publish the filtered and bounded PC pre-segmentation
  if (debug_)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    this->extract(transformed_pc, filter_indices, debug_pc);
  //  debug_pc2_pub_.publish(debug_pc);
  }


  // add to the markers
  table_marker_ = table_.marker;

  // publish the new list
  table_pub_.publish(table_);

  // publish the new marker array
  table_marker_pub_.publish(table_marker_);

  objects.objects.push_back(table_);

  if (only_surface)
    return true;  // client is interested only in segmenting a support surface, no objects

  //ros::WallTime t0 = ros::WallTime::now();

  // extract clusters
  vector<pcl::PointIndices> clusters;
  if (use_color_)
    this->extractClustersRGB(transformed_pc, filter_indices, clusters);
  else
    this->extractClustersEuclidean(transformed_pc, filter_indices, clusters);

  //printf("\n%f\n", (ros::WallTime::now() - t0).toSec());
  //t0 = ros::WallTime::now();

  if (clusters.size() > 0)
  {
    // lock for the messages
    boost::mutex::scoped_lock lock(msg_mutex_);
    // check each cluster
    //#pragma omp parallel for    // NOLINT
    for (size_t i = 0; i < clusters.size(); i++)
    {
      //ROS_ERROR("%d / %d", omp_get_thread_num(), omp_get_num_threads());
      // Keep track of image indices of points in the cluster
      vector<int> cluster_indices;
      // grab the points we need
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (size_t j = 0; j < clusters[i].indices.size(); j++)
      {
        cluster->points.push_back(transformed_pc->points[clusters[i].indices[j]]);
        cluster_indices.push_back(clusters[i].indices[j]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster->header.frame_id = transformed_pc->header.frame_id;

      // check if we need to transform to a different frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        // perform the copy/transform using TF
        pcl_ros::transformPointCloud(zone.getSegmentationFrameID(), ros::Time(0), *cluster, cluster->header.frame_id,
                                     *transformed_cluster, tf_);
        transformed_cluster->header.frame_id = zone.getSegmentationFrameID();
        transformed_cluster->header.seq = cluster->header.seq;
        transformed_cluster->header.stamp = cluster->header.stamp;
        pcl::toPCLPointCloud2(*transformed_cluster, *converted);
      } else
      {
        pcl::toPCLPointCloud2(*cluster, *converted);
      }

      // convert to a SegmentedObject message
      rail_manipulation_msgs::SegmentedObject segmented_object;
      segmented_object.recognized = false;
      // store the indices of the object
      segmented_object.image_indices = cluster_indices;

      // set the RGB image
      segmented_object.image = this->createImage(transformed_pc, clusters[i]);

      // check if we want to publish the image
      if (debug_)
      {
        debug_img_pub_.publish(segmented_object.image);
      }

      // set the point cloud
      pcl_conversions::fromPCL(*converted, segmented_object.point_cloud);
      segmented_object.point_cloud.header.stamp = ros::Time::now();
      // create a marker and set the extra fields
      segmented_object.marker = this->createMarker(converted);
      segmented_object.marker.id = i;

      // calculate color features
      Eigen::Vector3f rgb, lab;
      rgb[0] = segmented_object.marker.color.r;
      rgb[1] = segmented_object.marker.color.g;
      rgb[2] = segmented_object.marker.color.b;
      lab = RGB2Lab(rgb);
      segmented_object.rgb.resize(3);
      segmented_object.cielab.resize(3);
      segmented_object.rgb[0] = rgb[0];
      segmented_object.rgb[1] = rgb[1];
      segmented_object.rgb[2] = rgb[2];
      segmented_object.cielab[0] = lab[0];
      segmented_object.cielab[1] = lab[1];
      segmented_object.cielab[2] = lab[2];

      // set the centroid
      Eigen::Vector4f centroid;
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        pcl::compute3DCentroid(*transformed_cluster, centroid);
      } else
      {
        pcl::compute3DCentroid(*cluster, centroid);
      }
      segmented_object.centroid.x = centroid[0];
      segmented_object.centroid.y = centroid[1];
      segmented_object.centroid.z = centroid[2];

      // calculate the minimum volume bounding box (assuming the object is resting on a flat surface)
      segmented_object.bounding_volume = BoundingVolumeCalculator::computeBoundingVolume(segmented_object.point_cloud);

      // calculate the axis-aligned bounding box
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      segmented_object.width = max_pt[0] - min_pt[0];
      segmented_object.depth = max_pt[1] - min_pt[1];
      segmented_object.height = max_pt[2] - min_pt[2];

      // calculate the center
      segmented_object.center.x = (max_pt[0] + min_pt[0]) / 2.0;
      segmented_object.center.y = (max_pt[1] + min_pt[1]) / 2.0;
      segmented_object.center.z = (max_pt[2] + min_pt[2]) / 2.0;

      // calculate the orientation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      // project point cloud onto the xy plane
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      coefficients->values.resize(4);
      coefficients->values[0] = 0;
      coefficients->values[1] = 0;
      coefficients->values[2] = 1.0;
      coefficients->values[3] = 0;
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        proj.setInputCloud(transformed_cluster);
      } else
      {
        proj.setInputCloud(cluster);
      }
      proj.setModelCoefficients(coefficients);
      proj.filter(*projected_cluster);

      //calculate the Eigen vectors of the projected point cloud's covariance matrix, used to determine orientation
      Eigen::Vector4f projected_centroid;
      Eigen::Matrix3f covariance_matrix;
      pcl::compute3DCentroid(*projected_cluster, projected_centroid);
      pcl::computeCovarianceMatrixNormalized(*projected_cluster, projected_centroid, covariance_matrix);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
      eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
      //calculate rotation from eigenvectors
      const Eigen::Quaternionf qfinal(eigen_vectors);

      //convert orientation to a single angle on the 2D plane defined by the segmentation coordinate frame
      tf::Quaternion tf_quat;
      tf_quat.setValue(qfinal.x(), qfinal.y(), qfinal.z(), qfinal.w());
      double r, p, y;
      tf::Matrix3x3 m(tf_quat);
      m.getRPY(r, p, y);
      double angle = r + y;
      while (angle < -M_PI)
      {
        angle += 2 * M_PI;
      }
      while (angle > M_PI)
      {
        angle -= 2 * M_PI;
      }
      segmented_object.orientation = tf::createQuaternionMsgFromYaw(angle);

      //#pragma omp critical
      // add to the final list
      objects.objects.push_back(segmented_object);
      // add to the markers
      markers_.markers.push_back(segmented_object.marker);

      if (label_markers_)
      {
        // create a text marker to label the current marker
        visualization_msgs::Marker text_marker;
        text_marker.header = segmented_object.marker.header;
        text_marker.ns = "segmentation_labels";
        text_marker.id = i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;

        text_marker.pose.position.x = segmented_object.center.x;
        text_marker.pose.position.y = segmented_object.center.y;
        text_marker.pose.position.z = segmented_object.center.z + 0.05 + segmented_object.height/2.0;

        text_marker.scale.z = .1;

        text_marker.color.r = 1;
        text_marker.color.g = 1;
        text_marker.color.b = 1;
        text_marker.color.a = 1;

        stringstream marker_label;
        marker_label << "i:" << i;
        text_marker.text = marker_label.str();

        text_markers_.markers.push_back(text_marker);
      }
      //ROS_ERROR("%d done", omp_get_thread_num());
    }

    // create the new list
    objects.header.seq++;
    objects.header.stamp = ros::Time::now();
    objects.header.frame_id = zone.getSegmentationFrameID();
    objects.cleared = false;

    // update the new list and publish it
    object_list_ = objects;
    segmented_objects_pub_.publish(object_list_);

    // publish the new marker array
    if (label_markers_)
    {
      visualization_msgs::MarkerArray marker_list;
      marker_list.markers.reserve(markers_.markers.size() + text_markers_.markers.size());
      marker_list.markers.insert(marker_list.markers.end(), markers_.markers.begin(), markers_.markers.end());
      marker_list.markers.insert(marker_list.markers.end(), text_markers_.markers.begin(), text_markers_.markers.end());
      markers_pub_.publish(marker_list);
    } else
    {
      markers_pub_.publish(markers_);
    }
  } else
  {
    ROS_WARN("No segmented objects found.");
  }
  //printf("\n%f\n", (ros::WallTime::now() - t0).toSec());

  // publish the table, even if found no segmented objects

  // add to the markers
  table_marker_ = table_.marker;

  // publish the new list
  table_pub_.publish(table_);

  // publish the new marker array
  table_marker_pub_.publish(table_marker_);

  return true;
}

bool Segmenter::calculateFeaturesCallback(rail_manipulation_msgs::ProcessSegmentedObjects::Request &req,
                                          rail_manipulation_msgs::ProcessSegmentedObjects::Response &res)
{
  res.segmented_objects.header = req.segmented_objects.header;
  res.segmented_objects.cleared = req.segmented_objects.cleared;
  res.segmented_objects.objects.resize(req.segmented_objects.objects.size());

  for (size_t i = 0; i < res.segmented_objects.objects.size(); i ++)
  {
    // convert to a SegmentedObject message
    res.segmented_objects.objects[i].recognized = req.segmented_objects.objects[i].recognized;

    // can't recalculate this after initial segmentation has already happened...
    res.segmented_objects.objects[i].image = req.segmented_objects.objects[i].image;

    // can't recalculate this without the initial image...
    res.segmented_objects.objects[i].image_indices = req.segmented_objects.objects[i].image_indices;

    // set the point cloud
    res.segmented_objects.objects[i].point_cloud = req.segmented_objects.objects[i].point_cloud;

    // get point cloud as pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(res.segmented_objects.objects[i].point_cloud, *temp_cloud);
    pcl::fromPCLPointCloud2(*temp_cloud, *pcl_cloud);

    // create a marker and set the extra fields
    res.segmented_objects.objects[i].marker = this->createMarker(temp_cloud);
    res.segmented_objects.objects[i].marker.id = i;

    // calculate color features
    Eigen::Vector3f rgb, lab;
    rgb[0] = res.segmented_objects.objects[i].marker.color.r;
    rgb[1] = res.segmented_objects.objects[i].marker.color.g;
    rgb[2] = res.segmented_objects.objects[i].marker.color.b;
    lab = RGB2Lab(rgb);
    res.segmented_objects.objects[i].rgb.resize(3);
    res.segmented_objects.objects[i].cielab.resize(3);
    res.segmented_objects.objects[i].rgb[0] = rgb[0];
    res.segmented_objects.objects[i].rgb[1] = rgb[1];
    res.segmented_objects.objects[i].rgb[2] = rgb[2];
    res.segmented_objects.objects[i].cielab[0] = lab[0];
    res.segmented_objects.objects[i].cielab[1] = lab[1];
    res.segmented_objects.objects[i].cielab[2] = lab[2];

    // set the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pcl_cloud, centroid);
    res.segmented_objects.objects[i].centroid.x = centroid[0];
    res.segmented_objects.objects[i].centroid.y = centroid[1];
    res.segmented_objects.objects[i].centroid.z = centroid[2];

    // calculate the minimum volume bounding box (assuming the object is resting on a flat surface)
    res.segmented_objects.objects[i].bounding_volume =
      BoundingVolumeCalculator::computeBoundingVolume(res.segmented_objects.objects[i].point_cloud);

    // calculate the axis-aligned bounding box
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*pcl_cloud, min_pt, max_pt);
    res.segmented_objects.objects[i].width = max_pt[0] - min_pt[0];
    res.segmented_objects.objects[i].depth = max_pt[1] - min_pt[1];
    res.segmented_objects.objects[i].height = max_pt[2] - min_pt[2];

    // calculate the center
    res.segmented_objects.objects[i].center.x = (max_pt[0] + min_pt[0]) / 2.0;
    res.segmented_objects.objects[i].center.y = (max_pt[1] + min_pt[1]) / 2.0;
    res.segmented_objects.objects[i].center.z = (max_pt[2] + min_pt[2]) / 2.0;

    // calculate the orientation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    // project point cloud onto the xy plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pcl_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected_cluster);

    //calculate the Eigen vectors of the projected point cloud's covariance matrix, used to determine orientation
    Eigen::Vector4f projected_centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::compute3DCentroid(*projected_cluster, projected_centroid);
    pcl::computeCovarianceMatrixNormalized(*projected_cluster, projected_centroid, covariance_matrix);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
    eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
    //calculate rotation from eigenvectors
    const Eigen::Quaternionf qfinal(eigen_vectors);

    //convert orientation to a single angle on the 2D plane defined by the segmentation coordinate frame
    tf::Quaternion tf_quat;
    tf_quat.setValue(qfinal.x(), qfinal.y(), qfinal.z(), qfinal.w());
    double r, p, y;
    tf::Matrix3x3 m(tf_quat);
    m.getRPY(r, p, y);
    double angle = r + y;
    while (angle < -M_PI)
    {
      angle += 2 * M_PI;
    }
    while (angle > M_PI)
    {
      angle -= 2 * M_PI;
    }
    res.segmented_objects.objects[i].orientation = tf::createQuaternionMsgFromYaw(angle);
  }

  return true;
}

/*
void getPlaneTransform(const cv::Vec4f& plane_coefficients, cv::Matx33f& rotation, cv::Vec3f& translation)
{
  double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
  // assume plane coefficients are normalized
  translation = cv::Vec3f(-a * d, -b * d, -c * d);
  cv::Vec3f z(a, b, c);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
};
*/
bool Segmenter::findSurface(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
                            const pcl::IndicesConstPtr &indices_in, const SegmentationZone &zone, const pcl::IndicesPtr &indices_out,
                            bool check_contiguous, rail_manipulation_msgs::SegmentedObject &table_out) const
{
ros::WallTime t0 = ros::WallTime::now();

  // use a plane (SAC) segmenter
//  pcl::SACSegmentation<pcl::PointXYZRGB> plane_seg;
//  // set the segmenation parameters
//  plane_seg.setOptimizeCoefficients(true);
//  plane_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);   // try SACMODEL_NORMAL_PLANE if ko   need pcl::SACSegmentationFromNormals
//  plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
//  plane_seg.setEpsAngle(SAC_EPS_ANGLE  /2.0);  // 4.3 deg
//  plane_seg.setMethodType(pcl::SAC_RANSAC);
//  plane_seg.setMaxIterations(SAC_MAX_ITERATIONS   *100);
//  plane_seg.setDistanceThreshold(SAC_DISTANCE_THRESHOLD);  // 1 cm

//  ROS_WARN("norm1");
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(50);
//  ROS_WARN("norm2");

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> plane_seg;
  plane_seg.setNormalDistanceWeight(0.1);
  plane_seg.setOptimizeCoefficients(false);  // not really needed, and avoids a horrible log error
  plane_seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  plane_seg.setAxis(Eigen::Vector3f(0, 0, 1));
  plane_seg.setEpsAngle(SAC_EPS_ANGLE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setProbability(0.99);
  plane_seg.setDistanceThreshold(SAC_DISTANCE_THRESHOLD);
  plane_seg.setMaxIterations(SAC_MAX_ITERATIONS);

  // create a copy to work with
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZRGB>(*in));
  pcl::PointCloud<pcl::Normal>::Ptr pc_normals(new pcl::PointCloud<pcl::Normal>);

  norm_est.setInputCloud(pc_copy);
  //norm_est.setIndices(indices_in);
  norm_est.compute(*pc_normals);

//  ROS_WARN("norm3");
  plane_seg.setInputCloud(pc_copy);
  plane_seg.setInputNormals(pc_normals);
  //plane_seg.setIndices(indices_in);

  // Check point height -- if the plane is too low or high, extract another
  while (true)
  {
    // points included in the plane (surface)
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
    // segment the the current cloud
    pcl::ModelCoefficients coefficients;
    plane_seg.segment(*inliers_ptr, coefficients);

    // check if we found a surface
    if (inliers_ptr->indices.size() < min_surface_size_)
    {
      ROS_WARN("Could not find a surface above %fm and below %fm with more than %d point (biggest has %lu).",
               zone.getZMin(), zone.getZMax(), min_surface_size_, inliers_ptr->indices.size());
      //// *indices_out = *indices_in;
      table_out.centroid.z = -numeric_limits<double>::infinity();
      return false;
    }
    ROS_WARN_STREAM("plane found   "<< inliers_ptr->indices.size() <<   "     " <<pc_copy->size());

    // remove the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract(true);
    extract.setInputCloud(pc_copy);
    extract.setIndices(inliers_ptr);
    extract.setNegative(false);
    extract.filter(*plane);

    extract.setKeepOrganized(true);  // otherwise replaces the removed points with NaN   Can't use 2D indexing with a unorganized point clou
   // ROS_WARN("setIndices");

    //////this->extract(pc_copy, std::static_pointer_cast<const pcl::IndicesConstPtr>(inliers_ptr), pc_copy);
    plane_seg.setIndices(extract.getRemovedIndices());

  //  ROS_WARN_STREAM("avg z  " << this->averageZ(plane.points) << "     " <<pc_copy->size() << "     " <<extract.getRemovedIndices()->size());

    // Create the filtering object
//    extract.setNegative(true);
//    extract.filter(*pc_copy);
    /////cloud_filtered.swap (cloud_f);
    ///   plane_seg.setInputCloud(pc_copy);  //TRY TO REMOVE  TODO
//    plane_seg.setIndices(extract.getRemovedIndices());

  //  ROS_WARN_STREAM("NORM     " <<pc_copy->size());

    // check the height
    double height = this->averageZ(plane->points);
    if (height >= zone.getZMin() && height <= zone.getZMax())
    {
      ROS_INFO("Surface found at %fm.", height);

      if (check_contiguous)
      {
        std::vector<pcl::PointIndices> clusters;
        tree->setInputCloud(plane);
        pcl::extractEuclideanClusters<pcl::PointXYZRGB>(
            *plane, tree, 0.01f, clusters, min_surface_size_ / 2.0);
        if (clusters.size() > 1) {
          ROS_WARN("Discarding not-contiguous surface (%lu clusters)",
                   clusters.size());
          return false;
        }
      }

      *indices_out = *plane_seg.getIndices();

      // check if we need to transform to a different frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        // perform the copy/transform using TF
        pcl_ros::transformPointCloud(zone.getSegmentationFrameID(), ros::Time(0), *plane, plane->header.frame_id,
                                     *transformed_pc, tf_);
        transformed_pc->header.frame_id = zone.getSegmentationFrameID();
        transformed_pc->header.seq = plane->header.seq;
        transformed_pc->header.stamp = plane->header.stamp;
        pcl::toPCLPointCloud2(*transformed_pc, *converted);
      } else
      {
        pcl::toPCLPointCloud2(*plane, *converted);
      }

      //  pcl::fromROSMsg(table_.point_cloud, *debug_pc);
      // this->extract(transformed_pc, filter_indices, debug_pc);
      if (debug_)
        debug_pc3_pub_.publish(plane);

      // convert to a SegmentedObject message
      table_out.recognized = false;

      // set the RGB image
      table_out.image = this->createImage(pc_copy, *inliers_ptr);

      // check if we want to publish the image
      if (debug_)
      {
        debug_img_pub_.publish(table_out.image);
      }

      // set the point cloud
      pcl_conversions::fromPCL(*converted, table_out.point_cloud);
      table_out.point_cloud.header.stamp = ros::Time::now();
//      ROS_WARN_STREAM(""<< table_out.point_cloud.data.size());
      // create a marker and set the extra fields
      table_out.marker = this->createMarker(converted);
      table_out.marker.id = 0;

      // set the centroid
      Eigen::Vector4f centroid;
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        pcl::compute3DCentroid(*transformed_pc, centroid);
      } else
      {
        pcl::compute3DCentroid(*plane, centroid);
      }
      table_out.centroid.x = centroid[0];
      table_out.centroid.y = centroid[1];
      table_out.centroid.z = centroid[2];

      // calculate the bounding box
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*plane, min_pt, max_pt);
      table_out.width = max_pt[0] - min_pt[0];
      table_out.depth = max_pt[1] - min_pt[1];
      table_out.height = max_pt[2] - min_pt[2];

      // calculate the center
      table_out.center.x = (max_pt[0] + min_pt[0]) / 2.0;
      table_out.center.y = (max_pt[1] + min_pt[1]) / 2.0;
      table_out.center.z = (max_pt[2] + min_pt[2]) / 2.0;

      // calculate color features
      Eigen::Vector3f rgb, lab;
      rgb[0] = table_out.marker.color.r;
      rgb[1] = table_out.marker.color.g;
      rgb[2] = table_out.marker.color.b;
      lab = RGB2Lab(rgb);
      table_out.rgb.resize(3);
      table_out.cielab.resize(3);
      table_out.rgb[0] = rgb[0];
      table_out.rgb[1] = rgb[1];
      table_out.rgb[2] = rgb[2];
      table_out.cielab[0] = lab[0];
      table_out.cielab[1] = lab[1];
      table_out.cielab[2] = lab[2];

      // calculate the orientation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      // project point cloud onto the xy plane
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      coefficients->values.resize(4);
      coefficients->values[0] = 0;
      coefficients->values[1] = 0;
      coefficients->values[2] = 1.0;
      coefficients->values[3] = 0;
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      if (zone.getBoundingFrameID() != zone.getSegmentationFrameID())
      {
        proj.setInputCloud(transformed_pc);
      }
      else
      {
        proj.setInputCloud(plane);
      }
      proj.setModelCoefficients(coefficients);
      proj.filter(*projected_cluster);

//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
//      pcl::fromROSMsg(table_.point_cloud, *debug_pc);
//      debug_pc_pub_.publish(debug_pc);
/*
      //calculate the Eigen vectors of the projected point cloud's covariance matrix, used to determine orientation
      Eigen::Vector4f projected_centroid;
      Eigen::Matrix3f covariance_matrix;
      pcl::compute3DCentroid(*projected_cluster, projected_centroid);
      pcl::computeCovarianceMatrixNormalized(*projected_cluster, projected_centroid, covariance_matrix);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
      eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));


      // Transform the original cloud to the origin where the principal components correspond to the axes.
      Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
      projectionTransform.block<3,3>(0,0) = eigen_vectors.transpose();
      projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * projected_centroid.head<3>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*projected_cluster, *cloudPointsProjected, projectionTransform);
      // Get the minimum and maximum points of the transformed cloud.
      pcl::PointXYZRGB minPoint, maxPoint;
      pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
      const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

      // Final transform from eigenvectors
      const Eigen::Quaternionf bbox_q(eigen_vectors);
      const Eigen::Vector3f bbox_tf = eigen_vectors * meanDiagonal + projected_centroid.head<3>();
*/
      Eigen::Vector4f farthest_pt;
      Eigen::Vector4f center_pt(table_out.center.x, table_out.center.y, table_out.center.z, 0.0);
      pcl::getMaxDistance(*projected_cluster, center_pt, farthest_pt);

//      1. tengo las dimensiones, pero son inutiles sin la orientacion
//      2. tengo q entender los eigen-leches para obtenerla  (o usar otra cosa)
//      3. por curiosidad, el codigo actual funcionaria con una mesa alargada  -->  comprobarlo
//      double x1 maxPoint.getVector3fMap() + minPoint.getVector3fMap()
      double alpha = std::atan(table_out.width / table_out.depth);
      double beta = std::atan2(farthest_pt.y() - center_pt.y(), farthest_pt.x() - center_pt.x());
      double theta = std::atan2(farthest_pt.y() - center_pt.y(), farthest_pt.x() - center_pt.x()) - alpha;

      Eigen::Affine3f transform = //Eigen::Affine3f::Identity();
      //transform.translation() = center_pt.head<3>();
      pcl::getTransformation(center_pt.x(), center_pt.y(), center_pt.z(), 0.0, 0.0, theta);
      //transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*projected_cluster, *cloudPointsProjected, transform.inverse());
      // Get the minimum and maximum points of the transformed cloud.
      pcl::PointXYZRGB minPoint, maxPoint;
      pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
      table_out.depth = maxPoint.x - minPoint.x;
      table_out.width = maxPoint.y - minPoint.y;
  //   TODO    projected_cluster->header.frame_id = "table";
      if (debug_)
      {
        debug_pc2_pub_.publish(projected_cluster);
        debug_pc3_pub_.publish(cloudPointsProjected);
      }
      //Eigen::Vector2f corner_pt(farthest_pt.x() - pivot_pt.x(), farthest_pt.y() - pivot_pt.y());
/*   de momento solo mesas cuadradas,,,  no hace falta todo esto    mmm.... si, pero entonces los marcos tienen mala orientacion!!!
      double x = farthest_pt.x() - pivot_pt.x();
      double y = farthest_pt.y() - pivot_pt.y();
      Eigen::Vector2f corner_pt((std::cos(theta) * farthest_pt.x() - std::sin(theta) * farthest_pt.y()) - pivot_pt.x(),
                                (std::sin(theta) * farthest_pt.x() - std::cos(theta) * farthest_pt.y()) - pivot_pt.y());
//      corner_pt = Eigen::Translation2d(pivot_pt.x(), pivot_pt.y()) * corner_pt;   OSTIAS   esto:  .head<2>()
//      corner_pt = Eigen::Rotation2D<double>(theta) * corner_pt;
//        //pcl::transformPoint(
//
      ROS_ERROR("%f  %f    %f  %f  %f        %f  %f ", table_out.depth, table_out.width, alpha, beta, theta,
                corner_pt.x(), corner_pt.y());
      if (std::abs(std::abs(corner_pt.y()) - table_out.width/2.0) > 0.01)
      {
        theta = std::atan2(farthest_pt.y() - pivot_pt.y(), farthest_pt.x() - pivot_pt.x()) + alpha;
        corner_pt = Eigen::Vector2f(std::cos(theta) * x - std::sin(theta) * y, std::sin(theta) * x - std::cos(theta) * y);
//        corner_pt = Eigen::Translation2d(pivot_pt) * farthest_pt;
//        corner_pt = Eigen::Rotation2D<double>(theta) * corner_pt;
        ROS_ERROR("%f  %f    %f  %f  %f        %f  %f ", table_out.depth, table_out.width, alpha, beta, theta,
                  corner_pt.x(), corner_pt.y());
      }
    */  table_out.orientation = tf::createQuaternionMsgFromYaw(theta);
      geometry_msgs::PoseStamped table_pose;
      table_pose.header.frame_id = zone.getSegmentationFrameID();
      table_pose.pose.position = table_.center;
//      table_pose.pose.position.x = bbox_tf.x();
//      table_pose.pose.position.y = bbox_tf.y();
//      table_pose.pose.position.z = bbox_tf.z();
//      table_pose.pose.orientation.x = bbox_q.x();
//      table_pose.pose.orientation.y = bbox_q.y();
//      table_pose.pose.orientation.z = bbox_q.z();
//      table_pose.pose.orientation.w = bbox_q.w();
      table_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        table_pose_pub_.publish(table_pose);
      table_marker_pub_.publish(createMarker(table_pose, center_pt, farthest_pt));
//      table_marker_pub_.publish(createMarker(zone.getSegmentationFrameID(), bbox_tf, bbox_q, minPoint, maxPoint));
      table_marker_pub_.publish(createMarker(table_pose, minPoint, maxPoint));
      // This viewer has 4 windows, but is only showing images in one of them as written here.
//      pcl::visualization::PCLVisualizer *visu;
//      visu = new pcl::visualization::PCLVisualizer("PlyViewer");
//      int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
//      visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
//      visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
//      visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
//      visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
//      visu->addPointCloud(projected_cluster, "bboxedCloud", mesh_vp_3);
//      visu->addCube(bbox_tf, bbox_q, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
      ///visu->addCube(projected_centroid, bbox_q, 0.1, 0.1, 0.1, "centroid", mesh_vp_3);

      //convert orientation to a single angle on the 2D plane defined by the segmentation coordinate frame
//      tf::Quaternion tf_quat;
//      tf_quat.setValue(bbox_q.x(), bbox_q.y(), bbox_q.z(), bbox_q.w());
//      double r, p, y;
//      tf::Matrix3x3 m(tf_quat);
//      m.getRPY(r, p, y);
//      double angle = r + y;
//      while (angle < -M_PI)
//      {
//        angle += 2 * M_PI;
//      }
//      while (angle > M_PI)
//      {
//        angle -= 2 * M_PI;
//      }
//      table_out.orientation = tf::createQuaternionMsgFromYaw(angle);

//      printf("%f\n", (ros::WallTime::now() - t0).toSec());
      return true;
    }
  }
}

void Segmenter::extractClustersEuclidean(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
                                         const pcl::IndicesConstPtr &indices_in, vector<pcl::PointIndices> &clusters) const
{
  // ignore NaN and infinite values
  pcl::IndicesPtr valid(new vector<int>);
  for (size_t i = 0; i < indices_in->size(); i++)
  {
    if (pcl_isfinite(in->points[indices_in->at(i)].x) & pcl_isfinite(in->points[indices_in->at(i)].y) &
      pcl_isfinite(in->points[indices_in->at(i)].z))
    {
      valid->push_back(indices_in->at(i));
    }
  }

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> seg;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kd_tree->setInputCloud(in);
  seg.setClusterTolerance(cluster_tolerance_);
  seg.setMinClusterSize(min_cluster_size_);
  seg.setMaxClusterSize(max_cluster_size_);
  seg.setSearchMethod(kd_tree);
  seg.setInputCloud(in);
  seg.setIndices(valid);
  seg.extract(clusters);
}

void Segmenter::extractClustersRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
                                   const pcl::IndicesConstPtr &indices_in, vector<pcl::PointIndices> &clusters) const
{
  // ignore NaN and infinite values
  pcl::IndicesPtr valid(new vector<int>);
  for (size_t i = 0; i < indices_in->size(); i++)
  {
    if (pcl_isfinite(in->points[indices_in->at(i)].x) & pcl_isfinite(in->points[indices_in->at(i)].y) &
      pcl_isfinite(in->points[indices_in->at(i)].z))
    {
      valid->push_back(indices_in->at(i));
    }
  }
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> seg;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kd_tree->setInputCloud(in);
  seg.setPointColorThreshold(POINT_COLOR_THRESHOLD);
  seg.setRegionColorThreshold(REGION_COLOR_THRESHOLD);
  seg.setDistanceThreshold(cluster_tolerance_);
  seg.setMinClusterSize(min_cluster_size_);
  seg.setMaxClusterSize(max_cluster_size_);
  seg.setSearchMethod(kd_tree);
  seg.setInputCloud(in);
  seg.setIndices(valid);
  seg.extract(clusters);
}

sensor_msgs::Image Segmenter::createImage(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
                                          const pcl::PointIndices &cluster) const
{
  // determine the bounds of the cluster
  int row_min = numeric_limits<int>::max();
  int row_max = numeric_limits<int>::min();
  int col_min = numeric_limits<int>::max();
  int col_max = numeric_limits<int>::min();

  for (size_t i = 0; i < cluster.indices.size(); i++)
  {
    // calculate row and column of this point
    int row = cluster.indices[i] / in->width;
    int col = cluster.indices[i] - (row * in->width);

    if (row < row_min)
    {
      row_min = row;
    } else if (row > row_max)
    {
      row_max = row;
    }
    if (col < col_min)
    {
      col_min = col;
    } else if (col > col_max)
    {
      col_max = col;
    }
  }

  // create a ROS image
  sensor_msgs::Image msg;

  // set basic information
  msg.header.frame_id = in->header.frame_id;
  msg.header.stamp = ros::Time::now();
  msg.width = col_max - col_min;
  msg.height = row_max - row_min;
  // RGB data
  msg.step = 3 * msg.width;
  msg.data.resize(msg.step * msg.height);
  msg.encoding = sensor_msgs::image_encodings::BGR8;

  // extract the points
  for (int h = 0; h < msg.height; h++)
  {
    for (int w = 0; w < msg.width; w++)
    {
      // extract RGB information
      const pcl::PointXYZRGB &point = in->at(col_min + w, row_min + h);
      // set RGB data
      int index = (msg.step * h) + (w * 3);
      msg.data[index] = point.b;
      msg.data[index + 1] = point.g;
      msg.data[index + 2] = point.r;
    }
  }

  return msg;
}

visualization_msgs::Marker Segmenter::createMarker(const pcl::PCLPointCloud2::ConstPtr &pc) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = pc->header.frame_id;
  marker.ns = "segmentation_objects";

  // default position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // default scale
  marker.scale.x = MARKER_SCALE;
  marker.scale.y = MARKER_SCALE;
  marker.scale.z = MARKER_SCALE;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color.a = 1.0;

  // downsample point cloud for visualization
  pcl::PCLPointCloud2 downsampled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  voxel_grid.setInputCloud(pc);
  voxel_grid.setLeafSize(DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE);
  voxel_grid.filter(downsampled);

  // convert to an easy to use point cloud message
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(downsampled, pc2_msg);
  sensor_msgs::PointCloud pc_msg;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);

  // place in the marker message
  marker.points.resize(pc_msg.points.size());
  int r = 0, g = 0, b = 0;
  for (size_t j = 0; j < pc_msg.points.size(); j++)
  {
    marker.points[j].x = pc_msg.points[j].x;
    marker.points[j].y = pc_msg.points[j].y;
    marker.points[j].z = pc_msg.points[j].z;

    // use average RGB
    uint32_t rgb = *reinterpret_cast<int *>(&pc_msg.channels[0].values[j]);
    b += (int) ((rgb >> 16) & 0x0000ff);
    g += (int) ((rgb >> 8) & 0x0000ff);
    r += (int) ((rgb) & 0x0000ff);
  }

  // set average RGB
  marker.color.r = ((float) r / (float) pc_msg.points.size()) / 255.0;
  marker.color.g = ((float) g / (float) pc_msg.points.size()) / 255.0;
  marker.color.b = ((float) b / (float) pc_msg.points.size()) / 255.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker Segmenter::createMarker(const geometry_msgs::PoseStamped& table_pose,
                                                   const pcl::PointXYZRGB& min_pt, const pcl::PointXYZRGB& max_pt) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = table_pose.header.frame_id;
  marker.ns = "rotated_bbox";

  // default position
//  marker.pose.position.x = bbox_tf.x();
//  marker.pose.position.y = bbox_tf.y();
//  marker.pose.position.z = bbox_tf.z();
//  marker.pose.orientation.x = bbox_q.x();
//  marker.pose.orientation.y = bbox_q.y();
//  marker.pose.orientation.z = bbox_q.z();
//  marker.pose.orientation.w = bbox_q.w();
  marker.pose = table_pose.pose;


  // default scale
  marker.scale.x = max_pt.x - min_pt.x;
  marker.scale.y = max_pt.y - min_pt.y;
  marker.scale.z = max_pt.z - min_pt.z;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::CUBE;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.a = 0.2;

  return marker;
}


visualization_msgs::Marker Segmenter::createMarker(const geometry_msgs::PoseStamped& table_pose,
                                                   const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt) const
{
  visualization_msgs::Marker marker;
  // set header field
  marker.header.frame_id = table_pose.header.frame_id;
  marker.ns = "orig_bbox";

  // default position

//  marker.pose = table_pose.pose;
//  marker.pose.position.x = bbox_tf.x();
//  marker.pose.position.y = bbox_tf.y();
//  marker.pose.position.z = bbox_tf.z();
//  marker.pose.orientation.x = bbox_q.x();
//  marker.pose.orientation.y = bbox_q.y();
//  marker.pose.orientation.z = bbox_q.z();
//  marker.pose.orientation.w = bbox_q.w();
  marker.pose.orientation.w = 1;

  // default scale
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;

  // set the type of marker and our color of choice
  marker.type = visualization_msgs::Marker::POINTS;
  marker.color.g = 1.0;
  marker.color.a = 0.5;

  // place in the marker message
  marker.points.resize(3);
  marker.points[1].x = min_pt.x();
  marker.points[1].y = min_pt.y();
  marker.points[1].z = min_pt.z();
  marker.points[2].x = max_pt.x();
  marker.points[2].y = max_pt.y();
  marker.points[2].z = max_pt.z();

  return marker;
}

void Segmenter::extract(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in, const pcl::IndicesConstPtr &indices_in,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) const
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(in);
  extract.setIndices(indices_in);
  extract.filter(*out);
}

void Segmenter::inverseBound(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
                             const pcl::IndicesConstPtr &indices_in,
                             const pcl::ConditionBase<pcl::PointXYZRGB>::Ptr &conditions,
                             const pcl::IndicesPtr &indices_out) const
{
  // use a temp point cloud to extract the indices
  pcl::PointCloud<pcl::PointXYZRGB> tmp;
  //pcl::ConditionalRemoval<pcl::PointXYZRGB> removal(conditions, true);
  pcl::ConditionalRemoval<pcl::PointXYZRGB> removal(true);
  removal.setCondition(conditions);
  removal.setInputCloud(in);
  removal.setIndices(indices_in);
  removal.filter(tmp);
  *indices_out = *removal.getRemovedIndices();
}

double Segmenter::averageZ(const vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > &v) const
{
  double avg = 0.0;
  for (size_t i = 0; i < v.size(); i++)
  {
    avg += v[i].z;
  }
  return (avg / (double) v.size());
}

//convert from RGB color space to CIELAB color space, adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB)
{
  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double R, G, B, X, Y, Z;

  R = colorRGB[0];
  G = colorRGB[1];
  B = colorRGB[2];

  // linearize sRGB values
  if (R > 0.04045)
    R = pow ( (R + 0.055) / 1.055, 2.4);
  else
    R = R / 12.92;

  if (G > 0.04045)
    G = pow ( (G + 0.055) / 1.055, 2.4);
  else
    G = G / 12.92;

  if (B > 0.04045)
    B = pow ( (B + 0.055) / 1.055, 2.4);
  else
    B = B / 12.92;

  // postponed:
  //    R *= 100.0;
  //    G *= 100.0;
  //    B *= 100.0;

  // linear sRGB -> CIEXYZ
  X = R * 0.4124 + G * 0.3576 + B * 0.1805;
  Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
  Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

  // *= 100.0 including:
  X /= 0.95047;  //95.047;
  //    Y /= 1;//100.000;
  Z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (X > 0.008856)
    X = pow (X, 1.0 / 3.0);
  else
    X = 7.787 * X + 16.0 / 116.0;

  if (Y > 0.008856)
    Y = pow (Y, 1.0 / 3.0);
  else
    Y = 7.787 * Y + 16.0 / 116.0;

  if (Z > 0.008856)
    Z = pow (Z, 1.0 / 3.0);
  else
    Z = 7.787 * Z + 16.0 / 116.0;

  Eigen::Vector3f colorLab;
  colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
  colorLab[1] = static_cast<float> (500.0 * (X - Y));
  colorLab[2] = static_cast<float> (200.0 * (Y - Z));

  return colorLab;
}
