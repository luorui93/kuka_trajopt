#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>  //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>  //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

// polygonal segmentation
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/surface/convex_hull.h>

// oriented bouding box estimation
#include <pcl/features/moment_of_inertia_estimation.h>

// Needed to return a Pose message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
// Needed for ROS Service
#include <pick_and_place_perception/GetTargetPose.h>

class PerceptionPipeline
{
public:
  PerceptionPipeline(ros::NodeHandle& nh)
  {
    nh_ = nh;
    ROS_INFO("Perception service running");

    nh.getParam("processing_node/cloud_topic1", cloud_topic1);
    nh.getParam("processing_node/cloud_topic2", cloud_topic2);
    nh.getParam("processing_node/cloud_topic3", cloud_topic3);
    nh.getParam("processing_node/world_frame", world_frame);
    nh.getParam("processing_node/camera_frame", camera_frame);
    nh.getParam("processing_node/voxel_leaf_size", voxel_leaf_size);
    nh.getParam("processing_node/x_filter_min", x_filter_min);
    nh.getParam("processing_node/x_filter_max", x_filter_max);
    nh.getParam("processing_node/y_filter_min", y_filter_min);
    nh.getParam("processing_node/y_filter_max", y_filter_max);
    nh.getParam("processing_node/z_filter_min", z_filter_min);
    nh.getParam("processing_node/z_filter_max", z_filter_max);
    /*
     * ros::NodeHandle::getParamCached() and ros::param::getCached() provide local caching of parameter data.
     * Using these versions informs the Parameter Server that this node would like to be notified when the parameter is changed,
     * and prevents the node from having to re-lookup the value with the parameter server on subsequent calls.
     */
    nh.getParamCached("processing_node/plane_max_iterations", plane_max_iter);
    nh.getParamCached("processing_node/plane_distance_threshold", plane_dist_thresh);
    nh.getParam("processing_node/cluster_tolerance", cluster_tol);
    nh.getParam("processing_node/cluster_min_size", cluster_min_size);
    nh.getParam("processing_node/cluster_max_size", cluster_max_size);
    nh.getParam("cloud_debug", debug_);

    // TODO (for SwRI. Not for the student) Add check parameters
    //    if (!nh.getParam("stuff", data_)) throw std::runtime_error("Must provide parameter 'stuff'");

    // Create subscriber for pointcloud topics
    // LISTEN FOR POINTCLOUD
    std::string topic1 = nh_.resolveName(cloud_topic1);
    std::string topic2 = nh_.resolveName(cloud_topic2);
//    std::string topic3 = nh_.resolveName(cloud_topic3);
    cloud_sub1_ = nh.subscribe(topic1, 1, &PerceptionPipeline::savePointcloud1, this);
    cloud_sub2_ = nh.subscribe(topic2, 1, &PerceptionPipeline::savePointcloud2, this);

    ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic1);
    auto tc1 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic1, nh_);
    auto tc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic2, nh_);
//    auto tc3 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic3, nh_);

    ROS_INFO_STREAM("Start receving pointcloud");
//    cloud_sub3_ = nh.subscribe(topic3, 1, &PerceptionPipeline::savePointcloud3, this);
    // TRANSFORM POINTCLOUDS FROM CAMERA FRAME TO WORLD FRAME
    tf::TransformListener listener;
    try
    {
      listener.waitForTransform(world_frame, tc1->header.frame_id, ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(world_frame, tc1->header.frame_id, ros::Time(0), stransform1);
      listener.waitForTransform(world_frame, tc2->header.frame_id, ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(world_frame, tc2->header.frame_id, ros::Time(0), stransform2);
//      listener.waitForTransform(world_frame, recent_cloud3->header.frame_id, ros::Time::now(), ros::Duration(6.0));
//      listener.lookupTransform(world_frame, recent_cloud3->header.frame_id, ros::Time(0), stransform3);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // Create ROS interfaces
    server_ = nh.advertiseService("find_pick", &PerceptionPipeline::findPickPose, this);
    if (debug_)
    {
      ROS_INFO("Perception Debug Enabled: Intermediate clouds are being published");
      cropped_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
      object_pub_ = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1, true);
      cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1, true);
      pick_surface_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pick_surface", 1, true);
    }
  }

  bool findPickPose(pick_and_place_perception::GetTargetPoseRequest& req,
                    pick_and_place_perception::GetTargetPoseResponse& res)
  {
    res.succeeded = true;
    ROS_INFO("Perception service running");


//    ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic1);
//    sensor_msgs::PointCloud2::ConstPtr recent_cloud1 =
//        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic1, nh_);
//    ROS_INFO_STREAM("Waiting for a PointCloud2 on topic " << topic2);
//    sensor_msgs::PointCloud2::ConstPtr recent_cloud2 =
//        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic2, nh_);
//    ROS_INFO_STREAM("Waiting for a PointCloud2 on topic " << topic3);
//    sensor_msgs::PointCloud2::ConstPtr recent_cloud3 =
//        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic3, nh_, ros::Duration(2));

    ros::Time start_init = ros::Time::now();
    sensor_msgs::PointCloud2 transformed_cloud1, transformed_cloud2, transformed_cloud3;
    pcl_ros::transformPointCloud(world_frame, stransform1, *recent_cloud1_, transformed_cloud1);
    pcl_ros::transformPointCloud(world_frame, stransform2, *recent_cloud2_, transformed_cloud2);
//    pcl_ros::transformPointCloud(world_frame, stransform3, *recent_cloud3, transformed_cloud3);

    // COMBINE POINT CLOUDS
    sensor_msgs::PointCloud2 temp, transformed_cloud;
    pcl::concatenatePointCloud(transformed_cloud1, transformed_cloud2, transformed_cloud);
//    pcl::concatenatePointCloud(temp, transformed_cloud3, transformed_cloud);

    // CONVERT POINTCLOUD ROS->PCL
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud3, cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);
    //    pcl::fromROSMsg(transformed_cloud1, cloud);

    // MAKE TIMERS FOR PROCESS (OPTIONAL)
//    ros::Time start_init = ros::Time::now();

    /* ========================================
     * Fill Code: VOXEL GRID
     * ========================================*/
    // input cloud must be a pointer, so we make a new cloud_ptr from the cloud object   (IMPORTANT USAGE RUI)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    // output cloud - set up as pointer to ease transition into further processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    // create an instance of the pcl VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    // ENTER CODE HERE: Set input cloud
    voxel_filter.setInputCloud(cloud_ptr);
    // ENTER CODE HERE: Set Leaf Size
    voxel_filter.setLeafSize(voxel_leaf_size,voxel_leaf_size,voxel_leaf_size);
    // ENTER CODE HERE: Filter the cloud
    voxel_filter.filter(*cloud_voxel_filtered);

    ROS_INFO_STREAM("Original cloud  had " << cloud_voxel_filtered->size() << " points");
    ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

    /* ========================================
     * Fill Code: PASSTHROUGH FILTER(S)
     * ======================*/
    // step 1- filter in x
    pcl::PointCloud<pcl::PointXYZ> xf_cloud = cloud;
    pcl::PointCloud<pcl::PointXYZ> yf_cloud = cloud;
    pcl::PointCloud<pcl::PointXYZ> zf_cloud = cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_voxel_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min, x_filter_max);
    pass_x.filter(xf_cloud);

    // pass to filter in y
    // ENTER CODE HERE A new pointer is needed for input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    // ENTER CODE HERE
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    // ENTER CODE HERE
    pass_y.setInputCloud(xf_cloud_ptr);
    // ENTER CODE HERE
    pass_y.setFilterFieldName("y");
    // ENTER CODE HERE
    pass_y.setFilterLimits(y_filter_min, y_filter_max);
    // ENTER CODE HERE    
    pass_y.filter(yf_cloud);

    // pass to filter in z
    // ENTER CODE HERE A new pointer is needed for input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    // ENTER CODE HERE
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    // ENTER CODE HERE
    pass_z.setInputCloud(yf_cloud_ptr);
    // ENTER CODE HERE
    pass_z.setFilterFieldName("z");
    // ENTER CODE HERE
    pass_z.setFilterLimits(z_filter_min, z_filter_max);
    // ENTER CODE HERE
    pass_z.filter(zf_cloud);

    /* ========================================
     * Fill Code: CROPBOX (OPTIONAL)
     * Instead of three passthrough filters, the cropbox filter can be used
     * The user should choose one or the other method
     * ========================================*/
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    // ENTER CODE HERE
    //possible solution
    //pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
    //boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    //boxFilter.setInputCloud(body);
    //boxFilter.filter(*bodyFiltered);

    ROS_INFO_STREAM("Cropped point cloud: " << zf_cloud.points.size() << " data points.");
    // Publish cropped cloud
    if (debug_)
    {
      sensor_msgs::PointCloud2::Ptr cropped(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(zf_cloud, *cropped);
      //    pcl::toROSMsg(xyz_filtered_cloud, *cropped);
      cropped->header.frame_id = world_frame;
      cropped->header.stamp = ros::Time::now();
      cropped_pub_.publish(*cropped);
    }

    /* ========================================
     * Fill Code: PLANE SEGEMENTATION - REMOVE THE WORKTABLE
     * http://pointclouds.org/documentation/tutorials/planar_segmentation.php
     * ========================================*/
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(
//        new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));  // this passes in either passthrough or crop filtered cloud.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    // ENTER CODE HERE: Set max iterations
//    seg.setMaxIterations(plane_max_iter);
//    // ENTER CODE HERE: Set distance threshold
//    seg.setDistanceThreshold(plane_dist_thresh);
//    // Segment the largest planar component from the cropped cloud
//    seg.setInputCloud(cropped_cloud);
//    seg.segment(*inliers, *coefficients);
//    if (inliers->indices.size() > 0)
//    {
//      // Extract the planar inliers from the input cloud
//      pcl::ExtractIndices<pcl::PointXYZ> extract;
//      extract.setInputCloud(cropped_cloud);
//      extract.setIndices(inliers);
//      //false to return only inlier indices
//      extract.setNegative(false);

//      // Get the points associated with the planar surface
//      extract.filter(*cloud_plane);
//      ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size()
//                                                                       << " data points.");

//      // Remove the planar inliers, extract the rest
//      extract.setNegative(true);
//      extract.filter(*cloud_f);
//      ROS_INFO_STREAM("PointCloud without table plane: " << cloud_f->points.size()
//                                                                       << " data points.");
//    }
//    else
//    {
//      ROS_WARN_STREAM("Could not estimate a planar model for the given dataset. Proceeding with full point cloud.");
//      *cloud_f = *cropped_cloud;
//      res.succeeded = false;
//    }
    /* ========================================
     * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
     * ========================================*/
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
//    *cloud_filtered = *cloud_f;
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ENTER CODE HERE: Set cluster tolerance
    ec.setClusterTolerance(cluster_tol);
    // ENTER CODE HERE: Set minimum cluster size
    ec.setMinClusterSize(cluster_min_size);
    // ENTER CODE HERE: Set maximum cluster size
    ec.setMaxClusterSize(cluster_max_size);
    // ENTER CODE HERE: Set search method to tree
    ec.setSearchMethod(tree);
    // ENTER CODE HERE: Set input cloud
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
      clusters.push_back(cloud_cluster);
      sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
      pc2_clusters.push_back(tempROSMsg);
    }
    if (pc2_clusters.size() > 0)
    {
      pc2_clusters.at(0)->header.frame_id = world_frame;
      pc2_clusters.at(0)->header.stamp = ros::Time::now();
      if (debug_)
      {
        cluster_pub_.publish(pc2_clusters.at(0));
      }
    }
    else
    {
      ROS_WARN_STREAM("Clustering failed. Proceeding with full point cloud.");
      clusters.push_back(cloud_filtered);
      res.succeeded = false;
    }
    /* ========================================
     * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
     * ========================================*/
    //The Cluster will be stored based on their size from big to small
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr = clusters.at(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // ENTER CODE HERE: Set input cloud
    sor.setInputCloud(cluster_cloud_ptr);
    // ENTER CODE HERE: Set meanK
    sor.setMeanK(50);
    // ENTER CODE HERE: set StddevMulThresh
    sor.setStddevMulThresh(1);
    // ENTER CODE HERE: filter
    sor.filter(*sor_cloud_filtered);

    // Publish object as point cloud
    if (debug_)
    {
      sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*sor_cloud_filtered, *pc2_cloud);
      pc2_cloud->header.frame_id = world_frame;
      pc2_cloud->header.stamp = ros::Time::now();

      object_pub_.publish(*pc2_cloud);
    }

    /* ========================================
     * Fill Code: PLANE SEGEMENTATION OF PICK SURFACE - FIND TOP OF BOX
     * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planeTop(new pcl::PointCloud<pcl::PointXYZ>());
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> segTop;
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);

    // ENTER CODE HERE: SetOptimizeCoefficients
    segTop.setOptimizeCoefficients(true);
    // ENTER CODE HERE: SetModelType (May need to be different from above)
    segTop.setModelType(pcl::SACMODEL_PLANE);
    // ENTER CODE HERE: Set method type
    segTop.setMethodType(pcl::SAC_RANSAC);
    // ENTER CODE HERE: Set perpendicular axis
    segTop.setAxis(axis);
    // ENTER CODE HERE: Set epsAngle (around 5 degrees, edited by Rui)
    segTop.setEpsAngle(0.08);
    // ENTER CODE HERE: Set Max iter
    segTop.setMaxIterations(plane_max_iter);
    // ENTER CODE HERE: set distance threshold
    segTop.setDistanceThreshold(plane_dist_thresh);
    // Segment the largest planar component perpendicular to the world z axis from the cropped cloud
    // If statistical outlier removal is enabled, use sor_cloud_filtered here to replace cluster_cloud_ptr
    segTop.setInputCloud(cluster_cloud_ptr);
    segTop.segment(*inliers2, *coefficients2);
    if (inliers2->indices.size() > 0)
    {
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extractTop;
      extractTop.setInputCloud(cluster_cloud_ptr);
      extractTop.setIndices(inliers2);
      extractTop.setNegative(false);

      // Get the points associated with the planar surface
      extractTop.filter(*cloud_planeTop);
      ROS_INFO_STREAM("PointCloud representing the top planar component: " << cloud_planeTop->points.size()
                                                                           << " data points.");
    }
    else
    {
      ROS_WARN_STREAM("Could not estimate a planar model for the given dataset. Proceeding with full point cloud.");
      *cloud_planeTop = *cluster_cloud_ptr;
      res.succeeded = false;
    }

    // OPTIONAL TIMERS
    ros::Time finish_process = ros::Time::now();
    ros::Duration total_process = finish_process - start_init;
    ROS_INFO_STREAM("Point Cloud processing took " << total_process << " s");

    /* ========================================
     * CONVERT POINTCLOUD PCL->ROS
     * PUBLISH CLOUD
     * Fill Code: UPDATE AS NECESSARY
     * ========================================*/
    sensor_msgs::PointCloud2::Ptr pc2_cloud_top(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_planeTop, *pc2_cloud_top);
    pc2_cloud_top->header.frame_id = world_frame;
    pc2_cloud_top->header.stamp = ros::Time::now();
    if (debug_)
    {
      pick_surface_pub_.publish(*pc2_cloud_top);
    }

    /* ========================================
     * FIND ORIENTATION OF POINTCLOUD  http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
     * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr top_plane(new pcl::PointCloud<pcl::PointXYZ>(*cloud_planeTop));
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(top_plane);
    feature_extractor.compute();
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);



    /* ========================================
     * Fill Code: BROADCAST PART TRANSFORM
     * ========================================*/
    Eigen::Vector4f origin;
    // Compute centroid of the top of the pick plane
    // ENTER CODE HERE: Compute the centroid and store in origin
    pcl::compute3DCentroid(*cloud_planeTop, origin);
    geometry_msgs::Pose part_pose;
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f ea = rotational_matrix_OBB.eulerAngles(2,0,2);
    tf::Quaternion t_quat(0,0,ea[0]);  //axis(y,x,z)
    // ENTER CODE HERE: Set x
    part_pose.position.x = origin[0];
    // ENTER CODE HERE: Set y
    part_pose.position.y = origin[1];
    // ENTER CODE HERE: Set z
    part_pose.position.z = origin[2];
    // ENTER CODE HERE: Set rot x
    part_pose.orientation.x = 0;//t_quat.x();
    // ENTER CODE HERE: Set rot y
    part_pose.orientation.y = 0;//t_quat.y();
    // ENTER CODE HERE: Set rot z
    part_pose.orientation.z = 0;//t_quat.z();
    part_pose.orientation.w = 1;//t_quat.w();

    /* ========================================
     * FIND BOUNDING BOX
     * ========================================*/
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud_planeTop, min_pt, max_pt);
    res.min_pt.x = min_pt[0];
    res.min_pt.y = min_pt[1];
    res.min_pt.z = min_pt[2];
    res.max_pt.x = max_pt[0];
    res.max_pt.y = max_pt[1];
    res.max_pt.z = max_pt[2];

    // Store the returned values of the service as defined in the .srv file
    res.target_pose = part_pose;

    ROS_INFO("Perception service returning");

    // Publish the pick point as a TF for visualization
    if (debug_)
    {
      tf::Transform part_transform;
      part_transform.setOrigin(tf::Vector3(origin[0], origin[1], origin[2]));
      tf::Quaternion q(0,0,0,1);
      part_transform.setRotation(q);
      br_.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));
    }

    return true;
  }

  void savePointcloud1(const sensor_msgs::PointCloud2::ConstPtr msg) {
     recent_cloud1_ = msg;
  }

  void savePointcloud2(const sensor_msgs::PointCloud2::ConstPtr msg) {
     recent_cloud2_ = msg;
  }

  void savePointcloud3(const sensor_msgs::PointCloud2::ConstPtr msg) {
     recent_cloud3_ = msg;
  }
private:
  ros::ServiceServer server_;
  ros::Publisher cropped_pub_, object_pub_, cluster_pub_, pick_surface_pub_;
  ros::Subscriber cloud_sub1_, cloud_sub2_, cloud_sub3_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;
  sensor_msgs::PointCloud2::ConstPtr recent_cloud1_, recent_cloud2_, recent_cloud3_;
  tf::StampedTransform stransform1, stransform2, stransform3;

  // Configuration data
  std::string cloud_topic1, cloud_topic2, cloud_topic3, world_frame, camera_frame;
  double voxel_leaf_size;
  double x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  double plane_max_iter, plane_dist_thresh;
  double cluster_tol;
  int cluster_min_size, cluster_max_size;
  bool debug_ = true;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "processing_node");
  ros::NodeHandle nh;

  PerceptionPipeline pipeline(nh);
  ROS_INFO("Perception server ready to find pick targets");
  ros::spin();

  return 0;
}
