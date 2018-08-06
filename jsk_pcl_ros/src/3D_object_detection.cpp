/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/euclidean_cluster_extraction_nodelet.h" 
#include <jsk_recognition_utils/pcl_util.h>

using namespace std;
using namespace pcl;

namespace jsk_pcl_ros
{

  void EuclideanClustering::extract(
    const sensor_msgs::PointCloud2ConstPtr &input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    // list up indices which are not NaN to use
    // organized pointcloud
    pcl::PointIndices::Ptr nonnan_indices (new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ p = cloud->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
        nonnan_indices->indices.push_back(i);
      }
    } 

    if (nonnan_indices->indices.size() == 0) {
      // if input points is 0, publish empty data as result
      jsk_recognition_msgs::ClusterPointIndices result;
      result.header = input->header;
      result_pub_.publish(result);
      // do nothing and return it
      jsk_recognition_msgs::Int32Stamped::Ptr cluster_num_msg (new jsk_recognition_msgs::Int32Stamped);
      cluster_num_msg->header = input->header;
      cluster_num_msg->data = 0;
      cluster_num_pub_.publish(cluster_num_msg);
      return;
    } //looks like imp
    
    EuclideanClusterExtraction<pcl::PointXYZ> impl;
    {
      jsk_topic_tools::ScopedTimer timer = kdtree_acc_.scopedTimer();
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
      tree->setInputCloud (cloud);
      impl.setClusterTolerance (tolerance);
      impl.setMinClusterSize (minsize_);
      impl.setMaxClusterSize (maxsize_);
      impl.setSearchMethod (tree);
      impl.setIndices(nonnan_indices);
      impl.setInputCloud (cloud);

      //initialization of all variables used for correspondence grouping
      //Algorithm params
	  bool show_keypoints_ (false);
	  bool show_correspondences_ (false);
	  bool use_cloud_resolution_ (false);
	  bool use_hough_ (true);
	  float model_ss_ (0.01f);
	  float scene_ss_ (0.03f);
	  float rf_rad_ (0.015f);
	  float descr_rad_ (0.02f);
	  float cg_size_ (0.01f);
	  float cg_thresh_ (5.0f);
	  
	  //hard coding model and scene filename
	  //Note: Later we will have different model filenames and scene with be real-time point cloud input
	  model_filename_ = milk.pcd;
      scene_filename_ = milk_cartoon_all_small_clorox.pcd;
    }
    
    //for now skipping cloud resolution code
    
    {
      jsk_topic_tools::ScopedTimer timer = segmentation_acc_.scopedTimer();
      impl.extract (cluster_indices);
    }
    
    // Publish result indices 
    //San: According to me the main code goes here
    //3D code
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

    //
    //  Load clouds
    //
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
       std::cout << "Error loading model cloud." << std::endl;
       return (-1);
    }
   if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
   {
       std::cout << "Error loading scene cloud." << std::endl;
       return (-1);
   }

   // no resolution code merged for now
   
   //
   //  Compute Normals
   //
   pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
   norm_est.setKSearch (10);
   norm_est.setInputCloud (model);
   norm_est.compute (*model_normals);

   norm_est.setInputCloud (scene);
   norm_est.compute (*scene_normals);

   //
   //  Downsample Clouds to Extract keypoints
   //

   pcl::UniformSampling<PointType> uniform_sampling;
   uniform_sampling.setInputCloud (model);
   uniform_sampling.setRadiusSearch (model_ss_);
   uniform_sampling.filter (*model_keypoints);
   std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

   uniform_sampling.setInputCloud (scene);
   uniform_sampling.setRadiusSearch (scene_ss_);
   uniform_sampling.filter (*scene_keypoints);
   std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


   //
   //  Compute Descriptor for keypoints
   //
   pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
   descr_est.setRadiusSearch (descr_rad_);

   descr_est.setInputCloud (model_keypoints);
   descr_est.setInputNormals (model_normals);
   descr_est.setSearchSurface (model);
   descr_est.compute (*model_descriptors);

   descr_est.setInputCloud (scene_keypoints);
   descr_est.setInputNormals (scene_normals);
   descr_est.setSearchSurface (scene);
   descr_est.compute (*scene_descriptors);
   
   //
   //  Find Model-Scene Correspondences with KdTree
   //
   pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
 
   pcl::KdTreeFLANN<DescriptorType> match_search;
   match_search.setInputCloud (model_descriptors);

   //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
   for (size_t i = 0; i < scene_descriptors->size (); ++i)
   {
     std::vector<int> neigh_indices (1);
     std::vector<float> neigh_sqr_dists (1);
     if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
     {
       continue;
     }
     int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
     if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
     {
       pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
       model_scene_corrs->push_back (corr);
     }
   }
   std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

   //
   //  Actual Clustering
   //
   std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
   std::vector<pcl::Correspondences> clustered_corrs;

   //  Using Hough3D
   if (use_hough_)
   {
     //
     //  Compute (Keypoints) Reference Frames only for Hough
     //
     pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
     pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());
 
     pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
     rf_est.setFindHoles (true);
     rf_est.setRadiusSearch (rf_rad_);

     rf_est.setInputCloud (model_keypoints);
     rf_est.setInputNormals (model_normals);
     rf_est.setSearchSurface (model);
     rf_est.compute (*model_rf);

     rf_est.setInputCloud (scene_keypoints);
     rf_est.setInputNormals (scene_normals);
     rf_est.setSearchSurface (scene);
     rf_est.compute (*scene_rf);

     //  Clustering
     pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
     clusterer.setHoughBinSize (cg_size_);
     clusterer.setHoughThreshold (cg_thresh_);
     clusterer.setUseInterpolation (true);
     clusterer.setUseDistanceWeight (false);

     clusterer.setInputCloud (model_keypoints);
     clusterer.setInputRf (model_rf);
     clusterer.setSceneCloud (scene_keypoints);
     clusterer.setSceneRf (scene_rf);
     clusterer.setModelSceneCorrespondences (model_scene_corrs);

     //clusterer.cluster (clustered_corrs);
     clusterer.recognize (rototranslations, clustered_corrs);
   }
   else // Using GeometricConsistency
   {
     pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
     gc_clusterer.setGCSize (cg_size_);
     gc_clusterer.setGCThreshold (cg_thresh_);
 
     gc_clusterer.setInputCloud (model_keypoints);
     gc_clusterer.setSceneCloud (scene_keypoints);
     gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

     //gc_clusterer.cluster (clustered_corrs);
     gc_clusterer.recognize (rototranslations, clustered_corrs);
   }

   //
   //  Output results
   //
   std::cout << "Model instances found: " << rototranslations.size () << std::endl;
   for (size_t i = 0; i < rototranslations.size (); ++i)
   {
     std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
     std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
 
     // Print the rotation matrix and translation vector
     Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
     Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
 
     printf ("\n");
     printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
     printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
     printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
     printf ("\n");
     printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
   }
 
 //Need to figure out how the result point cloud should be visualized 
    

  bool EuclideanClustering::serviceCallback(
    jsk_recognition_msgs::EuclideanSegment::Request &req,
    jsk_recognition_msgs::EuclideanSegment::Response &res)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.input, *cloud);

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
    pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif

    vector<pcl::PointIndices> cluster_indices;
    EuclideanClusterExtraction<pcl::PointXYZ> impl;
    double tor;
    if ( req.tolerance < 0) {
      tor = tolerance;
    } else {
      tor = req.tolerance;
    }
    impl.setClusterTolerance (tor);
    impl.setMinClusterSize (minsize_);
    impl.setMaxClusterSize (maxsize_);
    impl.setSearchMethod (tree);
    impl.setInputCloud (cloud);
    impl.extract (cluster_indices);

    res.output.resize( cluster_indices.size() );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(req.input, *pcl_cloud);
    pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
    ex.setInputCloud(pcl_cloud);
#else
    pcl::ExtractIndices<sensor_msgs::PointCloud2> ex;
    ex.setInputCloud ( boost::make_shared< sensor_msgs::PointCloud2 > (req.input) );
#endif
    for ( size_t i = 0; i < cluster_indices.size(); i++ ) {
      ex.setIndices ( boost::make_shared< pcl::PointIndices > (cluster_indices[i]) );
      ex.setNegative ( false );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
      pcl::PCLPointCloud2 output_cloud;
      ex.filter ( output_cloud );
      pcl_conversions::fromPCL(output_cloud, res.output[i]);
#else
      ex.filter ( res.output[i] );
#endif
    }
    return true;
  }

  void EuclideanClustering::onInit()
  {
    DiagnosticNodelet::onInit();

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EuclideanClustering::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    ////////////////////////////////////////////////////////
    // Publisher
    ////////////////////////////////////////////////////////
    result_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices> (*pnh_, "output", 1);
    cluster_num_pub_ = advertise<jsk_recognition_msgs::Int32Stamped> (*pnh_, "cluster_num", 1);
    service_ = pnh_->advertiseService(pnh_->resolveName("euclidean_clustering"),
                                      &EuclideanClustering::serviceCallback, this);

    onInitPostProcess();
  }

  void EuclideanClustering::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscription
    ////////////////////////////////////////////////////////
    sub_input_ = pnh_->subscribe("input", 1, &EuclideanClustering::extract, this);
  }

  void EuclideanClustering::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void EuclideanClustering::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(
      diagnostic_msgs::DiagnosticStatus::OK,
      "EuclideanSegmentation running");

      jsk_topic_tools::addDiagnosticInformation(
        "Kdtree Construction", kdtree_acc_, stat);
      jsk_topic_tools::addDiagnosticInformation(
        "Euclidean Segmentation", segmentation_acc_, stat);
      stat.add("Cluster Num (Avg.)", cluster_counter_.mean());
      stat.add("Max Size of the cluster", maxsize_);
      stat.add("Min Size of the cluster", minsize_);
      stat.add("Cluster tolerance", tolerance);
      stat.add("Tracking tolerance", label_tracking_tolerance);
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
  
  void EuclideanClustering::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tolerance = config.tolerance;
    label_tracking_tolerance = config.label_tracking_tolerance;
    maxsize_ = config.max_size;
    minsize_ = config.min_size;
  }

  std::vector<pcl::PointIndices> EuclideanClustering::pivotClusterIndices(
    std::vector<int>& pivot_table,
    std::vector<pcl::PointIndices>& cluster_indices)
  {
    std::vector<pcl::PointIndices> new_cluster_indices;
    new_cluster_indices.resize(pivot_table.size());
    for (size_t i = 0; i < pivot_table.size(); i++) {
      new_cluster_indices[i] = cluster_indices[pivot_table[i]];
    }
    return new_cluster_indices;
  }
  std::vector<int> EuclideanClustering::buildLabelTrackingPivotTable(
    double* D, Vector4fVector cogs, Vector4fVector new_cogs,
    double label_tracking_tolerance)
  {
    std::vector<int> pivot_table;
    // initialize pivot table
    pivot_table.resize(cogs.size());
    for (size_t i = 0; i < pivot_table.size(); i++)
      pivot_table[i] = i;
    for (size_t pivot_counter = 0; pivot_counter < pivot_table.size();
         pivot_counter++)
    {
      double minimum_distance = DBL_MAX;
      size_t minimum_previous_index = 0;
      size_t minimum_next_index = 0;
      for (size_t i = 0; i < cogs.size(); i++)
      {
        for (size_t j = 0; j < new_cogs.size(); j++)
        {
          double distance = D[i * cogs.size() + j];
          //ROS_INFO("distance %lux%lu: %f", i, j, distance);
          if (distance < minimum_distance)
          {
            minimum_distance = distance;
            minimum_previous_index = i;
            minimum_next_index = j;
          }
        }
      }
      if (minimum_distance > label_tracking_tolerance)
      {
        // ROS_WARN("minimum tracking distance exceeds tolerance: %f > %f",
        //          minimum_distance, label_tracking_tolerance);
        std::vector<int> dummy;
        return dummy;
      }
      pivot_table[minimum_previous_index] = minimum_next_index;
      // fill the D matrix with DBL_MAX
      for (size_t j = 0; j < new_cogs.size(); j++)
      {
        D[minimum_previous_index * cogs.size() + j] = DBL_MAX;
      }
    }
    return pivot_table;
  }

  void EuclideanClustering::computeDistanceMatrix(
    double* D,
    Vector4fVector& old_cogs,
    Vector4fVector& new_cogs)
  {
    for (size_t i = 0; i < old_cogs.size(); i++) {
      Eigen::Vector4f previous_cog = old_cogs[i];
      for (size_t j = 0; j < new_cogs.size(); j++) {
        Eigen::Vector4f next_cog = new_cogs[j];
        double distance = (next_cog - previous_cog).norm();
        D[i * old_cogs.size() + j] = distance;
      }
    }
  }

  void EuclideanClustering::computeCentroidsOfClusters(
    Vector4fVector& ret,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices> cluster_indices)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    ret.resize(cluster_indices.size());
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
      // build pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      for (size_t j = 0; j < cluster_indices[i].indices.size(); j++)
      {
        segmented_indices->indices.push_back(cluster_indices[i].indices[j]);
      }
      extract.setIndices(segmented_indices);
      extract.filter(*segmented_cloud);
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*segmented_cloud, center);
      ret[i] = center;
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EuclideanClustering, nodelet::Nodelet);

