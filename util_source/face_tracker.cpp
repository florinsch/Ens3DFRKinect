#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include "/usr/local/src/pcl-trunk-svn/doc/tutorials/content/sources/template_alignment/template_alignment.cpp"

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

using namespace pcl::tracking;

#define USE_NORMALS false

template <typename PointType>
class OpenNIFaceTracking
{
  public:
#if USE_NORMALS == true
  typedef pcl::PointXYZRGBANormal RefPointType;
  //typedef pcl::PointXYZINormal RefPointType;
#else
  typedef pcl::PointXYZRGBA RefPointType;
  //typedef pcl::PointXYZ RefPointType;
#endif

  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  //typedef KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
  //typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
  //typedef typename ParticleFilter::CoherencePtr CoherencePtr;

  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;

  OpenNIFaceTracking (const std::string& device_id, int thread_nr, double downsampling_grid_size,
                      double pass_min, double pass_max,
                      bool use_convex_hull, bool visualize_non_downsample, bool visualize_particles,
                      bool use_fixed, std::string template_name)
  : viewer_ ("PCL OpenNI Tracking Viewer")
  , device_id_ (device_id)
  , new_cloud_ (false)
  , ne_ (thread_nr)
  , counter_ (0)
  , use_convex_hull_ (use_convex_hull)
  , visualize_non_downsample_ (visualize_non_downsample)
  , visualize_particles_ (visualize_particles)
  , downsampling_grid_size_ (downsampling_grid_size)
  , pass_min_ (pass_min)
  , pass_max_ (pass_max)
  , template_name_ (template_name)
  {
    // Set normal estimation search using Kdtree
    KdTreePtr tree (new KdTree (false));
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    if (use_fixed)
    {
      boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new ParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker_ = tracker;
    }
    else
    {
      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker->setMaximumParticleNum (500);
      tracker->setDelta (0.99);
      tracker->setEpsilon (0.2);
      ParticleT bin_size;
      bin_size.x = bin_size.y = bin_size.z = 0.1;
      bin_size.roll = bin_size.pitch = bin_size.yaw = 0.1;
      tracker->setBinSize (bin_size);
      tracker_ = tracker;
    }
    
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);

    tracker_->setResolutionOfChangeDetection(0.1);
    tracker_->setIntervalOfChangeDetection(3);
    //tracker_->setMinPointsOfChangeDetection(50);
    tracker_->setUseChangeDetector(false);
    
    tracker_->setParticleNum (400);
    tracker_->setResampleLikelihoodThr(0.0001); // ?????
    tracker_->setUseNormal (false); //TODO: toggle normal usage here

    // setup coherences
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    //NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
    //   (new NearestPairPointCloudCoherence<RefPointType> ());
    
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);
    
#if USE_NORMALS == true
    boost::shared_ptr<NormalCoherence<RefPointType> > normal_coherence
      = boost::shared_ptr<NormalCoherence<RefPointType> > (new NormalCoherence<RefPointType> ());
    coherence->addPointCoherence(normal_coherence);
#endif

    //boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
    //  = boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
    //color_coherence->setWeight (0.1);
    //coherence->addPointCoherence (color_coherence);

    //TODO: change search method here
    //boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);

    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
  }

  bool
  drawParticles (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles)
    {
      if (visualize_particles_)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (size_t i = 0; i < particles->points.size (); i++)
        {
          pcl::PointXYZ point;
          
          point.x = particles->points[i].x;
          point.y = particles->points[i].y;
          point.z = particles->points[i].z;
          particle_cloud->points.push_back (point);
        }
        
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> particle_color (particle_cloud, 0, 255, 125);
          if (!viz.updatePointCloud (particle_cloud, particle_color, "particle cloud"))
            viz.addPointCloud (particle_cloud, particle_color, "particle cloud");
        }
      }
      return true;
    }
    else
    {
      PCL_WARN ("no particles\n");
      return false;
    }
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer &viz)
  {
    //CloudPtr result_cloud (new Cloud ());
    Eigen::Affine3f trans = tracker_->toEigenMatrix ( tracker_->getResult () );

    // move a little bit for better visualization
    trans.translation () += Eigen::Vector3f (0.0, 0.0, -0.005);
    reference_.reset(new Cloud ());
    pcl::transformPointCloud<PointType> (*(tracker_->getReferenceCloud ()), *reference_, trans);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> red_color (reference_, 255, 0, 0);
    if (!viz.updatePointCloud (reference_, red_color, "resultcloud"))
      viz.addPointCloud (reference_, red_color, "resultcloud");
  }
  
  void
  drawAverageNormal(pcl::visualization::PCLVisualizer &viz)
  {
    ref_normals_.reset (new pcl::PointCloud<pcl::Normal>);
    normalEstimation (reference_, *ref_normals_);

    Eigen::Vector4f ctr;
    pcl::compute3DCentroid<PointType> (*reference_, ctr);
    pcl::PointXYZ ctrOid (ctr[0], ctr[1], ctr[2]);
    pcl::PointXYZ avgNrml (0, 0, 0);

    double size = ref_normals_->points.size ();
    for (size_t i = 0; i < size; i++)
    {
      avgNrml.x += ref_normals_->points[i].normal_x;
      avgNrml.y += ref_normals_->points[i].normal_y;
      avgNrml.z += ref_normals_->points[i].normal_z;
    }
    avgNrml.x /= size; avgNrml.y /= size; avgNrml.z /= size;

    //double distToSensor = sqrt( pow(ctrOid.x,2)+pow(ctrOid.y,2)+pow(ctrOid.z,2) );
    double lengthAvgNrml = sqrt( pow(avgNrml.x-ctrOid.x,2)+pow(avgNrml.y-ctrOid.y,2)+pow(avgNrml.z-ctrOid.z,2) );
    avgNrml.x /= lengthAvgNrml; avgNrml.y /= lengthAvgNrml; avgNrml.z /= lengthAvgNrml;

    viz.removeShape("avgNorm");
    viz.addArrow(avgNrml, ctrOid, 100, 255, 100, "avgNorm");
  }
  
  void
  drawInfo (pcl::visualization::PCLVisualizer &viz)
  {
    viz.removeShape("N");
    viz.addText ((boost::format ("number of Reference PointClouds: %d") % tracker_->getReferenceCloud ()->points.size ()).str (),
                 10, 15, 15, 1.0, 1.0, 1.0, "N");

    viz.removeShape ("M");
    viz.addText ((boost::format ("number of Measured PointClouds:  %d") % cloud_pass_downsampled_->points.size ()).str (),
                 10, 30, 15, 1.0, 1.0, 1.0, "M");

    viz.removeShape ("tracking");
    viz.addText ((boost::format ("tracking:        %f fps") % (1.0 / tracking_time_)).str (),
                 10, 45, 15, 1.0, 1.0, 1.0, "tracking");

    viz.removeShape ("downsampling");
    viz.addText ((boost::format ("downsampling:    %f fps") % (1.0 / downsampling_time_)).str (),
                 10, 60, 15, 1.0, 1.0, 1.0, "downsampling");

    viz.removeShape ("computation");
    viz.addText ((boost::format ("computation:     %f fps") % (1.0 / computation_time_)).str (),
                 10, 75, 15, 1.0, 1.0, 1.0, "computation");

    viz.removeShape ("particles");
    viz.addText ((boost::format ("particles:     %d") % tracker_->getParticles ()->points.size ()).str (),
                 10, 90, 15, 1.0, 1.0, 1.0, "particles");
  }

  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    boost::mutex::scoped_lock lock (mtx_);
    
    if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
    }
    
    if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      if (!visualize_non_downsample_)
        cloud_pass = cloud_pass_downsampled_;
      else
        cloud_pass = cloud_pass_;
      
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
        {
          viz.addPointCloud (cloud_pass, "cloudpass");
          viz.resetCameraViewpoint ("cloudpass");
        }
    }

    if (new_cloud_ && reference_)
    {
      if (drawParticles (viz))
      {
        drawResult (viz);
        drawAverageNormal(viz);
        drawInfo (viz);
      }
    }
    new_cloud_ = false;
  }

  // filtering and down-sampling
  void
  filterPassThrough (const CloudConstPtr &cloud, Cloud &result,
                          double min = 0.2, double max = 5.5)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min, max);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
    FPS_CALC_END("filterPassThrough");
  }

  void
  gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (cloud);
    grid.filter (result);
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSampleApprox");
  }

  void
  gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (cloud);
    grid.filter (result);
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }

  // stuff to remove planes out of the point cloud
  void
  planeSegmentation (const CloudConstPtr &cloud,
                            pcl::ModelCoefficients &coefficients,
                            pcl::PointIndices &inliers)
  {
    FPS_CALC_BEGIN;
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud);
    seg.segment (inliers, coefficients);
    FPS_CALC_END("planeSegmentation");
  }

  void
  planeProjection (const CloudConstPtr &cloud,
                          Cloud &result,
                          const pcl::ModelCoefficients::ConstPtr &coefficients)
  {
    FPS_CALC_BEGIN;
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (result);
    FPS_CALC_END("planeProjection");
  }

  void
  convexHull (const CloudConstPtr &cloud,
                    Cloud &result,
                    std::vector<pcl::Vertices> &hull_vertices)
   {
     FPS_CALC_BEGIN;
     pcl::ConvexHull<PointType> chull;
     chull.setInputCloud (cloud);
     chull.reconstruct (result, hull_vertices);
     FPS_CALC_END("convexHull");
   }

  void
  extractNonPlanePoints (const CloudConstPtr &cloud,
                                const CloudConstPtr &cloud_hull,
                                Cloud &result)
  {
    pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
    pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
    polygon_extract.setHeightLimits (0.01, 10.0);
    polygon_extract.setInputPlanarHull (cloud_hull);
    polygon_extract.setInputCloud (cloud);
    polygon_extract.segment (*inliers_polygon);
    {
      pcl::ExtractIndices<PointType> extract_positive;
      extract_positive.setNegative (false);
      extract_positive.setInputCloud (cloud);
      extract_positive.setIndices (inliers_polygon);
      extract_positive.filter (result);
    }
  }

  void
  removePlaneSegments (const CloudConstPtr &cloud, RefCloud &result)
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<pcl::Vertices> hull_vertices;
    CloudPtr cloud_projected (new Cloud);
    CloudPtr cloud_hull (new Cloud);

    planeSegmentation (cloud, *coefficients, *inliers);
    if (inliers->indices.size () > 3)
    {
      planeProjection (cloud, *cloud_projected, coefficients);
      convexHull (cloud_projected, *cloud_hull, hull_vertices);
      extractNonPlanePoints (cloud, cloud_hull, result);
    }
    else
    {
      PCL_WARN("Plane Segments Exclusion Failed");
    }
  }

  // cloud cleanup & processing
  void
  removeZeroPoints (const CloudConstPtr &cloud, Cloud &result)
  {
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      PointType point = cloud->points[i];
      if (!(fabs(point.x) < 0.01 &&
            fabs(point.y) < 0.01 &&
            fabs(point.z) < 0.01) &&
          !pcl_isnan(point.x) &&
          !pcl_isnan(point.y) &&
          !pcl_isnan(point.z))
        result.points.push_back(point);
    }

    result.width = result.points.size ();
    result.height = 1;
    result.is_dense = true;
  }

  void
  normalEstimation (const CloudConstPtr &cloud,
                           pcl::PointCloud<pcl::Normal> &result)
    {
      FPS_CALC_BEGIN;
      ne_.setInputCloud (cloud);
      ne_.compute (result);
      FPS_CALC_END("normalEstimation");
    }

  void
  addNormalsToCloud (const CloudConstPtr &cloud,
                         const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
                         RefCloud &result)
  {
    result.width = cloud->width;
    result.height = cloud->height;
    result.is_dense = cloud->is_dense;
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      RefPointType point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;

      //point.rgb = cloud->points[i].rgb;

      /*
      point.normal[0] = normals->points[i].normal[0];
      point.normal[1] = normals->points[i].normal[1];
      point.normal[2] = normals->points[i].normal[2];
      */
      result.points.push_back (point);
    }
  }

  void
  removeNormalsFromCloud (const RefCloudConstPtr &cloud, Cloud &result)
  {
    result.width = cloud->width;
    result.height = cloud->height;
    result.is_dense = cloud->is_dense;
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      PointType point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;

      //point.rgb = cloud->points[i].rgb;

      result.points.push_back (point);
    }
  }

  // stuff to segment cloud and select one cluster
  void
  euclideanSegment (const CloudConstPtr &cloud,
                         std::vector<pcl::PointIndices> &cluster_indices)
  {
    FPS_CALC_BEGIN;
    pcl::EuclideanClusterExtraction<PointType> ec;
    KdTreePtr tree (new KdTree ());

    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    //ec.setMaxClusterSize (400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    FPS_CALC_END("euclideanSegmentation");
  }

  void
  extractSegmentCluster (const CloudConstPtr &cloud,
                              const std::vector<pcl::PointIndices> cluster_indices,
                              const int segment_index,
                              Cloud &result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
    {
      PointType point = cloud->points[segmented_indices.indices[i]];
      result.points.push_back (point);
    }
    result.width = result.points.size ();
    result.height = 1;
    result.is_dense = true;
  }

  void
  selectReferenceCluster(const CloudConstPtr &cloud, Cloud &result)
  {
    Eigen::Vector4f c;
    std::vector<pcl::PointIndices> cluster_indices;
    CloudPtr temp_cloud (new Cloud);

    euclideanSegment (cloud, cluster_indices);
    if (cluster_indices.size () > 0)
    {
      // select the cluster to track
      extractSegmentCluster (cloud, cluster_indices, 0, *temp_cloud);
      pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
      int segment_index = 0;
      double segment_distance = c[0] * c[0] + c[1] * c[1];
      for (size_t i = 1; i < cluster_indices.size (); i++)
      {
        temp_cloud.reset (new Cloud);
        extractSegmentCluster (cloud, cluster_indices, i, *temp_cloud);
        pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
        double distance = c[0] * c[0] + c[1] * c[1];
        if (distance < segment_distance)
        {
          segment_index = i;
          segment_distance = distance;
        }
      }
      extractSegmentCluster (cloud, cluster_indices, segment_index, result);
    }
    else
    {
      PCL_WARN ("euclidean segmentation failed (Cluster extraction)\n");
    }
  }

  // set the template aka reference cloud
  void
  setReferenceCloud(const CloudConstPtr &cloud)
  {
    Eigen::Vector4f centroid;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
    CloudPtr nonzero_ref (new Cloud);
    RefCloudPtr final_cloud (new RefCloud);

    // take garbage out
    removeZeroPoints (cloud, *nonzero_ref);
    tracker_->setMinIndices (nonzero_ref->points.size () / 2);

    // compute centroid
    pcl::compute3DCentroid<PointType> (*nonzero_ref, centroid);
    transform.translation () = Eigen::Vector3f ((double)centroid[0], (double)centroid[1], (double)centroid[2]);

    // compute and add normals
    ref_normals_.reset (new pcl::PointCloud<pcl::Normal>);
    normalEstimation (nonzero_ref, *ref_normals_);

    //addNormalsToCloud (nonzero_ref, ref_normals_, *normal_cloud);
    //pcl::transformPointCloudWithNormals<RefPointType> (*normal_cloud, *final_cloud, transform.inverse());
    pcl::transformPointCloud<RefPointType> (*nonzero_ref, *final_cloud, transform.inverse());

    tracker_->setReferenceCloud (final_cloud);
    tracker_->setTrans (transform);

    //removeNormalsFromCloud(final_cloud, *reference_);
    reference_.reset(new Cloud(*final_cloud));
  }

  void
  trackCloud (const RefCloudConstPtr &cloud)
  {
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
    double end = pcl::getTime ();
    FPS_CALC_END("tracking");
    tracking_time_ = end - start;
  }

  double
  estimateGridSize(const CloudConstPtr &cloud)
  {
    int ndigits=1, first_digit;
    double npoints = cloud->points.size ();
    //PCL_WARN("points> %f\n", npoints);
    while (npoints >= 10)
    {
       npoints /= 10;
       ndigits++;
       first_digit = static_cast<int> (npoints) % 10;
    }
    //PCL_WARN("ndigits> %d\n", ndigits);
    //PCL_WARN("firstdigit> %d\n", first_digit);
    return 1/(pow(10,(7-ndigits)));
  }

  void alignmentSAC(const CloudConstPtr &target_cloud, Cloud &template_cloud)
  {
    FeatureCloud targt_frame, templ_cloud;
    TemplateAlignment template_alignment;
    TemplateAlignment::Result alignment_result;

    pcl::PointCloud<pcl::PointXYZ> xyz_target, xyz_template;
    pcl::copyPointCloud(*target_cloud, xyz_target);
    pcl::copyPointCloud(template_cloud, xyz_template);

    targt_frame.setInputCloud (xyz_target.makeShared());
    templ_cloud.setInputCloud(xyz_template.makeShared());

    template_alignment.setTargetCloud (targt_frame);
    //template_align.addTemplateCloud(templ_cloud);

    template_alignment.align(templ_cloud, alignment_result);
    PCL_INFO ("SAC-IA fitness score: %f\n", alignment_result.fitness_score);

    pcl::transformPointCloud (*templ_cloud.getPointCloud(), xyz_template, alignment_result.final_transformation);

    pcl::copyPointCloud(xyz_template, template_cloud);
  }

  void
  cloud_cb (const CloudConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;

    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);

    filterPassThrough (cloud, *cloud_pass_, pass_min_, pass_max_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

    if (counter_ == 10) // run only once after 10 frames
    {
      CloudPtr source_cloud (new Cloud);
      CloudPtr reference_cloud (new Cloud);
      CloudPtr temp_cloud (new Cloud);
      double ref_downsampl_grid_size;

      if( template_name_.empty() ) // sample from closest cluster
      {
        PCL_INFO("-->>> Performing segmentation and selecting target cluster\n");
        if(use_convex_hull_)
        {
          PCL_INFO("-->>> Removing plane segments first\n");
          removePlaneSegments(cloud_pass_downsampled_, *source_cloud);
        }
        else
        {
          // Approx Grid Sample is not accurate enough (but faster for cloud)
          gridSample (cloud_pass_, *source_cloud, downsampling_grid_size_);
        }
        selectReferenceCluster (source_cloud, *temp_cloud);
      }
      else // load and use predefined .pcd file
      {
        try
        {
          pcl::PointCloud<pcl::PointXYZ> xyz_temp;
          pcl::io::loadPCDFile<pcl::PointXYZ> (template_name_, xyz_temp);
          pcl::copyPointCloud(xyz_temp, *temp_cloud);
        }
        catch (std::exception& e)
        {
          PCL_ERROR (e.what ());
        }
        PCL_INFO("-->>> Point cloud loaded from file\n");

        // capture one frame from camera for initial alignment
        //alignmentSAC(cloud_pass_downsampled_, *temp_cloud);
      }

      // downsample and set cloud
      ref_downsampl_grid_size = estimateGridSize(temp_cloud);
      gridSample (temp_cloud, *reference_cloud, ref_downsampl_grid_size);

      setReferenceCloud (reference_cloud);
      reference_.reset (new Cloud(*temp_cloud));
      PCL_INFO("-->>> Target point cloud is set!\n");

      counter_ = 0;
      return;
    }

    if (counter_ > 10) // start tracking
    {
      //normals_.reset (new pcl::PointCloud<pcl::Normal>);
      //normalEstimation (cloud_pass_downsampled_, *normals_);
      //RefCloudPtr tracking_cloud (new RefCloud ());
      //addNormalsToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);

      trackCloud (cloud_pass_downsampled_);
    }
    
    new_cloud_ = true;
    double end = pcl::getTime ();
    computation_time_ = end - start;
    FPS_CALC_END("computation");
    counter_++;
  }
      
  void
  run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
    boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIFaceTracking::cloud_cb, this, _1);
    interface->registerCallback (f);
    
    viewer_.runOnVisualizationThread (boost::bind(&OpenNIFaceTracking::viz_cb, this, _1), "viz_cb");
    interface->start ();
      
    while (!viewer_.wasStopped ())
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    interface->stop ();
  }
  
  pcl::visualization::CloudViewer viewer_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::PointCloud<pcl::Normal>::Ptr ref_normals_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  RefCloudPtr reference_;

  boost::mutex mtx_;
  boost::shared_ptr<ParticleFilter> tracker_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool

  std::string device_id_, template_name_;
  bool new_cloud_, use_convex_hull_, visualize_non_downsample_, visualize_particles_;
  int counter_;
  double tracking_time_, computation_time_, downsampling_time_;
  double downsampling_grid_size_, pass_min_, pass_max_;
  };

void
usage (char** argv)
{
  std::cout << " Usage: " << argv[0] << " <device_id> [options] \n\n";
  std::cout << "\t -C: initialize the pointcloud to track without plane segmentation."
            << std::endl;
  std::cout << "\t -D: visualize with non-downsampled pointclouds."
            << std::endl;
  std::cout << "\t -P: do not show particle cloud."
            << std::endl;
  std::cout << "\t -fixed: use fixed number of particles for tracker."
            << std::endl;
  std::cout << "\t -d <value>: specify the grid size of downsampling (defaults to 0.01)."
            << std::endl;
  std::cout << "\t -pass <min, max>: specify the min (0.2) and max (5.5) distance in m for the passThrough filter."
            << std::endl;

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    std::cout << "\n device_id may be #1, #2, ... for the first second etc device in the list or" << endl
               << "bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
               << "<serial-number> (only in Linux and for devices which provide serial numbers)" << endl;
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      std::cout << "\n  -> Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
           << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
    }
  }
  else
    std::cout << "No devices connected." << endl;
}

int
main (int argc, char** argv)
{
  if (pcl::console::find_argument (argc, argv, "-h") != -1 ||
      pcl::console::find_argument (argc, argv, "--help") != -1 ||
      argc == 1)
  {
    // TODO: change prefix of # parameter for camera since it's not counted in argc
    usage (argv);
    exit (1);
  }

  bool use_convex_hull = true;
  bool visualize_non_downsample = false;
  bool visualize_particles = true;
  bool use_fixed = false;
  double downsampling_grid_size = 0.01, pass_min = 0.25, pass_max = 5.5;
  std::string template_name("");

  if (pcl::console::find_argument (argc, argv, "-C") > 0)
    use_convex_hull = false;
  if (pcl::console::find_argument (argc, argv, "-D") > 0)
    visualize_non_downsample = true;
  if (pcl::console::find_argument (argc, argv, "-P") > 0)
    visualize_particles = false;
  if (pcl::console::find_argument (argc, argv, "-fixed") > 0)
    use_fixed = true;

  pcl::console::parse_2x_arguments (argc, argv, "-pass", pass_min, pass_max, false);

  pcl::console::parse_argument (argc, argv, "-d", downsampling_grid_size);

  std::vector<int> index = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (!index.empty())
    template_name = std::string (argv[index[0]]);

  std::string device_id (argv[1] != NULL ? argv[1]:"");

  // open kinect
  OpenNIFaceTracking<pcl::PointXYZRGBA> v (device_id, rand() % 10,
                                       downsampling_grid_size,
                                       pass_min, pass_max,
                                       use_convex_hull, visualize_non_downsample,
                                       visualize_particles, use_fixed,
                                       template_name);
  v.run ();
}
