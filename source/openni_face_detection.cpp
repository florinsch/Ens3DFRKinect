/*
 * openni_face_detection.cpp
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 *
 *  Date 12 Feb 2013
 *  Added normalization, recording:
 *      Author: Florin Schimbinschi
 */

#include "pcl/recognition/face_detection/rf_face_detector_trainer.h"
//#include "pcl/apps/face_detection/openni_frame_source.h"
//#include "pcl/apps/face_detection/face_detection_apps_utils.h"
#include "openni_frame_source.h"
#include "face_detection_apps_utils.h"

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/mls.h>


// pcl::PointWithScale : 3-D position and scale.
// pcl::PointWithViewpoint : Euclidean xyz coordinates + viewpoint
// pcl::PointWithRange : Euclidean xyz coordinates + range float
typedef OpenNIFrameSource::PointT PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

static double fps = 0;
#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime();\
    double now = pcl::getTime(); \
    ++count; \
    if(now - last >= 1.0) \
    { \
      fps = double(count)/double(now - last); \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

using namespace pcl;
using std::cout;

std::string make_name(int counter, const char* suffix, const char* dir)
{
  char buf[4096];
  sprintf(buf, "%s/frame%04d_%s.pcd", dir, counter, suffix);
  return buf;
}

template <typename PointSave> void
savePCDFile(const std::string& filename, const pcl::PointCloud<PointSave>& cloud)
{
  pcl::io::savePCDFileASCII(filename, cloud);
  cout << "Saved: " << filename <<"\n";
}

double
computeCloudResolution (const CloudConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void
run(pcl::RFFaceDetectorTrainer &fdrf, bool heat_map, bool show_votes,
    std::string mask_path, std::string save_dir = ".")
{
  OpenNIFrameSource::OpenNIFrameSource camera;
  OpenNIFrameSource::PointCloudPtr scene_vis;

  // visualizer options
  pcl::visualization::PCLVisualizer vis("Face Detection & Recorder");
  vis.setSize(1360, 768);
  int v1(0), v2(0);
  vis.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  vis.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  vis.setBackgroundColor(0.5, 0.5, 0.5, v1);
  vis.setBackgroundColor(0, 255, 0, v2);
  vis.addCoordinateSystem(0.20, 0.0, 0.0, 0.0, v2);
//  vis.setFullScreen(true);
//  vis.createViewPortCamera(v2);
//  vis.setShowFPS(true);
  vis.initCameraParameters();
  //keyboard callback to stop getting frames and finalize application
  boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb = boost::bind
                       (&OpenNIFrameSource::OpenNIFrameSource::onKeyboardEvent, &camera, _1);
  vis.registerKeyboardCallback(keyboard_cb);

  // ------------------- parameter editing here -------------------
  pcl::CropBox<PointType> cbx;

  pcl::UniformSampling<PointType> us;
  us.setRadiusSearch (0.003);

  pcl::EuclideanClusterExtraction<PointType> xclstrs;
//  xclstrs.setClusterTolerance(0.003); // 1cm = 0.01
  xclstrs.setMinClusterSize(1000);
  xclstrs.setMaxClusterSize(20000);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(0.1); // max dist (src<->tgt)
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(0.0001);

  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
  ne.setRadiusSearch (0.03);
  ne.setViewPoint(15,15,-215);

  PointType minPt, maxPt;
  CloudPtr face(new Cloud);
  CloudPtr mask(new Cloud);
  // ------------------- end declarations & config -------------------

  //load generic face template (mask) as ICP target
  if(pcl::io::loadPCDFile<PointType>(mask_path, *mask) == -1)
  {
    PCL_ERROR("Couldn't read mask file! \n");
    exit(1);
  }
  else
  {
    // position mask at origin
    pcl::getMinMax3D(*mask, minPt, maxPt);
    pcl::demeanPointCloud(*mask, Eigen::Vector4f(minPt.x, minPt.y, minPt.z, 1), *mask);

    // uniform sampling for faster ICP
    us.setInputCloud (mask);
    pcl::PointCloud<int> sampled_indices;
    us.compute (sampled_indices);
    pcl::copyPointCloud (*mask, sampled_indices.points, *mask);

    // set target for icp
    icp.setInputTarget(mask);

    // compute normals
//    ne.setInputCloud (mask);
//    pcl::PointCloud<pcl::Normal>::Ptr mask_normals (new pcl::PointCloud<pcl::Normal>);
//    ne.compute (*mask_normals);
  }

  // main loop
  while(camera.isActive())
  {   
    FPS_CALC("FPS");
    scene_vis = camera.snap();

    // xyz cloud for face detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene_vis, *scene);

    // detect faces
    fdrf.setInputCloud(scene);
//    pcl::ScopeTime t1("Detect faces: ");
    fdrf.detectFaces();
    std::vector<Eigen::VectorXf> heads;
    fdrf.getDetectedFaces(heads);
    
    // show point cloud
    pcl::visualization::PointCloudColorHandlerRGBField<OpenNIFrameSource::PointT> handler_keypoints(scene_vis);
    vis.addPointCloud<OpenNIFrameSource::PointT>(scene_vis, handler_keypoints, "scene_cloud", v1);
    // show mask
    if(camera.isShowingMask())
    {
      pcl::visualization::PointCloudColorHandlerCustom <PointType> handler_votes(mask, 125, 255, 0);
      vis.addPointCloud <PointType>(mask, handler_votes, "mask", v2);
      vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "mask");
    }

    // iterate through detected heads
    for(size_t i = 0; i < heads.size(); ++i)
    {
      std::stringstream name;
      name << "face_" << i;

      // crop around predicted centroid from face detector
      float edge = 0.1;
      cbx.setMin(Eigen::Vector4f(heads[i][0]-edge*0.65, heads[i][1]-edge*0.9, heads[i][2]-edge, 1));
      cbx.setMax(Eigen::Vector4f(heads[i][0]+edge*0.65, heads[i][1]+edge, heads[i][2]+edge*0.15, 1));
      cbx.setInputCloud(scene_vis);
      cbx.filter(*face);

      // compute cloud resolution
      double resolution = computeCloudResolution(face);

      // pre-normalize pose according to head pose detector
      Eigen::Vector3f vec_y = Eigen::Vector3f::UnitY();
      Eigen::Vector3f vec_z = Eigen::Vector3f::UnitZ() * 1.f;
      Eigen::Matrix3f matrix;
      matrix = Eigen::AngleAxisf(heads[i][3], Eigen::Vector3f::UnitX())
             * Eigen::AngleAxisf(heads[i][4], Eigen::Vector3f::UnitY())
             * Eigen::AngleAxisf(heads[i][5], Eigen::Vector3f::UnitZ());
      vec_z = matrix * vec_z;
      vec_y = matrix * vec_y;
      pcl::transformPointCloud<PointType>(*face, *face, pcl::getTransformationFromTwoUnitVectors(vec_y, vec_z));

      // normalize position (move next to 0, 0 ,0)
      pcl::getMinMax3D(*face, minPt, maxPt);
      if(minPt.z < 0.4 || minPt.z > 1.3) continue; // drop false positive too close/far
      pcl::demeanPointCloud(*face, Eigen::Vector4f(minPt.x, minPt.y, minPt.z, 1), *face);

      // pre-filter out false positives
      pcl::IterativeClosestPoint<PointType, PointType> icp_pre;
      icp_pre.setInputSource(face);
      icp_pre.setInputTarget(mask);
//      std::cout << "Head #" << i
//                << " distance: " << minPt.z
//                << " fitness: " << icp_pre.getFitnessScore() << "\n";
      if(icp_pre.getFitnessScore() > 0.00035) continue; // terminate for non-faces

      // create new search tree
      pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

      // perform euclidean clustering (remove neck, artifacts)
      xclstrs.setClusterTolerance(resolution * 2.2);
      xclstrs.setInputCloud(face);
      xclstrs.setSearchMethod(tree);
      std::vector<pcl::PointIndices> clstrs;
      xclstrs.extract(clstrs);
      pcl::copyPointCloud(*face, clstrs, *face);

      // uniform sampling
      us.setInputCloud (face);
      us.setSearchMethod(tree);
      pcl::PointCloud<int> sampled_indices;
      us.compute (sampled_indices);
      CloudPtr face_down (new Cloud);
      pcl::copyPointCloud (*face, sampled_indices.points, *face_down);

      // moving least squares
//      pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;
//      mls.setInputCloud (face);
//      mls.setPolynomialFit (true);
//      mls.setSearchMethod (tree);
//      mls.setSearchRadius (0.02);
//      pcl::PointCloud<pcl::PointNormal> mls_points;
//      mls.process (mls_points);
//      CloudPtr mlscloud (new Cloud);
//      pcl::copyPointCloud(mls_points, *mlscloud);

      // compute normals
//      ne.setInputCloud (face_down);
//      pcl::PointCloud<pcl::Normal>::Ptr face_normals (new pcl::PointCloud<pcl::Normal>);
//      ne.compute (*face_normals);

      // ICP alignment (pose normalization)
      if(camera.isEnabledICP())
      {
        icp.setInputSource(face_down);
        // try to start from last position
//        if(icp.getFitnessScore() < 0.0003)
//          pcl::transformPointCloud<PointType>(*face_down, *face_down, icp.getFinalTransformation());
        icp.align(*face_down);

        if(icp.hasConverged())
        {
          pcl::transformPointCloud<PointType>(*face, *face, icp.getFinalTransformation());
          std::cout << "ICP Final Fitness: " << icp.getFitnessScore() <<"\n";
        }
      }

      // display extracted cloud
      pcl::visualization::PointCloudColorHandlerRGBField<PointType> handler_face(face);
      vis.addPointCloud <PointType>(face, handler_face, name.str().c_str(), v2);

//      name << "_down";
//      pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_faced(face_down, 255, 0, 0);
//      vis.addPointCloud <PointType>(face_down, handler_faced, name.str().c_str(), v2);

//      vis.addPointCloudNormals<PointType, pcl::Normal> (face_down, cloud_normals, 5, 0.05, "normals", v2);

      // show some info
      vis.removeAllShapes(v2);
      std::stringstream infoss;
      infoss << "Framerate: " << fps << " Hz\n"
             << "X: " << minPt.x << " Y: " << minPt.y << "\n"
             << "Distance to sensor: " << minPt.z << "\n"
             << "No. Points: " << face->points.size() << "\n"
             << "Cloud Resolution: " << resolution << "\n";
      vis.addText(infoss.str(), 10, 700, "info", v2);

      // toggle camera tracking head according to viewpoint
      if(camera.isTrackingCam() || camera.getNoFrames() == 20)
        vis.setCameraPosition(heads[i][0], heads[i][1]+0.1, heads[i][2]-1.2, 0, 0.1, 0, 0, -1, 0, v1);

      // toggle recording
      if(camera.isRecording())
      {
        std::string filename = make_name(camera.getNoFrames(), name.str().c_str(), save_dir.c_str());
        if(face->points.size())
            savePCDFile(filename, *face);
        filename.append(" was saved to disk");
        vis.addText(filename.c_str(), 10, 10, "recording", v2);
      }
    }
    
    face_detection_apps_utils::displayHeads(heads, vis, v1);
    
    //vis.setRepresentationToSurfaceForAllActors();
    vis.setRepresentationToWireframeForAllActors();
       
    vis.spinOnce();
    vis.removeAllPointClouds();
    vis.removeAllShapes();
  }
}

//./bin/pcl_openni_face_detector -face_threshold 0.99 -max_variance 2400 -min_votes_size 300 -stride_sw 4 -heat_map 0 -show_votes 1 -pose_refinement 1 -icp_iterations 5 -model_path face_model.pcd -forest_fn forest.txt
int
main(int argc, char ** argv)
{
  int STRIDE_SW = 4;
  int use_normals = 0;
  float trans_max_variance = 1600.f;
  int min_votes_size = 100;
  float face_threshold = 0.99f;
  int heat_map = 0;
  int show_votes = 0;
  int pose_refinement = 0;
  int icp_iterations = 5;
  int show_mask = 1;

  std::string forest_fn = "../source/forest_example.txt";
  std::string model_path = "../models/head.pcd";
  std::string mask_path = "../models/mask.pcd";
  std::string save_dir = ".";

  pcl::console::parse_argument(argc, argv, "-forest_fn", forest_fn);
  pcl::console::parse_argument(argc, argv, "-max_variance", trans_max_variance);
  pcl::console::parse_argument(argc, argv, "-min_votes_size", min_votes_size);
  pcl::console::parse_argument(argc, argv, "-use_normals", use_normals);
  pcl::console::parse_argument(argc, argv, "-face_threshold", face_threshold);
  pcl::console::parse_argument(argc, argv, "-stride_sw", STRIDE_SW);
  pcl::console::parse_argument(argc, argv, "-heat_map", heat_map);
  pcl::console::parse_argument(argc, argv, "-show_votes", show_votes);
  pcl::console::parse_argument(argc, argv, "-pose_refinement", pose_refinement);
  pcl::console::parse_argument(argc, argv, "-model_path", model_path);
  pcl::console::parse_argument(argc, argv, "-mask_path", mask_path);
  pcl::console::parse_argument(argc, argv, "-icp_iterations", icp_iterations);
  pcl::console::parse_argument(argc, argv, "-save_dir", save_dir);
  pcl::console::parse_argument(argc, argv, "-s", save_dir);

  // create decision forest
  typedef pcl::face_detection::RFTreeNode<pcl::face_detection::FeatureType> NodeType;
  pcl::DecisionForest<NodeType> forest;

  // create face detector
  pcl::RFFaceDetectorTrainer fdrf;
  fdrf.setForestFilename(forest_fn);
  fdrf.setWSize(80);
  fdrf.setUseNormals(static_cast<bool>(use_normals));
  fdrf.setWStride(STRIDE_SW);
  fdrf.setLeavesFaceMaxVariance(trans_max_variance);
  fdrf.setLeavesFaceThreshold(face_threshold);
  fdrf.setFaceMinVotes(min_votes_size);
  if(pose_refinement)
  {
    fdrf.setPoseRefinement(true, icp_iterations);
    fdrf.setModelPath(model_path);
  }

  // load forest from file and pass it to the detector
  std::filebuf fb;
  fb.open(forest_fn.c_str(), std::ios::in);
  std::istream os(&fb);
  forest.deserialize(os);
  fdrf.setForest(forest);
  fb.close();

  run(fdrf, heat_map, show_votes, mask_path, save_dir);
}
