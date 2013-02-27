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
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>

//#include </tmp/pcl-trunk/gpu/features/include/pcl/gpu/features/features.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

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

typedef pcl::PointXYZRGBA ColorType;
typedef pcl::PointCloud<ColorType> ColorCloud;
typedef typename ColorCloud::Ptr ColorCloudPtr;

typedef pcl::PointNormal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef typename NormalCloud::Ptr NormalCloudPtr;

typedef pcl::FPFHSignature33 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;
typedef typename DescriptorCloud::Ptr DescriptorCloudPtr;


std::string
make_name(int counter, const char* suffix, const char* dir)
{
  char buf[4096];
  sprintf(buf, "%s/face%04d_%s.pcd", dir, counter, suffix);
  return buf;
}

template <typename PointT> void
savePCDFile(const std::string& filename, const pcl::PointCloud<PointT>& cloud)
{
  pcl::io::savePCDFileASCII(filename, cloud);
  cout << "Saved: " << filename <<"\n";
}

double
computeCloudResolution (const pcl::PointCloud<ColorType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<ColorType> tree;
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

  // histogram visualizer
  pcl::visualization::PCLHistogramVisualizer histvis;

  // visualizer options
  pcl::visualization::PCLVisualizer vis("Face Detection & Recorder");
  vis.setSize(1360, 768);
  int v1(0), v2(0);
  vis.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  vis.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  vis.setBackgroundColor(0.5, 0.5, 0.5, v1);
  vis.setBackgroundColor(0, 255, 0, v2);
//  vis.addCoordinateSystem(0.25, 0.0, 0.0, 0.0, v2);
//  vis.setShowFPS(true); // not working
  vis.initCameraParameters();

//  pcl::search::KdTree<PointType> tree;
//  pcl::KdTreeFLANN<PointType> tree;

  pcl::NormalEstimationOMP<ColorType, NormalType> ne;
//  ne.setRadiusSearch (0.03); // meters
  ne.setKSearch(20);

  pcl::FPFHEstimationOMP<ColorType, NormalType, DescriptorType> fpfh;
//  fpfh.setRadiusSearch (0.05); // !!! > normals radius
  fpfh.setKSearch(25);

  pcl::CropBox<ColorType> cbx;
  pcl::ApproximateVoxelGrid<ColorType> vgrid;

  pcl::EuclideanClusterExtraction<ColorType> xclstrs;
  xclstrs.setClusterTolerance(0.0049); // !!! > downsampling leaf
  xclstrs.setMinClusterSize(400);
  xclstrs.setMaxClusterSize(30000); // TODO: CHANGE ME TO 3K IF DONWSAMPLING

  //keyboard callback to stop getting frames and finalize application
  boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb = boost::bind
                       (&OpenNIFrameSource::OpenNIFrameSource::onKeyboardEvent, &camera, _1);
  vis.registerKeyboardCallback(keyboard_cb);

  // load face template - mask
  ColorCloudPtr mask(new ColorCloud);
  ColorCloudPtr mask_keypoints (new ColorCloud);
  if(pcl::io::loadPCDFile<ColorType>(mask_path, *mask) == -1)
  {
    PCL_ERROR("Couldn't read mask file! \n");
    exit(1);
  }
  else
  {
    // compute normals
    pcl::search::KdTree<ColorType>::Ptr tree_ne (new pcl::search::KdTree<ColorType> ());
    ne.setSearchMethod (tree_ne);
    ne.setInputCloud (mask);
    NormalCloudPtr mask_normals (new NormalCloud);
    ne.compute (*mask_normals);

    // uniform sampling
    pcl::UniformSampling<ColorType> us;
    us.setInputCloud (mask);
    us.setRadiusSearch (0.005);
    pcl::PointCloud<int> sampled_indices;
    us.compute (sampled_indices);
    pcl::copyPointCloud (*mask, sampled_indices.points, *mask_keypoints);

    // downsample mask
//    double leaf = 0.01;
//    vgrid.setLeafSize(leaf, leaf, leaf);
//    vgrid.setInputCloud(mask);
//    ColorCloudPtr mask_down (new ColorCloud);
//    vgrid.filter(*mask);

    // compute fpfh histogram
    pcl::search::KdTree<ColorType>::Ptr tree_hist (new pcl::search::KdTree<ColorType> ());
    fpfh.setSearchMethod (tree_hist);
    fpfh.setInputCloud (mask);
    fpfh.setInputNormals (mask_normals);
    //fpfh.setSearchSurface(mask_keypoints);
    DescriptorCloudPtr hist_mask (new DescriptorCloud);
    fpfh.compute (*hist_mask);
//    histvis.addFeatureHistogram (*hist_mask, mask->size(), "mask");
  }

  // main loop
  while(camera.isActive())
  {   
    FPS_CALC("FPS");

    pcl::visualization::PointCloudColorHandlerCustom <ColorType> handler_votes(mask_keypoints, 125, 255, 0);
    vis.addPointCloud <ColorType>(mask_keypoints, handler_votes, "mask", v2);
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "mask");

    // show point cloud
    scene_vis = camera.snap();
    pcl::visualization::PointCloudColorHandlerRGBField<OpenNIFrameSource::PointT> handler_keypoints(scene_vis);
    vis.addPointCloud < OpenNIFrameSource::PointT >(scene_vis, handler_keypoints, "scene_cloud", v1);

    // get xyz cloud for face detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene_vis, *scene_xyz);

    // detect faces & store
    fdrf.setInputCloud(scene_xyz);
    //pcl::ScopeTime t1("Detect faces: ");
    fdrf.detectFaces();
    std::vector<Eigen::VectorXf> heads;
    fdrf.getDetectedFaces(heads);

    // extract & show cropped face / toggle writing to disk
    std::stringstream name;
    for(size_t i = 0; i < heads.size(); ++i)
    {
        name << "no_" << i;
        
        // crop around predicted centroid from face detector
        ColorCloudPtr face(new ColorCloud);
        float edge = 0.1;
        cbx.setMin(Eigen::Vector4f(heads[i][0]-edge*0.65, heads[i][1]-edge*0.9, heads[i][2]-edge, 1));
        cbx.setMax(Eigen::Vector4f(heads[i][0]+edge*0.65, heads[i][1]+edge, heads[i][2]+edge*0.15, 1));
        cbx.setInputCloud(scene_vis);
        cbx.filter(*face);
        double resolution = computeCloudResolution(face);

        // normalize pose according to face detector
        Eigen::Vector3f vec_y = Eigen::Vector3f::UnitY();
        Eigen::Vector3f vec_z = Eigen::Vector3f::UnitZ() * 1.f;
        Eigen::Matrix3f matrix;
        matrix = Eigen::AngleAxisf(heads[i][3], Eigen::Vector3f::UnitX())
               * Eigen::AngleAxisf(heads[i][4], Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(heads[i][5], Eigen::Vector3f::UnitZ());
        vec_z = matrix * vec_z;
        vec_y = matrix * vec_y;
        //pcl::ScopeTime t2("Transform: ");
        pcl::transformPointCloud<ColorType>(*face, *face, pcl::getTransformationFromTwoUnitVectors(vec_y, vec_z));

        // downsample
        vgrid.setInputCloud(face);
        //pcl::ScopeTime t3("Downsampling: ");
        //vgrid.filter(*face);
        
        // TODO: compute normals and histogram with fixed template

        // perform euclidean clustering on cube(remove neck, etc)
        xclstrs.setInputCloud(face);
        //pcl::ScopeTime t4("Extract clusters: ");
        std::vector<pcl::PointIndices> clstrs;
        xclstrs.extract(clstrs);
        pcl::copyPointCloud(*face, clstrs, *face);

        // compute normals
        pcl::search::KdTree<ColorType>::Ptr tree_ne (new pcl::search::KdTree<ColorType> ());
        ne.setSearchMethod (tree_ne);
        ne.setInputCloud (face);
        NormalCloudPtr face_normals (new NormalCloud);
        pcl::ScopeTime t5("Computing normals: ");
        ne.compute (*face_normals);

        // compute fpfh histogram
        pcl::search::KdTree<ColorType>::Ptr tree_hist (new pcl::search::KdTree<ColorType> ());
        fpfh.setSearchMethod (tree_hist);

        fpfh.setInputCloud (face);
        fpfh.setInputNormals (face_normals);
        DescriptorCloudPtr hist_face (new DescriptorCloud);
        //pcl::ScopeTime t6("Extracting features: ");
        fpfh.compute (*hist_face);

        cout << *hist_face; int size = face->size();
        histvis.addFeatureHistogram (*hist_face, size, name.str());
        histvis.updateFeatureHistogram(*hist_face, size, name.str());
        histvis.spinOnce();

        // normalize position(move to 0, 0 ,0)
        ColorType minPt, maxPt;
        pcl::getMinMax3D(*face, minPt, maxPt);
        if(minPt.z < 0.3) continue; // drop false positive too close to the camera
        pcl::demeanPointCloud(*face, Eigen::Vector4f(minPt.x, minPt.y, minPt.z, 1), *face);

        // display extracted cloud
        std::stringstream normalname;
        normalname << name.str() << "_normals"; name << "_face";
        pcl::visualization::PointCloudColorHandlerRGBField<ColorType> handler_face(face);
        vis.addPointCloud <ColorType>(face, handler_face, name.str().c_str(), v2);
        vis.addPointCloudNormals<ColorType, NormalType> (face, face_normals, 5, 0.03, normalname.str().c_str(), v2);

        // show some info        
        vis.removeAllShapes();
        std::stringstream infoss;
        infoss << "Framerate: " << fps << " Hz\n"
               << "X: " << minPt.x << " Y: " << minPt.y << "\n"
               << "Distance to sensor: " << minPt.z << "\n"
               << "Cloud Resolution: " << resolution << "\n"
               << "No. Points: " << face->points.size() << "\n";
        vis.addText(infoss.str(), 10, 700, "info", v2);
        
        // fullscreen freezes keyboard + mouse
        //vis.setFullScreen(camera.isFullscreen());

        // toggle camera tracking and position change
        if(camera.isTrackingCam() || camera.getNoFrames() == 20)
            vis.setCameraPosition(heads[i][0], heads[i][1]+0.1, heads[i][2]-1.2, 0, 0.1, 0, 0, -1, 0, v1);
        
        // toggle recording
        if(camera.isRecording())
        {
            std::string filenameface = make_name(camera.getNoFrames(), name.str().c_str(), save_dir.c_str());
            std::string filenamenormals = make_name(camera.getNoFrames(), normalname.str().c_str(), save_dir.c_str());
            if(face->points.size() > 0)
            {
              savePCDFile(filenameface, *face);
              savePCDFile(filenamenormals, *face_normals);
            }
            filenameface.append(" was saved to disk");
            vis.addText(filenameface.c_str(), 10, 10, "recording", v2);
        }
        name.str(std::string());
    }
    
    face_detection_apps_utils::displayHeads(heads, vis, v1);
    
    //histvis.spinOnce();

    //vis.setRepresentationToSurfaceForAllActors();
    vis.setRepresentationToWireframeForAllActors();
    vis.spinOnce();
    vis.removeAllPointClouds();
    vis.removeAllShapes();
  }
}

//./bin/pcl_openni_face_detector -face_threshold 0.99 -max_variance 2400 -min_votes_size 300 -stride_sw 4 -heat_map 0 -show_votes 1 -pose_refinement 1 -icp_iterations 5 -model_path face_model.pcd -forest_fn forest.txt
int main(int argc, char ** argv)
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
  std::string model_path = "../models/model.pcd";
  std::string mask_path = "../models/newmask.pcd";
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
