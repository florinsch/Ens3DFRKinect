#include "boost/filesystem.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/esf.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/pfh.h>

#include <pcl/filters/filter.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointNormal NormalType;

using namespace pcl;
using std::cout;
namespace fs = boost::filesystem;

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

template <typename PointSave> void
savePCDFile(const std::string& filename, const pcl::PointCloud<PointSave>& cloud)
{
  pcl::io::savePCDFileASCII(filename, cloud);
  cout << "Saved: " << filename <<"\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbaVis()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  return(viewer);
}

void
printUsage(const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options]\n\n"
      << " This program will generate .esf .pfh and .shot"
      << "files in 'dir' with features from the .pcd files\n"
      << "-------------------------------------------\n"
      << "Options:\n"
      << "-------------------------------------------\n"
      << "-h this help\n"
      << "-shot radius search in meters (default: 0.03) \n"
      << "-pfh radius search in meters (default: 0.01) \n"
      << "-dir folder to start recursive iteration (default: current)\n"
      << "-label for all the features (class name) (default: pwd) \n"
      << "\n\n";
}

//find . -mindepth 2 -type d -exec sh -c '../../build/keypoint_features -dir {} -mask_path ../../models/mask.pcd' \;

int
main(int argc, char ** argv)
{
  // parse arguments
  if(pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }

  // cloud viewer (comment out + stuff at end of file)
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbaVis();
//  int v1(0), v2(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->addCoordinateSystem(0.05);

  // ICP to normalize keypoints
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(0.1); // max dist (src<->tgt)
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(0.0001);

  // uniform sampling
  pcl::UniformSampling<PointType> us;
  us.setRadiusSearch (0.009);

  // load icp target
  CloudPtr mask(new Cloud);
  std::string mask_path = "../models/mask.pcd";
  pcl::console::parse_argument(argc, argv, "-mask_path", mask_path);
  if(pcl::io::loadPCDFile<PointType>(mask_path, *mask) == -1)
  {
    PCL_ERROR("Couldn't read mask file! \n");
    exit(1);
  }
  else
  {
    // position mask at origin
    PointType minPt, maxPt;
    pcl::getMinMax3D(*mask, minPt, maxPt);
    pcl::demeanPointCloud(*mask, Eigen::Vector4f(minPt.x, minPt.y, minPt.z, 1), *mask);

    // uniform sampling for faster ICP
    us.setInputCloud (mask);
    pcl::PointCloud<int> sampled_indices;
    us.compute (sampled_indices);
    pcl::copyPointCloud (*mask, sampled_indices.points, *mask);

    // set target for icp
    icp.setInputTarget(mask);
    // add to visualizer
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> msk(mask, 200, 10, 40);
//    viewer->addPointCloud<PointType>(mask, msk, "Mask", v2);
  }

  // folder names and parsing
  std::string folder = ".";
  pcl::console::parse_argument(argc, argv, "-dir", folder);
  std::string absolute = fs::canonical(fs::path(folder)).string();
  std::string label = fs::path(folder).string();
  std::replace(label.begin(), label.end(), '/', '-');
  label.erase(std::remove(label.begin(), label.end(), '.'), label.end());
  pcl::console::parse_argument(argc, argv, "-label", label);

  float shotSize = 0.03;
  pcl::console::parse_argument(argc, argv, "-shot", shotSize);
  float pfhSize = 0.01;
  pcl::console::parse_argument(argc, argv, "-pfh", pfhSize);
  std::cout << "\nRadius search SHOT:" << shotSize << " , PFH:" << pfhSize << "\n";

  // Open 3 files for writing features
  std::stringstream ss;
  ss << absolute << "/features-ESF-" << label << ".csv";
  std::ofstream esf_writer(ss.str().c_str());
  ss.str(""); ss << absolute << "/features-PFH-" << label << ".csv";
  std::ofstream pfh_writer(ss.str().c_str());
  ss.str(""); ss << absolute << "/features-SHOT-" << label << ".csv";
  std::ofstream shot_writer(ss.str().c_str());
  cout << "\nFiles saved in: " << absolute << "\n";

  // SIFT !high precision low variance !!med precision & variance
  const float min_scale = 0.004f; //003 ! 004!!
  const int n_octaves = 4; //5 ! 4!!
  const int n_scales_per_octave = 5; // 10 ! 5!
  const float min_contrast = 0.001f; //001 !
  pcl::SIFTKeypoint<NormalType, pcl::PointWithScale> sift;
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);

  // normal estimation
  pcl::NormalEstimationOMP<PointType, NormalType> ne;
  double inff = std::numeric_limits<double>::infinity();
  ne.setViewPoint(inff, inff, inff);
  ne.setRadiusSearch (0.005); // !!! > res ~= 0.001xxx

  // ESF
  pcl::ESFEstimation<PointType, pcl::ESFSignature640> esf;

//  cout << esf.getKSearch() << " -- " << esf.getRadiusSearch();
//  esf.setKSearch(0);
//  esf.setRadiusSearch(0.5f);

  // PFH
  pcl::PFHEstimation<pcl::PointXYZ, NormalType, pcl::PFHSignature125> pfh;
  pfh.setRadiusSearch(pfhSize); // 1cm

  // SHOT
  pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> shot;
  shot.setRadiusSearch(shotSize); // 3cm

  CloudPtr cloud(new Cloud);
  // iterate recursively through directory
  for( fs::recursive_directory_iterator end, dir(folder); dir != end; ++dir )
  {
    fs::path path(*dir);
    std::string filename = path.stem().string();
    std::string fileabs = fs::canonical(path).string();

    if(!filename.compare("bad") || !filename.compare("false")
      || !filename.compare("patches_2cm")
      || !filename.compare("patches_3cm")
      || !filename.compare("patches_4cm")
      || !filename.compare("patches_5cm"))
    {
      dir.no_push();
      continue;
    }
    else // only files not in "bad" and "false" folders
    {
      cout << "\nProcessing: " << path.string() << "\n";
      if(fs::extension(path) != ".pcd")
        continue; // skip non-pcd files

      if(pcl::io::loadPCDFile<PointType>(fileabs, *cloud) == -1) // load the file
        continue;

      // remove NaNs
//      std::vector<int> indices;
//      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

      unsigned pos = filename.find("_");
      std::string frame_name = filename.substr (0, pos);
      std::stringstream tmp; tmp << "-" << rand() % 1000; // uniq frame name
      frame_name += tmp.str();
      std::string patch_name = filename.substr (pos+1);

      // kdtree
      pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
      esf.setSearchMethod(tree);
      pfh.setSearchMethod(tree);
      shot.setSearchMethod(tree);
      ne.setSearchMethod(tree);

      // normal estimation
      ne.setInputCloud(cloud);
      pcl::PointCloud<NormalType>::Ptr normals (new pcl::PointCloud<NormalType>);
      ne.compute (*normals);
      // copy the xyz field in PointNormals estimation since its zero
      for(size_t i = 0; i<normals->points.size(); ++i)
      {
        normals->points[i].x = cloud->points[i].x;
        normals->points[i].y = cloud->points[i].y;
        normals->points[i].z = cloud->points[i].z;
      }
      // remove NaNs from normals (this breaks since #normals == #points)
//      pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);

      // SIFT
      pcl::PointCloud<pcl::PointWithScale> sift_kpts;
      pcl::search::KdTree<NormalType>::Ptr tree2(new pcl::search::KdTree<NormalType>());
      sift.setSearchMethod(tree2);
      sift.setInputCloud(normals);
      sift.compute(sift_kpts);
      std::cout << "# SIFT kpts (SHOT): " << sift_kpts.points.size() << std::endl;

      // Keep an original copy
      pcl::PointCloud<pcl::PointXYZ>::Ptr kpts_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(sift_kpts, *kpts_cloud);

      // this one for icp just to get new XYZ
      pcl::PointCloud<pcl::PointXYZ>::Ptr kpts_cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(sift_kpts, *kpts_cloud_icp);

      icp.setInputSource(kpts_cloud_icp);
      icp.align(*kpts_cloud_icp);

      // save cloud resolution as a measure of distance to camera
      double cloud_res = computeCloudResolution(cloud);

      Eigen::MatrixXf buffer;
      // save SHOT features
      for (size_t i = 0; i<kpts_cloud->points.size(); ++i)
      {
        CloudPtr queryPt (new Cloud);
        queryPt->push_back(kpts_cloud->at(i));

        // SHOT
        shot.setInputCloud(queryPt);
        shot.setInputNormals(normals);
        shot.setSearchSurface(cloud);
        pcl::PointCloud<pcl::SHOT352>::Ptr desc(new pcl::PointCloud<pcl::SHOT352>);
        shot.compute(*desc);

        buffer = desc->getMatrixXfMap();
        for(int p = 0 ; p < 352; ++p)
          shot_writer << buffer(p, 0) << ",";
        shot_writer << frame_name << "," << patch_name << "," << label << ","
                    << kpts_cloud_icp->points[i].x << ","
                    << kpts_cloud_icp->points[i].y << ","
                    << kpts_cloud_icp->points[i].z << ","
                    << cloud_res << "\n";
      }

      // uniform sampling for PFH (add to SIFT kpts)
      us.setRadiusSearch (0.02);
      us.setInputCloud(cloud);
      pcl::PointCloud<int> smp_indices;
      us.compute (smp_indices);
      pcl::copyPointCloud (*cloud, smp_indices.points, *kpts_cloud);
      std::cout << "# Uniform sampling kpts (PFH): " << smp_indices.points.size() << "\n";

      // save PFH features
      for (size_t i = 0; i<kpts_cloud->points.size(); ++i)
      {
        CloudPtr queryPt (new Cloud);
        queryPt->push_back(kpts_cloud->at(i));

        // PFH
        pfh.setInputCloud(queryPt);
        pfh.setInputNormals(normals);
        pfh.setSearchSurface(cloud);
        pcl::PointCloud<pcl::PFHSignature125>::Ptr histcloudPFH(new pcl::PointCloud<pcl::PFHSignature125>);
        pfh.compute(*histcloudPFH);

        buffer = histcloudPFH->getMatrixXfMap();
        for(int p = 0 ; p < 125; ++p)
          pfh_writer << buffer(p, 0) <<",";
        pfh_writer << frame_name << "," << patch_name << "," << label << ","
                   << kpts_cloud_icp->points[i].x << ","
                   << kpts_cloud_icp->points[i].y << ","
                   << kpts_cloud_icp->points[i].z << ","
                   << cloud_res << "\n";
      }

      // ESF -- does not take into consideration any keypoints, global descriptor
      esf.setInputCloud(cloud);
      pcl::PointCloud<pcl::ESFSignature640>::Ptr histcloudESF(new pcl::PointCloud<pcl::ESFSignature640>);
      esf.compute(*histcloudESF);

      buffer = histcloudESF->getMatrixXfMap();
      for(int p = 0 ; p < 640; ++p)
        esf_writer << buffer(p, 0) <<",";
      esf_writer << frame_name << "," << patch_name << "," << label << "," << cloud_res << "\n";

// VISUALIZATION ---------------------
// keypoints
//      std::stringstream ss;
//      ss << filename << "kp1";
//      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgbp(kpts_cloud_icp);
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpt(kpts_cloud, 0, 255, 0);
//      viewer->addPointCloud(kpts_cloud_icp, kpt, ss.str(), v2);
//      ss << filename << "kp2";
//      viewer->addPointCloud(kpts_cloud, kpt, ss.str(), v1);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss.str());
//cloud
//      pcl::visualization::PointCloudColorHandlerRandom<PointType> rgb(cloud);
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 200, 0, 0);
//      viewer->addPointCloud<PointType>(cloud, red, filename, v1);
// vis normals
//      std::stringstream sstr; sstr << "clone";
//      viewer->addPointCloudNormals<PointType, NormalType>(cloud, normals, 5, 0.01, "norm1", v1);
// vis camera
//      viewer->spinOnce();
//      viewer->removeAllPointClouds(v1);
    }
  }
//  viewer->spin();
//  viewer->close();

  esf_writer.close();
  pfh_writer.close();
  shot_writer.close();
}
