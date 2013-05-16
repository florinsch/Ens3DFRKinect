#include "boost/filesystem.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/filters/voxel_grid.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/esf.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/pfh.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

typedef pcl::Normal NormalType;

using namespace pcl;
using std::cout;
namespace fs = boost::filesystem;

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
      << "-dir folder to start recursive iteration (default: current)\n"
      << "-label for all the features (class name) (default: pwd) \n"
      << "\n\n";
}

// find . -mindepth 2 -type d -exec sh -c '../../../build/compute_features -dir {} -label ${PWD##*/}' \;
// from top folder:
// find . -mindepth 3 -type d -exec sh -c '../../build/compute_features -dir {}' \;

int
main(int argc, char ** argv)
{
  // parse arguments
  if(pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }
  std::string folder = ".";
  pcl::console::parse_argument(argc, argv, "-dir", folder);
  std::string absolute = fs::canonical(fs::path(folder)).string();
  std::string label = fs::path(folder).string();
  std::replace(label.begin(), label.end(), '/', '-');
  label.erase(std::remove(label.begin(), label.end(), '.'), label.end());
  pcl::console::parse_argument(argc, argv, "-label", label);

  std::stringstream ss;
  // Open 3 files for writing
  ss << absolute << "/features-ESF-" << label << ".csv";
  std::ofstream esf_writer(ss.str().c_str());
  ss.str(""); ss << absolute << "/features-PFH-" << label << ".csv";
  std::ofstream pfh_writer(ss.str().c_str());
  ss.str(""); ss << absolute << "/features-SHOT-" << label << ".csv";
  std::ofstream shot_writer(ss.str().c_str());
  cout << "\nFiles saved in: " << absolute << "\n";

  // histogram viewer
//  pcl::visualization::PCLHistogramVisualizer histvis;
  // cloud viewer
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbaVis();
//  int v1(0), v2(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->addCoordinateSystem(0.05);

  // kdtree
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

  // normal estimation
  pcl::NormalEstimationOMP<PointType, NormalType> ne;
  double inff = std::numeric_limits<double>::infinity();
  ne.setViewPoint(inff, inff, inff);
  ne.setRadiusSearch (0.003); // !!! > res ~= 0.001xxx
  ne.setSearchMethod(tree);

  // ESF
  pcl::ESFEstimation<PointType, pcl::ESFSignature640> esf;
  esf.setSearchMethod(tree);
//  cout << esf.getKSearch() << " -- " << esf.getRadiusSearch();
//  esf.setKSearch(0);
//  esf.setRadiusSearch(0.5f);

  // PFH
  pcl::PFHEstimation<pcl::PointXYZ, NormalType, pcl::PFHSignature125> pfh;
  pfh.setSearchMethod(tree);
  pfh.setRadiusSearch(0.5f);

  // SHOT
  pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> shot;
  shot.setSearchMethod(tree);
  shot.setRadiusSearch(0.5f);

  CloudPtr cloud(new Cloud);
  // iterate recursively through directory
  for( fs::recursive_directory_iterator end, dir(folder); dir != end; ++dir )
  {
    fs::path path(*dir);
    std::string filename = path.stem().string();
    std::string fileabs = fs::canonical(path).string();

    if(!filename.compare("bad") || !filename.compare("false") )
    {
      dir.no_push();
      continue;
    }
    else // only files not in "bad" and "false" folders
    {
      cout << "\n Processing: " << path.string() << "\n";
      if(fs::extension(path) != ".pcd")
        continue; // skip non-pcd files

      if(pcl::io::loadPCDFile<PointType>(fileabs, *cloud) == -1) // load the file
        continue;

      unsigned pos = filename.find("_");
      std::string frame_name = filename.substr (0, pos);
      std::string patch_name = filename.substr (pos+1);

      // Compute Centroid -- the query point for SHOT and PFH histograms
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*cloud, centroid);
      CloudPtr ctr (new Cloud);
      pcl::PointXYZ ctrPt;
      ctrPt.x = centroid[0];
      ctrPt.y = centroid[1];
      ctrPt.z = centroid[2];
      ctr->push_back(ctrPt);

      // normal estimation
      ne.setInputCloud(cloud);
      pcl::PointCloud<NormalType>::Ptr normals (new pcl::PointCloud<NormalType>);
      ne.compute (*normals);

//      // Clone and rotate around axis to test rotation invariance
//      CloudPtr new_cloud(new Cloud);
//      pcl::transformPointCloud(*cloud, *new_cloud, pcl::getTransformation (0, 0, 0, .5f, .5f, .5f));
//
//      // compute normals for rotated cloud
//      ne.setInputCloud (new_cloud);
//      pcl::PointCloud<NormalType>::Ptr new_normals (new pcl::PointCloud<NormalType>);
//      ne.compute (*new_normals);

      // SIFT
//      const float min_scale = 0.0001f;
//      const int n_octaves = 8;
//      const int n_scales_per_octave = 4;
//      const float min_contrast = 0.0001f;
//      pcl::SIFTKeypoint<NormalType, pcl::PointWithScale> sift;
//      pcl::PointCloud<pcl::PointWithScale> result;
//      pcl::search::KdTree<NormalType>::Ptr tree2(new pcl::search::KdTree<NormalType>());
//      sift.setSearchMethod(tree2);
//      sift.setScales(min_scale, n_octaves, n_scales_per_octave);
//      sift.setMinimumContrast(min_contrast);
//      sift.setInputCloud(normals);
//      sift.compute(result);
//      std::cout << "No of SIFT points in the result are " << result.points.size() << std::endl;
//
//      pcl::PointCloud<pcl::PointWithScale> new_result;
//      sift.setInputCloud(normals2);
//      sift.compute(new_result);
//      std::cout << "No of SIFT points in the result are " << new_result.points.size() << std::endl;
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
//      copyPointCloud(result, *cloud_temp);
//
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZ>);
//      copyPointCloud(new_result, *cloud_temp2);
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_temp, 0, 255, 0);
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2(cloud_temp, 255, 0, 0);
//
//      std::stringstream ss; ss << filename << "kp1";
//      viewer->addPointCloud(cloud_temp, keypoints_color_handler, ss.str(), v1);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, ss.str());
//      ss << "kp2";
//      viewer->addPointCloud(cloud_temp2, keypoints_color_handler2, ss.str(), v2);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, ss.str());


      // SHOT
      shot.setInputCloud(ctr);

      shot.setInputNormals(normals);
      shot.setSearchSurface(cloud);
      pcl::PointCloud<pcl::SHOT352>::Ptr desc(new pcl::PointCloud<pcl::SHOT352>);
      shot.compute(*desc);

      Eigen::MatrixXf tmp = desc->getMatrixXfMap();

      for(int p = 0 ; p < 352; ++p)
        shot_writer << tmp(p, 0) << ",";
      shot_writer << frame_name << "," << patch_name << "," << label << "\n";

      // 2nd feature
//      pcl::Histogram<352> hist;
//      std::copy(desc.points[0].descriptor, desc.points[0].descriptor+352, hist.histogram);
//      pcl::PointCloud< pcl::Histogram<352> >::Ptr histcloudSHOT (new pcl::PointCloud< pcl::Histogram<352> >);
//      histcloudSHOT->push_back(hist);

//      shot.setInputNormals(new_normals);
//      shot.setSearchSurface(new_cloud);
//      pcl::PointCloud<pcl::SHOT352> desc2;
//      shot.compute(desc2);
//
//      pcl::Histogram<352> hist2;
//      std::copy(desc2.points[0].descriptor, desc2.points[0].descriptor+352, hist2.histogram);
//      pcl::PointCloud< pcl::Histogram<352> >::Ptr new_histcloud (new pcl::PointCloud< pcl::Histogram<352> >);
//      new_histcloud->push_back(hist2);

      // PFH
      pfh.setInputCloud(ctr);

      pfh.setInputNormals(normals);
      pfh.setSearchSurface(cloud);
      pcl::PointCloud<pcl::PFHSignature125>::Ptr histcloudPFH(new pcl::PointCloud<pcl::PFHSignature125>);
      pfh.compute(*histcloudPFH);

      tmp = histcloudPFH->getMatrixXfMap();
      for(int p = 0 ; p < 125; ++p)
        pfh_writer << tmp(p, 0) <<",";
      pfh_writer << frame_name << "," << patch_name << "," << label << "\n";

//      pfh.setInputNormals(new_normals);
//      pfh.setSearchSurface(new_cloud);
//      pcl::PointCloud<pcl::PFHSignature125>::Ptr new_histcloud(new pcl::PointCloud<pcl::PFHSignature125>);
//      pfh.compute(*new_histcloud);

      // ESF
      esf.setInputCloud(cloud);
      pcl::PointCloud<pcl::ESFSignature640>::Ptr histcloudESF(new pcl::PointCloud<pcl::ESFSignature640>);
      esf.compute(*histcloudESF);

      tmp = histcloudESF->getMatrixXfMap();
      for(int p = 0 ; p < 640; ++p)
        esf_writer << tmp(p, 0) <<",";
      esf_writer << frame_name << "," << patch_name << "," << label << "\n";

//      esf.setInputCloud(new_cloud);
//      pcl::PointCloud<pcl::ESFSignature640>::Ptr new_histcloud(new pcl::PointCloud<pcl::ESFSignature640>);
//      esf.compute(*new_histcloud);

      // HIST VIS
//      int size = 640;
//      histvis.addFeatureHistogram(*histcloud, size, filename);
//      histvis.updateFeatureHistogram(*histcloud, size, filename);
//      std::stringstream histviss; histviss << filename << "hist2";
//      histvis.addFeatureHistogram(*new_histcloud, size, histviss.str());
//      histvis.updateFeatureHistogram(*new_histcloud, size, histviss.str());
//      histvis.updateWindowPositions();
//      histvis.spinOnce();

      // CLOUD VIS
//      pcl::visualization::PointCloudColorHandlerRandom<PointType> rgb(cloud);
//      viewer->addPointCloud<PointType>(cloud, rgb, filename, v1);
//
//      std::stringstream sstr; sstr << "clone";
//      viewer->addPointCloud<PointType>(new_cloud, rgb, sstr.str(), v2);
//      viewer->addPointCloudNormals<PointType, NormalType>(cloud, normals, 5, 0.01, "norm1", v1);
//      viewer->addPointCloudNormals<PointType, NormalType>(new_cloud, new_normals, 5, 0.01, "norm2", v2);
//
//      viewer->resetCamera();
//      viewer->spin();
//      cin.get(); // wait for keypress
//      viewer->removeAllPointClouds();
    }
  }
//  viewer->spin();
//  cin.get();
//  viewer->close();
  esf_writer.close();
  pfh_writer.close();
  shot_writer.close();
}
