#include "boost/filesystem.hpp"
//#include <iostream>
//#include <cstdio>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

using namespace pcl;
using std::cout;
namespace fs = boost::filesystem;

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

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
  if(event.getKeySym() == "d" && event.keyDown())
    cout << "d";
  if(event.getKeySym() == "n" && event.keyDown())
    viewer->removeAllPointClouds();
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbaVis()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  //viewer->registerKeyboardCallback(keyboardEventOccurred,(void*)&viewer);
  return(viewer);
}

void
printUsage(const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options]\n\n"
      << " Press [r] to remove a file or [n] for next item\n"
      << "-------------------------------------------\n"
      << "Options:\n"
      << "-------------------------------------------\n"
      << "-h this help\n"
      << "-dir folder to start recursive iteration(default current)\n"
      << "\n\n";
}

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

  // get absolute path to current folder
  fs::path absolute(folder);
  absolute = fs::canonical(absolute);
  // make empty folder for false positives
  fs::create_directories(fs::path(absolute)/="false");

  // create viewer and pointcloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbaVis();

  int v1(0), v2(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  CloudPtr cloud(new Cloud);
  // iterate recursively through directory
  for( fs::recursive_directory_iterator end, dir(folder); dir != end; ++dir )
  {
    fs::path path(*dir);
    std::string filename = path.filename().string();
    if(fs::is_directory(path) && (!filename.compare("bad") || !filename.compare("false")))
    {
      dir.no_push();
      continue;
    }
    else // only files not in "bad" and "false" folders
    {
      if(fs::extension(path) != ".pcd")
        continue;
      std::string fileabs = fs::canonical(path).string();
      if(pcl::io::loadPCDFile<PointType>(fileabs, *cloud) == -1) //* load the file
      {
        cout << "\n Couldn't read [" << filename << "] moving to './bad' \n";
        fs::create_directories(fs::path(absolute)/="bad");
        fs::path newpath(absolute);
        newpath /= "bad/" + filename;
        fs::rename(path, newpath);
        continue;
      }

      // compute centroid
      Eigen::Vector4f ctrd;
      pcl::compute3DCentroid (*cloud, ctrd);
      pcl::demeanPointCloud(*cloud, ctrd, *cloud);


      // Flip cloud around y axis (just for vis)
      CloudPtr new_cloud (new Cloud);
      float rot_x = 3.2f;
      float rot_y = 3.1f;
      float rot_z = 0.0f;
      pcl::transformPointCloud(*cloud, *new_cloud, pcl::getTransformation (0,0,0, rot_x, rot_y, rot_z));

      pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
      viewer->addPointCloud<PointType>(cloud, rgb, filename, v1);
      std::stringstream ss; ss << filename << "new";
      viewer->addPointCloud<PointType>(new_cloud, rgb, ss.str(), v2);
      viewer->resetCamera();
      viewer->addText(path.string(), 10, 30, "info");
      viewer->spinOnce();

      cout << "Move [ " << filename << " ] to ./false/ ? (y/n) ";
      char c; cin.get(c); cin.ignore();
      if (c == 'y' || c == 'Y')
      {
        fs::path newpath(absolute);
        newpath /= "false/" + filename;
        fs::rename(path, newpath);
      }

      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
    }
  }
  viewer->close();
}
