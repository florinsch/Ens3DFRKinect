#include "boost/filesystem.hpp"
#include <algorithm>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

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
      << "-------------------------------------------\n"
      << "Options:\n"
      << "-------------------------------------------\n"
      << "-h this help\n"
      << "-dir folder to start recursive iteration (default: current)\n"
      << "  Subfolders named: 'bad', 'false' and 'patches_*' are excluded\n"
      << "-leaf size in meters (default: 0.05) \n"
      << "-vis 1 to visualise, else make patches (default: false) \n"
      << "\n\n";
}

int
main(int argc, char ** argv)
{
  // parse arguments
  if(pcl::console::find_argument(argc, argv, "-h") >= 0 || argc <= 1)
  {
    printUsage(argv[0]);
    return 0;
  }
  std::string folder = ".";
  float leaf = 0.05f;
  bool vis = false;
  pcl::console::parse_argument(argc, argv, "-dir", folder);
  pcl::console::parse_argument(argc, argv, "-leaf", leaf);
  pcl::console::parse_argument(argc, argv, "-vis", vis);

  // get absolute path to current folder
  fs::path absolute(folder);
  absolute = fs::canonical(absolute);
  std::stringstream patchname; patchname << "patches_" << leaf*100 << "cm";

  // setup viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbaVis();
  if(!vis)
  {
//    std::stringstream ss; ss << absolute.string() << "/" << patchname.str();
//    fs::create_directories(ss.str());
    viewer->close();
//    viewer->~PCLVisualizer();
//    viewer.reset();
  }
  // voxel grid for generating patches
  pcl::VoxelGrid<PointType> vgrid;
  vgrid.setLeafSize(leaf, leaf, std::numeric_limits<float>::infinity());

  // remove non-connected points
//  pcl::EuclideanClusterExtraction<pcl::PointXYZ> xclstrs;
//  xclstrs.setClusterTolerance(0.003); // 1cm = 0.01
//  xclstrs.setMinClusterSize(1000);
//  xclstrs.setMaxClusterSize(20000);

  CloudPtr cloud(new Cloud);
  // iterate recursively through directory
  for( fs::recursive_directory_iterator end, dir(folder); dir != end; ++dir )
  {
    fs::path filepath(*dir);
    std::string filename = filepath.filename().string();
    std::string fileabs = fs::canonical(filepath).string();
    std::string current_dir = fs::canonical(filepath.parent_path()).string();
    std::stringstream savefolder; savefolder << current_dir << "/" << patchname.str() << "/";

    if(fs::is_directory(filepath))
    {
      if(!filename.compare("bad") || !filename.compare("false") || !filename.find("patches_") )
      {
        dir.no_push();
        continue;
      }
      else if(!vis)  // create subdirectory
        fs::create_directories(fs::canonical(filepath)/=patchname.str());
    }
    else // only files not in "bad", "false" and patches_* folders
    {
      if(vis)
        viewer->addText(fileabs, 10, 30, "info");

      if(fs::extension(filepath) != ".pcd")
        continue; // skip non-pcd files

      if(pcl::io::loadPCDFile<PointType>(fileabs, *cloud) == -1) //* load the file
        continue;

      int cloud_size = cloud->size(); // remember cloud size

      // apply voxel grid, save leafs
      vgrid.setInputCloud (cloud);
      vgrid.setSaveLeafLayout(true);
      CloudPtr cloud_down(new Cloud);
      vgrid.filter(*cloud_down);

      std::vector<int> uniq; // contains ids of all centroids
      for(size_t i = 0; i < cloud_size; ++i)
        uniq.push_back(vgrid.getCentroidIndex(cloud->points[i]));

      // copy indices of all points <-> centroids
      std::vector<int> ctrids(uniq);

      // sort and remove duplicates to get unique items
      std::sort(uniq.begin(), uniq.end());
      uniq.erase( unique( uniq.begin(), uniq.end()), uniq.end() );
      int no_patches = uniq.size();
      float avg_patch_size = cloud_size / no_patches;

      // for each centroid, go through cloud
      for(unsigned int u = 0; u < no_patches; ++u)
      {
        pcl::PointIndices::Ptr patch_indices (new pcl::PointIndices());
        patch_indices->header = cloud->header;
        // extract indices for each centroid
        for(size_t i = 0; i < cloud_size; ++i)
          if(uniq[u] == ctrids[i])
            patch_indices->indices.push_back(i);

        // TODO: add patch dimensions into consideration ?
        if(!vis && patch_indices->indices.size() / avg_patch_size < 0.75 )
          continue; // skip patches with 1/3 missing data

        // extract patch and convert to xyz
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<PointType, pcl::PointXYZ>(*cloud, *patch_indices, *patch);

        // remove non-connected points
//        xclstrs.setInputCloud(patch);
//        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//        xclstrs.setSearchMethod(tree);
//        std::vector<pcl::PointIndices> idx;
//        xclstrs.extract(idx);
//        pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*patch, idx, *patch);

        if(vis) // show cloud
        {
          pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(patch);
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(patch, 255, 0, 0);
          std::stringstream ss; ss << filename << u;
          if( patch_indices->indices.size() / avg_patch_size < 0.75 )
          {
            viewer->addPointCloud<pcl::PointXYZ>(patch, red, ss.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, ss.str());
          }
          else
            viewer->addPointCloud<pcl::PointXYZ>(patch, rgb, ss.str());

        }
        else // save to disk
        {
          // demean pointcloud (move to 0, 0, 0)
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid (*patch, centroid);
          pcl::PointCloud<pcl::PointXYZ>::Ptr demean(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::demeanPointCloud(*patch, centroid, *demean);
          // create name
          std::stringstream filess;
          filess << savefolder.str() << fs::basename(fileabs) << "_patch_" << u << ".pcd";
          // save patch
          if(patch->size())
            savePCDFile(filess.str(), *demean);
        }
      }
      if(vis) // see patches as segmented on the face
      {
        viewer->resetCamera();
        viewer->spinOnce();
        cout << "\n Press any key to go to the next frame, Ctrl+C to exit\n";
        cin.get(); // wait for keypress
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
      }
    }
  }
  viewer->close();
}
