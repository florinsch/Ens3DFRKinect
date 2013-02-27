/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *	
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// test & compile later (uncommented from CmakeLists)
//#include <pcl/registration/incremental_registration.h>

#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <typeinfo>
#include <exception>

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

template <typename PointType>
class OpenNIRegisterVoxelGridICP
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIRegisterVoxelGridICP (
                      const std::string& device_id = "",
                      const std::string& field_name = "z", float min_v = 0, float max_v = 5.0,
                      float leaf_size_x = 0.025, float leaf_size_y = 0.025, float leaf_size_z = 0.025,
                      unsigned int max_iterations = 50, float max_corr_dist = 0.1, float ransac_outlier = 0.5
                      )
    : viewer ("PCL OpenNI VoxelGrid ICP Template matcher")
    , device_id_(device_id)
    {
      device_id_ = device_id;
      viewer.setBackgroundColor (0, 0, 0);
      //viewer.addCoordinateSystem (1.0);
      viewer.initCameraParameters ();

      // keep only what's in between 0.25m and 1.5m from the cam
      pass_.setFilterFieldName ("z");
      pass_.setFilterLimits (0.25, 1.5);
      pass_.setKeepOrganized(false);

      // voxel grid downsampling parameters
      grid_.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
      grid_.setFilterFieldName (field_name);
      grid_.setFilterLimits (min_v, max_v);

      // iterative closest point parameters
      icp_.setMaximumIterations(max_iterations);
      icp_.setMaxCorrespondenceDistance (max_corr_dist);
      //icp_.setRANSACOutlierRejectionThreshold (ransac_outlier);

      //icp_.setRANSACIterations (30);
      //icp_.setTransformationEpsilon (0.5);
      //icp_.setEuclideanFitnessEpsilon(1);

      //reg_.setDownsamplingLeafSizeInput(0.03);
      //reg_.setDownsamplingLeafSizeModel(0.03);
      //reg_.setRegistrationDistanceThreshold(0.5);

      // init identity matrix
      t_ = Eigen::Matrix4f::Identity ();
      working_ = false;
    }

    void
    load_and_downsample_template (std::string cloud_name)
    {
      Cloud temp;
      try
      {
        pcl::io::loadPCDFile<PointType> (cloud_name, temp);

        template_ = temp.makeShared();
        grid_.setInputCloud (template_);
        grid_.filter (temp);
        template_ = temp.makeShared();
      }
      catch (std::exception& e)
      {
        PCL_ERROR(e.what ());
      }
    }

    void 
    cloud_cb_ (const CloudConstPtr & cloud)
    {
      FPS_CALC ("computation");
      boost::mutex::scoped_lock lock (mtx_);

      cloud_ = cloud;

      filterPassThrough (cloud, *temp_cloud_);
      //downsample_camera ();

      align_template ();
    }

    void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
    {
      FPS_CALC("passthrough");
      pass_.setInputCloud (cloud);
      pass_.filter (result);
    }

    void
    downsample_camera()
    {
      temp_cloud_.reset (new Cloud);

      grid_.setInputCloud (cloud_);
      grid_.filter (*temp_cloud_);

      cloud_= temp_cloud_;
    }

    void
    align_template ()
    {
     // while(true)
      {
        //if(!working_)
        {
          working_ = true;

          CloudPtr temp (new Cloud);

          icp_.align (*temp);

          t_ = icp_.getLastIncrementalTransformation () * t_;
          pcl::transformPointCloud (*template_, *temp, t_);
          template_ = temp;

          PCL_INFO ("working ...");
        }

        //PCL_INFO ("fitness %f\n", icp_.getFitnessScore ());

        if(icp_.hasConverged () == 1)
        {
          working_ = false;
          icp_.setInputTarget (cloud_);
          icp_.setInputCloud (template_);
          PCL_INFO ("Converged, Fitness score:%f\n", icp_.getFitnessScore ());
        }
      }
    }

    void
    run (std::string template_name)
    {
      load_and_downsample_template (template_name);

      // Set OpenNI interface and register callback
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);
      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIRegisterVoxelGridICP::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();

      // Init template vis properties
      pcl::visualization::PointCloudColorHandlerCustom<PointType> green_handler (template_, 0, 255, 0);
      viewer.addPointCloud (template_, green_handler, "TemplateCloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 , "TemplateCloud");

      // Spin once and init window size
      viewer.spinOnce();
      viewer.getRenderWindow ()->SetSize (cloud_->width, cloud_->height);

      // Set source and target
      icp_.setInputTarget (cloud_);
      icp_.setInputCloud (template_);

      //align_template();

      while (!viewer.wasStopped ())
      {
        viewer.spinOnce();
        FPS_CALC ("drawing");

        if (cloud_ && mtx_.try_lock ())
        {
          pcl::visualization::PointCloudColorHandlerCustom<PointType> red_handler (cloud_, 255, 0, 0);
          if (!viewer.updatePointCloud (cloud_, red_handler, "OpenNICloud"))
          {
            viewer.removePointCloud("OpenNICloud");
            viewer.addPointCloud (cloud_, red_handler, "OpenNICloud");
            viewer.resetCameraViewpoint ("OpenNICloud");
          }

          if (!viewer.updatePointCloud (template_, green_handler, "TemplateCloud"))
          {
            viewer.removePointCloud("TemplateCloud");
            //if(icp_.hasConverged () && icp_.getFitnessScore () > 0.0002)
            viewer.addPointCloud (template_, green_handler, "TemplateCloud");
            viewer.resetCameraViewpoint ("TemplateCloud");
          }
          //mtx_.unlock ();
        }
       boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }
      interface->stop ();
    }

    pcl::visualization::PCLVisualizer viewer;
    pcl::VoxelGrid<PointType> grid_;
    pcl::PassThrough<PointType> pass_;

    //TODO: test GeneralICP and nonlinear
    pcl::IterativeClosestPointNonLinear<PointType, PointType> icp_;
    //pcl::registration::IncrementalRegistration<PointType> reg_;

    Eigen::Matrix4f t_;
    std::string device_id_;
    boost::mutex mtx_;
    CloudConstPtr cloud_, template_;
    CloudPtr temp_cloud_;
    bool working_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <template.pcd> <options>\n\n"
            << "where options are:\n"
            << "\t-minmax min-max  :: set the PassThrough min-max cutting values (default: 0-5.0)\n"
            << "\t-field  X \t :: use field/dimension 'X' to filter data on (default: 'z')\n"
            << "\t-leaf x, y, z \t :: set the VoxelGrid leaf size (default: 0.01)\n"
            << "\t-ransac X \t :: set the RANSAC outlier rejection threshold X (default: 0.5)\n"
            << "\t-max_corr X \t :: set the maximum X correspondence distance (default: 0.5)\n"
            << "\t-iter X \t :: set the maximum number X of iterations (default: 50)\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      std::cout << "\nDevice: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
           << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      std::cout << "\tdevice_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "\tbus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "\t<serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    std::cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{

  if (pcl::console::find_argument (argc, argv, "-h") != -1 || argc == 1)
  {
    usage (argv);
    return (1);
  }

  std::string template_name (argv[pcl::console::parse_file_extension_argument (argc, argv, ".pcd")[0]]);

  /*
   * TODO: figure out why this doesnt work (weird stuff here)
  if(argc > 1 && ( template_name.length() > 0  || template_name.empty() ))
    return (-1);
  */

  double min_v = 0, max_v = 5.0;
  pcl::console::parse_2x_arguments (argc, argv, "-minmax", min_v, max_v, false);
  std::string field_name ("z");
  pcl::console::parse_argument (argc, argv, "-field", field_name);
  PCL_INFO ("Filtering data on %s between %f -> %f.\n", field_name.c_str (), min_v, max_v);
  double leaf_x = 0.025, leaf_y = 0.025, leaf_z = 0.025;
  pcl::console::parse_3x_arguments (argc, argv, "-leaf", leaf_x, leaf_y, leaf_z, false);
  PCL_INFO ("Using %f, %f, %f as a leaf size for VoxelGrid.\n", leaf_x, leaf_y, leaf_z);
  unsigned int max_iter = 50; double max_corr = 0.5, ransac = 0.5;
  pcl::console::parse_argument (argc, argv, "-max_corr", max_corr);
  pcl::console::parse_argument (argc, argv, "-ransac", ransac);
  pcl::console::parse_argument (argc, argv, "-iter", max_iter);
  PCL_INFO ("Using maximum correspondence distance of %f,\nRANSAC outlier rejection threshold is %f,\nMax iterations for ICP is %u.\n", max_corr, ransac, max_iter);

  pcl::OpenNIGrabber grabber ("");
/*
 * XYZRGB seems to crash, test later
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIRegisterVoxelGridICP<pcl::PointXYZRGB> v ("", field_name, min_v, max_v, leaf_x, leaf_y, leaf_z, max_iter, max_corr, ransac);
    v.run (template_name);
  }
  else */
  {
    OpenNIRegisterVoxelGridICP<pcl::PointXYZ> v ("", field_name, min_v, max_v, leaf_x, leaf_y, leaf_z, max_iter, max_corr, ransac);
    v.run (template_name);
  }

  return (0);
}

