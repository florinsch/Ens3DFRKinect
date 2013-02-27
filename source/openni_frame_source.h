#ifndef OPENNI_CAPTURE_H
#define OPENNI_CAPTURE_H

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace OpenNIFrameSource
{

  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

  /* A simple class for capturing data from an OpenNI camera */
  class PCL_EXPORTS OpenNIFrameSource
  {
  public:
    OpenNIFrameSource (const std::string& device_id = ""):
      grabber_ (device_id), most_recent_frame_ (), frame_counter_ (0),
      active_ (true), recording_(false), camera_track_(false), show_mask_(false), do_icp_(false)
    {
        boost::function<void(const PointCloudConstPtr&)> frame_cb = boost::bind (&OpenNIFrameSource::onNewFrame, this, _1);
        grabber_.registerCallback (frame_cb);
        grabber_.start ();
    }
    ~OpenNIFrameSource ();

    const PointCloudPtr
    snap ();
    bool
    isActive ();
    
    inline void
    onKeyboardEvent(const pcl::visualization::KeyboardEvent & event)
    {
        mutex_.lock ();
        if (event.keyUp ())
            switch (event.getKeyCode ())
            {
                case 27: case 'Q': case 'q': active_ = false;
                break;
                case ' ':
                    recording_ = !recording_;
                    cout << "\n\tRecording toggled "
                         << ( recording_? "ON":"OFF" )
                         <<  " at frame [" << frame_counter_ << "]\n";
                break;
                case 'T': case 't': camera_track_ = !camera_track_;
                break;
                case 'M': case 'm': show_mask_ = !show_mask_;
                break;
                case 'I': case 'i': do_icp_ = !do_icp_;
                break;
                case 'F': case 'f': fullscreen_ = !fullscreen_;
                break;
            }
        mutex_.unlock ();
    };
    
    inline int
    getNoFrames() { return frame_counter_; };
    
    inline bool
    isRecording() { return recording_; };
    
    inline bool
    isTrackingCam() { return camera_track_; };

    inline bool
    isShowingMask() { return show_mask_; };

    inline bool
    isEnabledICP() { return do_icp_; };

    inline bool
    isFullscreen() { return fullscreen_; };

  protected:
    void
    onNewFrame (const PointCloudConstPtr &cloud);

    pcl::OpenNIGrabber grabber_;
    PointCloudPtr most_recent_frame_;
    int frame_counter_;
    boost::mutex mutex_;
    bool active_;
    bool recording_;
    bool camera_track_;
    bool show_mask_;
    bool do_icp_;
    bool fullscreen_;
  };
}
#endif
