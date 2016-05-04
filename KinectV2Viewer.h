#ifndef KINECTV2VIEWER_H
#define KINECTV2VIEWER_H

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>

using namespace pcl::io::openni2;

template <typename PointType>
class KinectV2Viewer
{
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    
    KinectV2Viewer (pcl::Grabber* grabber, pcl::visualization::PCLVisualizer *cloud_viewer)
    : cloud_viewer_ (cloud_viewer)
    , image_viewer_ ()
    , depth_image_viewer_ ()
    , grabber_ (grabber)
    , rgb_data_ (0), rgb_data_size_ (0)
    {
    }
    
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
        boost::mutex::scoped_lock lock (cloud_mutex_);
        cloud_ = cloud;
    }
    
    void
    image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
    {
        boost::mutex::scoped_lock lock (image_mutex_);
        image_ = image;
        
        if (image->getEncoding () != pcl::io::openni2::Image::RGB)
        {
            if (rgb_data_size_ < image->getWidth () * image->getHeight ())
            {
                if (rgb_data_)
                    delete [] rgb_data_;
                rgb_data_size_ = image->getWidth () * image->getHeight ();
                rgb_data_ = new unsigned char [rgb_data_size_ * 3];
            }
            image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
        }
    }
    
    void
    depth_image_callback (const boost::shared_ptr<pcl::io::DepthImage>& depth_image)
    {
        boost::mutex::scoped_lock lock (depth_image_mutex_);
        depth_image_ = depth_image;
    }
    
    /**
     * @brief starts the main loop
     */
    void
    run ()
    {
        cloud_viewer_->setCameraFieldOfView (1.02259994f);
        boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&KinectV2Viewer::cloud_callback, this, _1);
        cloud_connection_ = grabber_->registerCallback (cloud_cb);
        
        if (grabber_->providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
        {
            image_viewer_ = new pcl::visualization::ImageViewer ("PCL OpenNI image");
            boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&KinectV2Viewer::image_callback, this, _1);
            image_connection_ = grabber_->registerCallback (image_cb);
        }
        
        depth_image_viewer_ = new pcl::visualization::ImageViewer ("Depth Image");
        boost::function<void (const boost::shared_ptr<pcl::io::DepthImage>&) > depth_image_cb = boost::bind (&KinectV2Viewer::depth_image_callback, this, _1);
        depth_connection_ = grabber_->registerCallback (depth_image_cb);
        
        bool image_init = false, cloud_init = false, depth_image_init = false;
        
        grabber_->start ();
        
        while (grabber_->isRunning())
        {
            boost::shared_ptr<pcl::io::openni2::Image> image;
            boost::shared_ptr<pcl::io::DepthImage> depth_image;
            CloudConstPtr cloud;
            
            cloud_viewer_->spinOnce ();
            
            // See if we can get a cloud
            if (cloud_mutex_.try_lock ())
            {
                cloud_.swap (cloud);
                cloud_mutex_.unlock ();
            }
            
            if (cloud)
            {
                if (!cloud_init)
                {
                    cloud_viewer_->setPosition (0, 0);
                    cloud_viewer_->setSize (cloud->width, cloud->height);
                    cloud_init = !cloud_init;
                }
                
                if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
                {
                    cloud_viewer_->addPointCloud (cloud, "OpenNICloud");
                    cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
                    cloud_viewer_->setCameraPosition (0,0,0,		// Position
                                                      0,0,1,		// Viewpoint
                                                      0,-1,0);	// Up
                }
            }
            
            // See if we can get an image
            if (image_mutex_.try_lock ())
            {
                image_.swap (image);
                image_mutex_.unlock ();
            }
            
            if (image)
            {
                if (!image_init && cloud && cloud->width != 0)
                {
                    image_viewer_->setPosition (cloud->width, 0);
                    image_viewer_->setSize (image->getWidth (), image->getHeight ());
                    image_init = !image_init;
                }
                
                if (image->getEncoding () == pcl::io::openni2::Image::RGB)
                    image_viewer_->addRGBImage ( (const unsigned char*)image->getData (), image->getWidth (), image->getHeight ());
                else
                    image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
                image_viewer_->spinOnce ();
            }
            
            // See if we can get an ir image
            if (depth_image_mutex_.try_lock ())
            {
                depth_image_.swap (depth_image);
                depth_image_mutex_.unlock ();
            }
            
            if (depth_image)
            {
                if (!depth_image_init && cloud && cloud->width != 0)
                {
                    depth_image_viewer_->setPosition (cloud->width, cloud->height);
                    depth_image_viewer_->setSize (depth_image->getWidth (), depth_image->getHeight ());
                    depth_image_init = !depth_image_init;
                }
                
                depth_image_viewer_->addShortImage(depth_image->getData (), depth_image->getWidth (), depth_image->getHeight (), true);
                
                depth_image_viewer_->spinOnce ();
            }
        }
        
        cloud_viewer_->removeAllPointClouds();
        cloud_viewer_->spinOnce ();
        image_viewer_->close();
        depth_image_viewer_->close();
        
        cloud_connection_.disconnect ();
        image_connection_.disconnect ();
        depth_connection_.disconnect ();
        if (rgb_data_)
            delete[] rgb_data_;
    }
    
    pcl::visualization::PCLVisualizer *cloud_viewer_;
    pcl::visualization::ImageViewer *image_viewer_;
    pcl::visualization::ImageViewer *depth_image_viewer_;
    
    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;
    boost::signals2::connection depth_connection_;
    
    pcl::Grabber* grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;
    boost::mutex depth_image_mutex_;
    
    CloudConstPtr cloud_;
    boost::shared_ptr<pcl::io::openni2::Image> image_;
    boost::shared_ptr<pcl::io::DepthImage> depth_image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
};

#endif // KINECTV2VIEWER_H