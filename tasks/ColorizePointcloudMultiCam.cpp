/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ColorizePointcloudMultiCam.hpp"

using namespace projection;

ColorizePointcloudMultiCam::ColorizePointcloudMultiCam(std::string const& name)
    : ColorizePointcloudMultiCamBase(name)
{
}

ColorizePointcloudMultiCam::ColorizePointcloudMultiCam(std::string const& name, RTT::ExecutionEngine* engine)
    : ColorizePointcloudMultiCamBase(name, engine)
{
}

ColorizePointcloudMultiCam::~ColorizePointcloudMultiCam()
{
}

struct __attribute__((__packed__)) rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};



void ColorizePointcloudMultiCam::colorizePointCloud(base::samples::Pointcloud& pointsCloud, base::samples::frame::Frame& image, const Eigen::Matrix4d& points2Cam, base::samples::frame::Frame& image2, const Eigen::Matrix4d& points2Cam2
)
{
    // prepare the target pointcloud
    //init pointcloud color with black (unknown)
    pointsCloud.colors.resize( pointsCloud.points.size(), base::Vector4d::Zero());

    // iterate through all the points 
    for( size_t i = 0; i < pointsCloud.points.size(); ++i )
    {
        const Eigen::Vector4d point(pointsCloud.points[i].x(), pointsCloud.points[i].y(), pointsCloud.points[i].z(), 1);
        
        // get image coordinate of point
        Eigen::Vector3d p = (points2Cam * point).head(3);
        if(p.z() > 0 )
        {
            int x = p.x() / p.z();
            int y = p.y() / p.z();

            // is in image
            if( x >= 0 && x < image.size.width && y >= 0 && y < image.size.height )
            {
                rgb *v = (rgb*)&image.at<uint8_t>( x, y );
                pointsCloud.colors[i] = base::Vector4d( v->r, v->g, v->b, 255.0 ) / 255.0;
            } 
            else
            {
                p = (points2Cam2 * point).head(3);
                if(p.z() > 0 )
                {
                    int x = p.x() / p.z();
                    int y = p.y() / p.z();

                    // is in image
                    if( x >= 0 && x < image.size.width && y >= 0 && y < image.size.height )
                    {
                        rgb *v = (rgb*)&image.at<uint8_t>( x, y );
                        pointsCloud.colors[i] = base::Vector4d( v->r, v->g, v->b, 255.0 ) / 255.0;
                    } 
                }
            }
        }
    }
    
}

void ColorizePointcloudMultiCam::camera1Callback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera1_sample)
{
    // get calibration matrix
    frame_helper::CameraCalibration calib = 
        frame_helper::CameraCalibration::fromFrame( *camera1_sample );

    if( !calib.isValid() )
        throw std::runtime_error("No valid calibration matrix embedded in frame");

    // convert to target colorspace and apply undistort
    frame1.init( camera1_sample->size.width, camera1_sample->size.height, 8, base::samples::frame::MODE_RGB );
    frameHelper.convert( *camera1_sample, frame1, 0, 0, frame_helper::INTER_LINEAR, _doUndistortion.get());

    hasImage1 = true;
}

void ColorizePointcloudMultiCam::camera2Callback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera2_sample)
{
    // get calibration matrix
    frame_helper::CameraCalibration calib = 
        frame_helper::CameraCalibration::fromFrame( *camera2_sample );

    if( !calib.isValid() )
        throw std::runtime_error("No valid calibration matrix embedded in frame");

    // convert to target colorspace and apply undistort
    frame2.init( camera2_sample->size.width, camera2_sample->size.height, 8, base::samples::frame::MODE_RGB );
    frameHelper.convert( *camera2_sample, frame2, 0, 0, frame_helper::INTER_LINEAR, _doUndistortion.get());

    hasImage2 = true;
}

void ColorizePointcloudMultiCam::pointsCallback(const base::Time &ts, const ::base::samples::Pointcloud &points_sample)
{
    if(!(hasImage1 && hasImage2))
        return;
    
    colorizePointCloud(points, frame1, _pc2Cam1.get(), frame2, _pc2Cam2.get());
    
    _colored_points.write(points);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ColorizePointcloudMultiCam.hpp for more detailed
// documentation about them.

bool ColorizePointcloudMultiCam::configureHook()
{
    if (! ColorizePointcloudMultiCamBase::configureHook())
        return false;
    return true;
}
bool ColorizePointcloudMultiCam::startHook()
{
    if (! ColorizePointcloudMultiCamBase::startHook())
        return false;
    return true;
}
void ColorizePointcloudMultiCam::updateHook()
{
    ColorizePointcloudMultiCamBase::updateHook();
}
void ColorizePointcloudMultiCam::errorHook()
{
    ColorizePointcloudMultiCamBase::errorHook();
}
void ColorizePointcloudMultiCam::stopHook()
{
    ColorizePointcloudMultiCamBase::stopHook();
}
void ColorizePointcloudMultiCam::cleanupHook()
{
    ColorizePointcloudMultiCamBase::cleanupHook();
}
