/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ColorizePointcloud.hpp"
#include <fstream>

using namespace projection;

ColorizePointcloud::ColorizePointcloud(std::string const& name)
    : ColorizePointcloudBase(name)
{
}

ColorizePointcloud::ColorizePointcloud(std::string const& name, RTT::ExecutionEngine* engine)
    : ColorizePointcloudBase(name, engine)
{
}

ColorizePointcloud::~ColorizePointcloud()
{
}

struct __attribute__((__packed__)) rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void writePlyFile( const base::samples::Pointcloud& points, const std::string& file )
{
    std::ofstream data( file.c_str() );

    data << "ply" << "\n";
    data << "format ascii 1.0\n";

    data << "element vertex " << points.points.size() <<  "\n";
    data << "property float x\n";
    data << "property float y\n";
    data << "property float z\n";

    if( !points.colors.empty() )
    {
	data << "property uchar red\n";
	data << "property uchar green\n";
	data << "property uchar blue\n";
	data << "property uchar alpha\n";
    }
    data << "end_header\n";

    for( size_t i = 0; i < points.points.size(); i++ )
    {
	data 
	    << points.points[i].x() << " "
	    << points.points[i].y() << " "
	    << points.points[i].z() << " ";
	if( !points.colors.empty() )
	{
	    data 
		<< (int)(points.colors[i].x()*255) << " "
		<< (int)(points.colors[i].y()*255) << " "
		<< (int)(points.colors[i].z()*255) << " "
		<< (int)(points.colors[i].w()*255) << " ";
	}
	data << "\n";
    }
}

void ColorizePointcloud::cameraCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera_sample)
{
    // get calibration matrix
    frame_helper::CameraCalibration calib = 
        frame_helper::CameraCalibration::fromFrame( *camera_sample );

    if( !calib.isValid() )
        throw std::runtime_error("No valid calibration matrix embedded in frame");

    // convert to target colorspace and apply undistort
    frame.init( camera_sample->size.width, camera_sample->size.height, 8, base::samples::frame::MODE_RGB );
    frameHelper.convert( *camera_sample, frame, 0, 0, frame_helper::INTER_LINEAR, _doUndistortion.get());

    hasImage = true;
}

void ColorizePointcloud::colorizePointCloud(base::samples::Pointcloud& pointsCloud, base::samples::frame::Frame& image, const Eigen::Matrix4d& points2Cam)
{
    // prepare the target pointcloud
    //init pointcloud color with black (unknown)
    pointsCloud.colors.resize( pointsCloud.points.size(), base::Vector4d::Zero());

    // iterate through all the points 
    for( size_t i = 0; i < pointsCloud.points.size(); ++i )
    {
        //check if the color was already set
        if(!pointsCloud.colors[i].isZero())
            continue;
        
        // get image coordinate of point
        Eigen::Vector3d p = (points2Cam * Eigen::Vector4d(pointsCloud.points[i].x(), pointsCloud.points[i].y(), pointsCloud.points[i].z(), 1)).head(3);
        if(p.z() > 0 )
        {
            int x = p.x() / p.z();
            int y = p.y() / p.z();

            // is in image
            if( x >= 0 && x < image.size.width && y >= 0 && y < image.size.height )
            {
                const rgb *v = (const rgb*)&image.at<const uint8_t>( x, y );
                pointsCloud.colors[i] = base::Vector4d( v->r, v->g, v->b, 255.0 ) / 255.0;
            }
        }
    }
    
}


void ColorizePointcloud::pointsCallback(const base::Time &ts, const ::base::samples::Pointcloud &points_sample)
{
    if(!hasImage)
        return;

    points = points_sample;
    
    colorizePointCloud(points, frame, _pc2Cam.get());
    
    // write out the result
    _colored_points.write( points );

    if( !_output_ply.value().empty() )
        writePlyFile( points, _output_ply.value() );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ColorizePointcloud.hpp for more detailed
// documentation about them.

bool ColorizePointcloud::configureHook()
{
    if (! ColorizePointcloudBase::configureHook())
        return false;
    return true;
}
bool ColorizePointcloud::startHook()
{
    if (! ColorizePointcloudBase::startHook())
        return false;

    hasImage = false;
    return true;
}
void ColorizePointcloud::updateHook()
{
    ColorizePointcloudBase::updateHook();
}
void ColorizePointcloud::errorHook()
{
    ColorizePointcloudBase::errorHook();
}
void ColorizePointcloud::stopHook()
{
    ColorizePointcloudBase::stopHook();
}
void ColorizePointcloud::cleanupHook()
{
    ColorizePointcloudBase::cleanupHook();
}
