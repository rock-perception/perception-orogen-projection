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

void ColorizePointcloud::cameraTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera_sample)
{
    // only process once per pointcloud
    if( !has_points )
	return;

    // get calibration matrix
    frame_helper::CameraCalibration calib = 
	frame_helper::CameraCalibration::fromFrame( *camera_sample );

    if( !calib.isValid() )
	throw std::runtime_error("No valid calibration matrix embedded in frame");

    Eigen::Matrix3d camMatrix = 
	calib.getCameraMatrix().cast<double>();

    // convert to target colorspace and apply undistort
    frame.init( camera_sample->size.width, camera_sample->size.height, 8, base::samples::frame::MODE_RGB );
    frameHelper.convert( *camera_sample, frame, 0, 0, frame_helper::INTER_LINEAR, true );

    // get the transformation
    Eigen::Affine3d points2cam;
    if( !_pointcloud2camera.get( ts, points2cam ) )
	throw std::runtime_error("Could not get pointcloud to camera transformation");

    // prepare the target pointcloud
    points.colors.resize( points.points.size() );
    
    // iterate through all the points 
    for( size_t i = 0; i < points.points.size(); ++i )
    {
	// get image coordinate of point
	Eigen::Vector3d p = camMatrix * points2cam * points.points[i];
	if( p.z() > 0 )
	{
	    int x = p.x() / p.z();
	    int y = p.y() / p.z();

	    // is in image
	    if( x >= 0 && x < frame.size.width && y >= 0 && y < frame.size.height )
	    {
		rgb *v = (rgb*)&frame.at<uint8_t>( x, y );
		points.colors[i] = base::Vector4d( v->r, v->g, v->b, 255.0 ) / 255.0;
	    }
	    else
	    {
		points.colors[i] = base::Vector4d::Zero();
	    }
	}
    }

    // write out the result
    _colored_points.write( points );

    if( !_output_ply.value().empty() )
	writePlyFile( points, _output_ply.value() );

    has_points = false;
}

void ColorizePointcloud::pointsTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &points_sample)
{
    points = points_sample;
    has_points = true;
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

    has_points = false;
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
