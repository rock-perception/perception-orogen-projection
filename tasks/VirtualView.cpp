/* Generated from orogen/lib/orogen/templates/tasks/VirtualView.cpp */

#include "VirtualView.hpp"
#include <frame_helper/Calibration.h>
#include <frame_helper/FrameHelper.h>

using namespace projection;

VirtualView::VirtualView(std::string const& name)
    : VirtualViewBase(name)
{
}

VirtualView::VirtualView(std::string const& name, RTT::ExecutionEngine* engine)
    : VirtualViewBase(name, engine)
{
}

VirtualView::~VirtualView()
{
}

void VirtualView::checkComplete()
{
    if( (_cam1.connected() && !addedCams.count(1))
	|| (_cam2.connected() && !addedCams.count(2))
	|| (_cam3.connected() && !addedCams.count(3))
	|| (_cam4.connected() && !addedCams.count(4)) )
    {
	return;
    }

    base::samples::frame::Frame* frame_ptr = viewFrame.write_access();
    cv::Mat viewMat = frame_helper::FrameHelper::convertToCvMat( *frame_ptr );
    cv::cvtColor( hom.getVirtualImage(), viewMat, CV_RGBA2RGB );
    frame_ptr->time = last_sample_ts;
    frame_ptr->setStatus(base::samples::frame::STATUS_VALID);
    viewFrame.reset( frame_ptr );

    _virtual_cam.write( viewFrame );
    hom.clearVirtualImage();

    addedCams.clear();
}


void VirtualView::addCam( const base::Affine3d& cam2plane, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame >& frame, int id )
{
    // already added
    if( addedCams.count( id ) )
	return;

    // get calibration matrix
    frame_helper::CameraCalibration calib = 
	frame_helper::CameraCalibration::fromFrame( *frame );

    if( !calib.isValid() )
	throw std::runtime_error("No valid calibration matrix embedded in frame");

    Eigen::Matrix3f camMatrix = 
	calib.getCameraMatrix().cast<float>();

    // convert to target colorspace and apply undistort
    tmpFrame.init( frame->size.width, frame->size.height, 8, base::samples::frame::MODE_RGB );
    frameHelper.convert( *frame, tmpFrame, 0, 0, frame_helper::INTER_LINEAR, true );
    last_sample_ts = frame->time;

    // get cv image
    cv::Mat img = frame_helper::FrameHelper::convertToCvMat( tmpFrame );

    // project image in homography
    hom.addImage( img, Eigen::Isometry3f( cam2plane.matrix().cast<float>() ), camMatrix );

    addedCams.insert( id );

    checkComplete();
}

void VirtualView::cam1TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam1_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam1_sample, 1 );
}

void VirtualView::cam2TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam2_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam22plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam2_sample, 2 );
}

void VirtualView::cam3TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam3_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam32plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam3_sample, 3 );
}

void VirtualView::cam4TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam4_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam42plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam4_sample, 4 );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VirtualView.hpp for more detailed
// documentation about them.

bool VirtualView::configureHook()
{
    if (! VirtualViewBase::configureHook())
        return false;

    Eigen::Affine3d vcam2plane;
    if( _virtual_cam2plane.get( base::Time(), vcam2plane ) )
    {
	hom.init( _width.value(), _height.value(), _focal_length.value(), 
		Eigen::Isometry3f( vcam2plane.matrix().cast<float>() ) );
    }
    else
    {
	LOG_ERROR_S << "Could not get vcam2plane transformation chain" << std::endl;
	return false;
    }

    viewFrame.reset( new base::samples::frame::Frame( _width.value(), _height.value(), 8, base::samples::frame::MODE_RGB ) );

    return true;
}
bool VirtualView::startHook()
{
    if (! VirtualViewBase::startHook())
        return false;
    return true;
}
void VirtualView::updateHook()
{
    VirtualViewBase::updateHook();
}
void VirtualView::errorHook()
{
    VirtualViewBase::errorHook();
}
void VirtualView::stopHook()
{
    VirtualViewBase::stopHook();
}
void VirtualView::cleanupHook()
{
    VirtualViewBase::cleanupHook();

    viewFrame.reset(0);
}
