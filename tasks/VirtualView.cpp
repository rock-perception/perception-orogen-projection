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


void VirtualView::addCam( const base::Affine3d& cam2plane, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame >& frame )
{
    // get calibration matrix
    frame_helper::CameraCalibration calib = 
	frame_helper::CameraCalibration::fromFrame( *frame );

    if( !calib.isValid() )
	throw std::runtime_error("No valid calibration matrix embedded in frame");

    Eigen::Matrix3f camMatrix = 
	calib.getCameraMatrix();

    // get cv image
    cv::Mat img = frame_helper::FrameHelper::convertToCvMat( *frame );

    // project image in homography
    hom.addImage( img, Eigen::Isometry3f( cam2plane.matrix().cast<float>() ), camMatrix );
}

void VirtualView::cam1TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam1_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam1_sample );

    // TODO for now, sync on cam1, but do something smarter later
    frame_helper::FrameHelper::copyMatToFrame( hom.getVirtualImage(), viewFrame );
    _virtual_cam.write( &viewFrame );
    hom.clearVirtualImage();
}

void VirtualView::cam2TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam2_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam2_sample );
}

void VirtualView::cam3TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam3_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam3_sample );
}

void VirtualView::cam4TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam4_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( ts, cam2plane ) )
	addCam( cam2plane, cam4_sample );
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
}
