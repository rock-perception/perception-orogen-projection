/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OmnicamEquirectangular.hpp"

using namespace projection;

OmnicamEquirectangular::OmnicamEquirectangular(std::string const& name)
    : OmnicamEquirectangularBase(name)
{
}

OmnicamEquirectangular::OmnicamEquirectangular(std::string const& name, RTT::ExecutionEngine* engine)
    : OmnicamEquirectangularBase(name, engine)
{
}

OmnicamEquirectangular::~OmnicamEquirectangular()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OmnicamEquirectangular.hpp for more detailed
// documentation about them.

bool OmnicamEquirectangular::configureHook()
{
    if (! OmnicamEquirectangularBase::configureHook())
        return false;

    equiProjection.init( _width.value(), _omnicam_calibration.value() );
    viewFrame.reset( new base::samples::frame::Frame( _width.value(), _width.value() * 0.5, 8, base::samples::frame::MODE_RGB ) );

    return true;
}
bool OmnicamEquirectangular::startHook()
{
    if (! OmnicamEquirectangularBase::startHook())
        return false;
    return true;
}
void OmnicamEquirectangular::updateHook()
{
    OmnicamEquirectangularBase::updateHook();

    // read input frame
    _omnicam.read( omniFrame );

    if( omniFrame.valid() )
    {
        // convert to target colorspace and apply undistort
        tmpFrame.init( omniFrame->size.width, omniFrame->size.height, 8, base::samples::frame::MODE_RGB );
        frameHelper.convert( *omniFrame, tmpFrame, 0, 0, frame_helper::INTER_LINEAR );

        // get cv image
        cv::Mat img = frame_helper::FrameHelper::convertToCvMat( tmpFrame );

        // perform the reprojection
        equiProjection.process( img );

        // write image back
        base::samples::frame::Frame* frame_ptr = viewFrame.write_access();
        cv::Mat viewMat = frame_helper::FrameHelper::convertToCvMat( *frame_ptr );
        equiProjection.getView().copyTo( viewMat );
        frame_ptr->setStatus(base::samples::frame::STATUS_VALID);
        viewFrame.reset( frame_ptr );

        _view.write( viewFrame );
    }
}
void OmnicamEquirectangular::errorHook()
{
    OmnicamEquirectangularBase::errorHook();
}
void OmnicamEquirectangular::stopHook()
{
    OmnicamEquirectangularBase::stopHook();
}
void OmnicamEquirectangular::cleanupHook()
{
    OmnicamEquirectangularBase::cleanupHook();
}
