/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OmnicamPlanar.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace projection;

OmnicamPlanar::OmnicamPlanar(std::string const& name)
    : OmnicamPlanarBase(name)
{
}

OmnicamPlanar::OmnicamPlanar(std::string const& name, RTT::ExecutionEngine* engine)
    : OmnicamPlanarBase(name, engine)
{
}

OmnicamPlanar::~OmnicamPlanar()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OmnicamPlanar.hpp for more detailed
// documentation about them.

bool OmnicamPlanar::configureHook()
{
    if (! OmnicamPlanarBase::configureHook())
        return false;

    //projection::omnicam::Model model;
    //model.loadFromFile("/home/jakob/Desktop/calib_results.txt");
    //model.setAngleRange( -43.0 / 180.0 * M_PI, 35 / 180.0 * M_PI );
    //planarProjection.init( _width.value(), _height.value(), model );
    //_omnicam_calibration.value() = model;

    planarProjection.init( _width.value(), _height.value(), _omnicam_calibration.value() );
    viewFrame.reset( new base::samples::frame::Frame( _width.value(), _height.value(), 8, base::samples::frame::MODE_RGB ) );

    return true;
}
bool OmnicamPlanar::startHook()
{
    if (! OmnicamPlanarBase::startHook())
        return false;
    return true;
}
void OmnicamPlanar::updateHook()
{
    OmnicamPlanarBase::updateHook();

    // read input frame
    _omnicam.read( omniFrame );

    // ... and input view, which needs to be set if updated
    projection::omnicam::PlanarViewConfiguration config;
    if( _view_config.read( config ) == RTT::NewData )
        planarProjection.setView( config );

    if( omniFrame.valid() )
    {
        // convert to target colorspace and apply undistort
        tmpFrame.init( omniFrame->size.width, omniFrame->size.height, 8, base::samples::frame::MODE_RGB );
        frameHelper.convert( *omniFrame, tmpFrame, 0, 0, frame_helper::INTER_LINEAR );

        // get cv image
        cv::Mat img = frame_helper::FrameHelper::convertToCvMat( tmpFrame );

        // perform the reprojection
        planarProjection.process( img );

        // write image back
        base::samples::frame::Frame* frame_ptr = viewFrame.write_access();
        cv::Mat viewMat = frame_helper::FrameHelper::convertToCvMat( *frame_ptr );
        planarProjection.getView().copyTo( viewMat );
        viewFrame.reset( frame_ptr );

        _planar_view.write( viewFrame );
    }
}
void OmnicamPlanar::errorHook()
{
    OmnicamPlanarBase::errorHook();
}
void OmnicamPlanar::stopHook()
{
    OmnicamPlanarBase::stopHook();
}
void OmnicamPlanar::cleanupHook()
{
    OmnicamPlanarBase::cleanupHook();
}
