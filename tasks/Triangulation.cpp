/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Triangulation.hpp"

using namespace projection;

Triangulation::Triangulation(std::string const& name)
    : TriangulationBase(name),
      _valid1(false), _valid2(false)
{
}

Triangulation::Triangulation(std::string const& name, RTT::ExecutionEngine* engine)
    : TriangulationBase(name, engine),
      _valid1(false), _valid2(false)
{
}

Triangulation::~Triangulation()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Triangulation.hpp for more detailed
// documentation about them.

bool Triangulation::configureHook()
{
    if (! TriangulationBase::configureHook())
        return false;
    return true;
}
bool Triangulation::startHook()
{
    if (! TriangulationBase::startHook())
        return false;
    return true;
}
void Triangulation::updateHook()
{
    TriangulationBase::updateHook();

    // get the point from the input
    base::Vector2d p1, p2;
    while( _cam1_point.read( p1 ) == RTT::NewData )
        _valid1 = true;
    while( _cam2_point.read( p2 ) == RTT::NewData )
        _valid2 = true;

    if( _valid1 && _valid2 )
    {
        // setup the triangulation class
        Eigen::Isometry3d trans( _extrinsic_calibration.value().getTransform().matrix() );
        _tri.setTransform( trans );
        _tri.calcScenePoint( p1, p2 );

        // write out the result to a port
        _scene_point.write( _tri.getScenePoint() );
        _error.write( _tri.getError() );
    }
}
void Triangulation::errorHook()
{
    TriangulationBase::errorHook();
}
void Triangulation::stopHook()
{
    TriangulationBase::stopHook();
}
void Triangulation::cleanupHook()
{
    TriangulationBase::cleanupHook();
}
