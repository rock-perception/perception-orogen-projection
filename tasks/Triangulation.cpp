/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Triangulation.hpp"

using namespace projection;

Triangulation::Triangulation(std::string const& name)
    : TriangulationBase(name)
{
}

Triangulation::Triangulation(std::string const& name, RTT::ExecutionEngine* engine)
    : TriangulationBase(name, engine)
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
    while( _cam1_point.read( p1 ) == RTT::NewData );
    while( _cam2_point.read( p2 ) == RTT::NewData );

    // get homogenous coordinates
    Eigen::Vector3d xl, xr;
    xl << p1, 1;
    xr << p2, 1; 

    // get the transform from a to b
    base::Transform3d trans = _extrinsic_calibration.value().getTransform();
    Eigen::Quaterniond R(trans.rotation());
    Eigen::Vector3d T(trans.translation());

    // now calculate the triangulation by solving a linear system
    Eigen::Matrix<double,2,3> A;
    A << xl, -(R.inverse()*xr), (xl.cross(R.inverse()*xr-T)-T);
    Eigen::Matrix<double,3,1> b;
    b << -1.0*R.inverse()*T;
    Eigen::Vector3d param = A.colPivHouseholderQr().solve(b);
    
    Eigen::Vector3d Vp = param[2]*(xl.cross(R.inverse()*xr-T)-T);
    Eigen::Vector3d Xl; 
    Xl << xl, 1.0;
    Xl *= param[0];

    Eigen::Vector3d X = Xl + 0.5 * Vp;

    // write out the result to a port
    _scene_point.write( X );
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
