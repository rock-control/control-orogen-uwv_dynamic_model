/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityAggregator.hpp"

using namespace uwv_dynamic_model;

VelocityAggregator::VelocityAggregator(std::string const& name)
    : VelocityAggregatorBase(name)
{
}

VelocityAggregator::VelocityAggregator(std::string const& name, RTT::ExecutionEngine* engine)
    : VelocityAggregatorBase(name, engine)
{
    last_vel_state.sourceFrame = "body";
    last_vel_state.targetFrame = "body";
}

VelocityAggregator::~VelocityAggregator()
{
    last_vel_state.sourceFrame = "body";
    last_vel_state.targetFrame = "body";
}

bool VelocityAggregator::getTransformation(const transformer::Transformation &transformer, Eigen::Affine3d &transformationMatrix)
{
    if (!transformer.get(base::Time::now(), transformationMatrix))
    {
        if(state() != MISSING_TRANSFORMATION)
            state(MISSING_TRANSFORMATION);
        return false;
    }
    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityAggregator.hpp for more detailed
// documentation about them.

bool VelocityAggregator::configureHook()
{
    if (! VelocityAggregatorBase::configureHook())
        return false;
    return true;
}
bool VelocityAggregator::startHook()
{
    if (! VelocityAggregatorBase::startHook())
        return false;
    return true;
}
void VelocityAggregator::updateHook()
{
    VelocityAggregatorBase::updateHook();

    base::samples::RigidBodyState dvl_sample;
    if(_dvl_samples.read(dvl_sample) == RTT::NewData)
    {
        if(!dvl_sample.hasValidVelocity())
            return;
        Eigen::Affine3d dvl2body;
        if(!getTransformation(_dvl2body, dvl2body))
            return;
        dvl_sample.velocity = dvl2body.rotation()*dvl_sample.velocity;
        if(last_vel_state.hasValidAngularVelocity())
            dvl_sample.velocity -= last_vel_state.angular_velocity.cross(dvl2body.translation());
        last_vel_state.velocity = dvl_sample.velocity;
        last_vel_state.time = dvl_sample.time;
        _velocity_samples.write(last_vel_state);
    }

    base::samples::RigidBodyState ori_sample;
    if(_orientation_samples.read(ori_sample) == RTT::NewData)
    {
        if(!ori_sample.hasValidOrientation())
            return;
        last_vel_state.orientation = ori_sample.orientation;
    }

    base::samples::IMUSensors imu_sample;
    if(_imu_samples.read(imu_sample) == RTT::NewData)
    {
        Eigen::Affine3d imu2body;
        if(imu_sample.gyro.hasNaN())
            return;
        if(!getTransformation(_imu2body, imu2body))
            return;
        last_vel_state.angular_velocity = imu2body.rotation()*imu_sample.gyro;
    }

}
void VelocityAggregator::errorHook()
{
    VelocityAggregatorBase::errorHook();
}
void VelocityAggregator::stopHook()
{
    VelocityAggregatorBase::stopHook();
}
void VelocityAggregator::cleanupHook()
{
    VelocityAggregatorBase::cleanupHook();
}
