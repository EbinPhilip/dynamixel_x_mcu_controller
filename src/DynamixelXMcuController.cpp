#include "DynamixelXMcuController.h"

#include "dynamixel_x_mcu_controller/InitializeActuators.h"
#include "dynamixel_x_mcu_controller/InitializeActuatorsRequest.h"
#include "dynamixel_x_mcu_controller/InitializeActuatorsResponse.h"

#include "dynamixel_x_mcu_controller/EnableActuators.h"
#include "dynamixel_x_mcu_controller/EnableActuatorsRequest.h"
#include "dynamixel_x_mcu_controller/EnableActuatorsResponse.h"

#include "dynamixel_x_mcu_controller/DriveActuators.h"
#include "dynamixel_x_mcu_controller/DriveActuatorsRequest.h"
#include "dynamixel_x_mcu_controller/DriveActuatorsResponse.h"

#include <rotational_units/Rotational_Units.h>
#include "dynamixel_x_controller/Dynamixel_X_Units.h"

#include <ros/console.h>

#include <stdexcept>
#include <cmath>

template<typename T>
void scopedLockCopy(T& from, T& to, std::mutex& lock)
{
    std::lock_guard<std::mutex> guard(lock);
    to = from;
}

using namespace DynamixelXMcu;
using namespace dynamixel_x_mcu_controller;

Controller::Controller(const std::string &controller_name, uint16_t baud, double publish_hz,
                        bool &stop_flag) :
      controller_name_(controller_name),
      publish_hz_(publish_hz),
      error_status_(false),
      stop_flag_(stop_flag),
      actuator_enabled_(false),
      stop_rw_(false),
      nh_(ros::NodeHandle()),
      spinner_(2),
      initialized_(false)
{
    initialize_service_ = nh_.serviceClient<dynamixel_x_mcu_controller::
        InitializeActuators>("InitializeActuators", true);
    rw_service_ = nh_.serviceClient<dynamixel_x_mcu_controller::
        DriveActuators>("DriveActuators");
    enable_service_ = nh_.serviceClient<dynamixel_x_mcu_controller::
        EnableActuators>("EnableActuators");

    rw_thread_ = std::thread(&Controller::_performRW, this);
}

Controller::~Controller()
{
    try
    {
        disableActuators();
        stop_rw_ = true;
        spinner_.stop();
    }
    catch (...)
    {
        ROS_ERROR("%s: unexpected error in desctructor", controller_name_.c_str());
    }
}

void Controller::setActuator(Dynamixel_X::Actuator_Properties_Ptr actuator)
{
    actuator_ = actuator;

    InitializeActuators init_msg;
    init_msg.request.ccw_limit = (uint16_t) Dynamixel_X::Pos_Unit(
            actuator_->ccw_limit_deg).Value();
    init_msg.request.cw_limit = (uint16_t) Dynamixel_X::Pos_Unit(
            actuator_->cw_limit_deg).Value();
    init_msg.request.id = actuator_->servo_id;
    init_msg.request.zero_pos = (uint16_t) Dynamixel_X::Pos_Unit(
            actuator_->zero_deg).Value();
    
    bool success = initialize_service_.call(init_msg);
    
    if (!success || init_msg.response.status!=0)
    {
        ROS_ERROR("%s: initialize failed", actuator_->actuator_name.c_str());
        stop_flag_ = true;
    }
}

void Controller::readState()
{
    DriveActuatorsResponse response;
    scopedLockCopy(response_, response, read_mutex_);

    uint16_t effort_register = response.effort;

    double effort = 0.0;
    if (effort_register>>15) // if negative, take 2s complement of register value
    {
        effort = ((~effort_register & 0xFFFF) + 1) * -1;
    }
    else
    {
        effort = effort_register;
    }
    actuator_->state.effort = effort/actuator_->max_effort_value;
    Dynamixel_X::Pos_Unit pos(response.position);
    RUnits::Degrees pos_deg = pos;
    RUnits::RPM rpm = Dynamixel_X::Speed_Unit(response.speed);
    
    pos_deg = pos_deg.Value() - actuator_->zero_deg.Value();
    
    double position = static_cast<RUnits::Radians>(pos_deg).Value();
    double speed = static_cast<RUnits::Radians_Per_Sec>(rpm).Value();
    if ((position - actuator_->state.position) < 0)
    {
        speed*=-1;
    }
    actuator_->state.position = position;
    actuator_->state.velocity = speed;
}

void Controller::writeCommand()
{
    if (!actuator_enabled_)
    {
        return;
    }

    RUnits::Degrees pos_deg = RUnits::Radians(actuator_->command.position);
    RUnits::RPM speed_rpm = RUnits::Radians_Per_Sec(fabs(actuator_->command.velocity));
    
    pos_deg = actuator_->zero_deg.Value() + pos_deg.Value();

    Dynamixel_X::Pos_Unit pos = pos_deg;
    Dynamixel_X::Speed_Unit speed = speed_rpm;

    std::lock_guard<std::mutex> write_guard(read_mutex_);
    request_.position = (uint16_t) pos.Value();
    request_.speed = (uint16_t) speed.Value();
}

void Controller::enableActuators()
{
    EnableActuators enable;
    enable.request.enable = true;
    bool success = enable_service_.call(enable);

    if (!success || enable.response.status!=0)
    {
        ROS_ERROR("%s: unexpected error, enable failed", actuator_->actuator_name.c_str());
        stop_flag_ = true;
        error_status_ = true;
        error_msg_ = "enable failed";
    }
    else
    {
        actuator_enabled_ = true;
    }
}

void Controller::disableActuators()
{
    EnableActuators enable;
    enable.request.enable = false;
    bool success = enable_service_.call(enable);

    if (!success || enable.response.status!=0)
    {
        ROS_ERROR("%s: unexpected error, disable failed", actuator_->actuator_name.c_str());
    }
    else
    {
        actuator_enabled_ = false;
    }
}

bool Controller::getErrorDetails(std::string &error_msg)
{
    error_msg.append(error_msg_);
    return error_status_;
}

::Actuator_Properties_Ptr Controller::getActuator(const std::string &name)
{
    return actuator_;
}

void Controller::getActuatorNames(std::vector<std::string> &names)
{
    names.push_back(actuator_->actuator_name);
}

Dynamixel_X::Actuator_Properties_Ptr Controller::getActuator()
{
    return actuator_;
}

void Controller::_performRW()
{
    while (!stop_flag_)
    {
        DriveActuators rw_msg;

        scopedLockCopy(request_, rw_msg.request, write_mutex_);
        if (!rw_service_.call(rw_msg) || rw_msg.response.error != 0)
        {
            stop_flag_ = true;
            error_status_ = true;
            return;
        }
        scopedLockCopy(rw_msg.response, response_, read_mutex_);
        
        ros::Rate rate(publish_hz_);
        rate.sleep();
    }
}



