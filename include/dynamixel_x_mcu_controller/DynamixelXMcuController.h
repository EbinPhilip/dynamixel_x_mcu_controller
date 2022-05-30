#ifndef __DYNAMIXEL_X_MCU_CONTROLLER_H__
#define __DYNAMIXEL_X_MCU_CONTROLLER_H__

#include <configurable_control_hw/Actuator_Controller.h>
#include "dynamixel_x_controller/Dynamixel_X_Actuator_Properties.h"
#include "dynamixel_x_mcu_controller/DriveActuatorsRequest.h"
#include "dynamixel_x_mcu_controller/DriveActuatorsResponse.h"

#include <ros/ros.h>

#include <map>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

namespace DynamixelXMcu
{

    class Controller : public Actuator_Controller
    {
    public:
        Controller(const std::string &controller_name, uint16_t baud, double publish_hz,
                                bool &stop_flag);
        ~Controller();

        void setActuator(Dynamixel_X::Actuator_Properties_Ptr actuator);

        virtual void readState() override;
        virtual void writeCommand() override;

        virtual void enableActuators() override;
        virtual void disableActuators() override;

        virtual bool getErrorDetails(std::string &error_msg) override;

        virtual ::Actuator_Properties_Ptr getActuator(const std::string &) override;
        virtual void getActuatorNames(std::vector<std::string> &) override;

        Dynamixel_X::Actuator_Properties_Ptr getActuator();

    protected:
        void _performRW();

        std::string controller_name_;

        uint16_t baud_rate_;
        double publish_hz_;

        bool error_status_;
        std::string error_msg_;
        bool &stop_flag_;

        bool actuator_enabled_;
        
        std::thread rw_thread_;
        bool stop_rw_;

        ros::NodeHandle nh_;
        ros::AsyncSpinner spinner_;

        dynamixel_x_mcu_controller::DriveActuatorsRequest request_;
        dynamixel_x_mcu_controller::DriveActuatorsResponse response_;

        ros::ServiceClient initialize_service_;
        bool initialized_;
        ros::ServiceClient rw_service_;
        ros::ServiceClient enable_service_;

        std::mutex read_mutex_;
        std::mutex write_mutex_;

        Dynamixel_X::Actuator_Properties_Ptr actuator_;
    };
}
#endif