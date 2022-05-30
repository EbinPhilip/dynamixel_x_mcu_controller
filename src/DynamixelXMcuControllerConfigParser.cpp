#include <pluginlib/class_list_macros.h>

#include "DynamixelXMcuControllerConfigParser.h"
#include <configurable_control_hw/Actuator_Controller.h>
#include "DynamixelXMcuController.h"

#include <memory>
#include <stdexcept>
#include <algorithm>

using namespace XmlRpc;
using namespace DynamixelXMcu;

void ControllerConfigParser::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
     if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("controller config parsing failed!");
    }
    
    for(auto it = config.begin(); it != config.end(); ++it)
    {
        if (it->second.getType() != XmlRpcValue::Type::TypeStruct)
        {
            throw std::runtime_error("actuator config parsing failed!");
        }
    
        std::string controller_name = it->first;
        uint16_t controller_baud_rate = (uint16_t)static_cast<int>(it->second["baud_rate"]);
        uint16_t publish_hz = static_cast<double>(it->second["publish_hz"]);

        Actuator_Controller_Ptr controller_ptr = std::make_shared<Controller>(controller_name, controller_baud_rate, controller_baud_rate, *stop_flag_ptr_);
        controller_map->insert(std::make_pair(controller_name, controller_ptr));
    }
}

PLUGINLIB_EXPORT_CLASS(DynamixelXMcu::ControllerConfigParser, Actuator_Config_Parser)