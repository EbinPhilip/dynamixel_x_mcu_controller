#ifndef __DYNAMIXEL_X_MCU_ACTUATOR_CONFIG_PARSER_H__
#define __DYNAMIXEL_X_MCU_ACTUATOR_CONFIG_PARSER_H__

#include <configurable_control_hw/Actuator_Config_Parser.h>

namespace DynamixelXMcu
{
class ActuatorConfigParser : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
};
}

#endif