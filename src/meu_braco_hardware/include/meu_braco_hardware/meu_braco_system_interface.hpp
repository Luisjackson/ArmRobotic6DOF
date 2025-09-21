#ifndef MEU_BRACO_SYSTEM_INTERFACE_HPP
#define MEU_BRACO_SYSTEM_INTERFACE_HPP

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

namespace meu_braco_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MeuBracoSystemInterface : public hardware_interface::SystemInterface
{
public:

    MeuBracoSystemInterface();
    ~MeuBracoSystemInterface();

    // Funcoes de ciclo de vida
    CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // funcoes do ros2 control
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // leitura e escrita
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    LibSerial::SerialPort arduino_serial_;
    std::string serial_port_name_;

    // Vetores para guardar os comandos do PC para o Arduino e os estados do Arduino para o PC
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    std::vector<double> hw_velocities_; 
};
}
#endif