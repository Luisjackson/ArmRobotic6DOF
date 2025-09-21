#include "meu_braco_hardware/meu_braco_system_interface.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <chrono>

namespace meu_braco_hardware
{

// Funções de conversão
int rad_to_dxl(double rad) { return static_cast<int>((rad + M_PI) * (1023.0 / (2.0 * M_PI))); }
double dxl_to_rad(int dxl) { return (static_cast<double>(dxl) * (2.0 * M_PI) / 1023.0) - M_PI; }
int rad_to_dxl_inverted(double rad) { return static_cast<int>((M_PI - rad) * (1023.0 / (2.0 * M_PI))); }
double dxl_to_rad_inverted(int dxl) { return M_PI - (static_cast<double>(dxl) * (2.0 * M_PI) / 1023.0); }

MeuBracoSystemInterface::MeuBracoSystemInterface() {}
MeuBracoSystemInterface::~MeuBracoSystemInterface() {}

CallbackReturn MeuBracoSystemInterface::on_init(const hardware_interface::HardwareInfo & info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    serial_port_name_ = info_.hardware_parameters["serial_port"];
    return CallbackReturn::SUCCESS;
}

CallbackReturn MeuBracoSystemInterface::on_configure(const rclcpp_lifecycle::State &){
    try {
        arduino_serial_.Open(serial_port_name_);
        arduino_serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("MeuBracoSystemInterface"), "Falha ao abrir a porta serial %s: %s", serial_port_name_.c_str(), e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn MeuBracoSystemInterface::on_cleanup(const rclcpp_lifecycle::State &){
    if (arduino_serial_.IsOpen()) { arduino_serial_.Close(); }
    return CallbackReturn::SUCCESS;
}

CallbackReturn MeuBracoSystemInterface::on_activate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("MeuBracoSystemInterface"), "Ativando hardware... Lendo estado inicial.");
    rclcpp::sleep_for(std::chrono::seconds(1));
    for (int i = 0; i < 5; ++i) { read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0)); }
    for (size_t i = 0; i < hw_states_.size(); ++i) { hw_commands_[i] = hw_states_[i]; }
    RCLCPP_INFO(rclcpp::get_logger("MeuBracoSystemInterface"), "Hardware ativado. O robô deve ficar parado.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MeuBracoSystemInterface::on_deactivate(const rclcpp_lifecycle::State &){ return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::StateInterface> MeuBracoSystemInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, "position", &hw_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_velocities_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MeuBracoSystemInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, "position", &hw_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type MeuBracoSystemInterface::read(const rclcpp::Time &, const rclcpp::Duration &){
    if (!arduino_serial_.IsOpen()) return hardware_interface::return_type::ERROR;
    try {
        std::string response;
        arduino_serial_.ReadLine(response, '\n', 100);
        if (response.rfind("S ", 0) == 0) {
            std::stringstream ss(response.substr(2));
            std::vector<double> new_states;
            double dxl_pos;
            while(ss >> dxl_pos) { new_states.push_back(dxl_pos); }
            if (new_states.size() == 6) {
                for (size_t i = 0; i < hw_states_.size(); ++i) {
                    if (i == 1 || i == 5) { // Inverte ombro (1) e garra (5)
                        hw_states_[i] = dxl_to_rad_inverted(static_cast<int>(new_states[i]));
                    } else {
                        hw_states_[i] = dxl_to_rad(static_cast<int>(new_states[i]));
                    }
                }
            }
        }
    } catch (const LibSerial::ReadTimeout &) {}
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MeuBracoSystemInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
    if (!arduino_serial_.IsOpen()) return hardware_interface::return_type::ERROR;
    std::stringstream command_stream;
    command_stream << "P ";
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
        if (i == 1 || i == 5) { // Inverte ombro (1) e garra (5)
            command_stream << rad_to_dxl_inverted(hw_commands_[i]);
        } else {
            command_stream << rad_to_dxl(hw_commands_[i]);
        }
        if (i < hw_commands_.size() - 1) { command_stream << " "; }
    }
    command_stream << "\n";
    try {
        arduino_serial_.Write(command_stream.str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("MeuBracoSystemInterface"), "Falha ao enviar comando: %s", e.what());
    }
    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    meu_braco_hardware::MeuBracoSystemInterface,
    hardware_interface::SystemInterface
)