#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <thread>
#include <vector>

#include "io_context/io_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"


class SerialPortNode : public rclcpp::Node
{

public:
    explicit SerialPortNode(const rclcpp::NodeOptions& options)
        : Node("serial_example",options),
          owned_ctx_ {new IoContext(2)},
          serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {          
        try{
            param();

            if (serial_port_config_.get() == nullptr) {
                std::cout << "指针为空" << std::endl;
            } else {
                std::cout << "指针不为空" << std::endl;
            }

            RCLCPP_INFO(this->get_logger(),"Current device name is %s %d %d %d %d %lu"
                                                                    ,device_name_.c_str()
                                                                    ,serial_port_config_->get_baud_rate() 
                                                                    ,serial_port_config_->get_flow_control() == FlowControl::NONE
                                                                    ,serial_port_config_->get_parity() == Parity::NONE
                                                                    ,serial_port_config_->get_stop_bits() == StopBits::ONE
                                                                    ,sizeof(serial_port_config_));

            RCLCPP_INFO(this->get_logger(),"name = %s", device_name_.c_str());

            serial_driver_->init_port(device_name_, *serial_port_config_);
         

            if (!serial_driver_->port()->is_open()) {
                serial_driver_->port()->open();
   
                demo_serial_start();
            }
        }catch(const std::exception& ex){
   

            RCLCPP_ERROR(this->get_logger(),"Error creating serail port %s - %s", device_name_.c_str(), ex.what());
            throw ex;
        }
    }
    
    ~SerialPortNode(){
        if (demo_thread.joinable()) {
            demo_thread.join();
        }

        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
         }

        if (owned_ctx_) {
            owned_ctx_->waitForExit();
         }
    }
    
    void demo_serial_start(){
     demo_thread = std::thread(&SerialPortNode::readSerialDemo,this);
     RCLCPP_INFO(this->get_logger(),"demo serial thread started!");
    }

    void readSerialDemo(){
       std::vector<uint8_t> buffer(1);
        
       while(rclcpp::ok()){
        serial_driver_->port()->receive(buffer);
        RCLCPP_INFO(this->get_logger(),"Current buffer content %d",buffer[0]);
       }
    }

    void param(){

        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;
        
        try{
            device_name_ = this->declare_parameter<std::string>("device_name","");
        }catch(rclcpp::ParameterTypeException& ex){
            RCLCPP_ERROR(this->get_logger(),"The device name provided was invalid");
            throw ex;
        }

        try{
            baud_rate = this->declare_parameter<int>("baud_rate",0);
        }catch(rclcpp::ParameterTypeException& ex){
            RCLCPP_ERROR(this->get_logger(),"The baud rate provided was incalid");
            throw ex;
        }     


        serial_port_config_ = 
            std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate,fc,pt,sb);
    }
    
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    std::unique_ptr<IoContext> owned_ctx_;

    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_ = nullptr;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> serial_port_config_ = nullptr;
    rclcpp::TimerBase::SharedPtr read_timer_;

    uint32_t baud_rate;
    std::string device_name_;

    std::thread demo_thread;
};


#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(SerialPortNode);