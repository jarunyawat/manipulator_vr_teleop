#include<rclcpp/rclcpp.hpp>
#include<std_srvs/srv/set_bool.hpp>
#include<ur_msgs/srv/set_io.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class GripperCMD: public rclcpp::Node
{
private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client;
    // 0 gripper open
    // 1 gripper close
    int gripper_state = 0;
public:
    GripperCMD(const std::string &node_name);
    void gripper_FSM(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

GripperCMD::GripperCMD(const std::string &node_name): Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "QuestNode intialize");
    service = this->create_service<std_srvs::srv::SetBool>("/gripper_service", std::bind(&GripperCMD::gripper_FSM, this, std::placeholders::_1, std::placeholders::_2));
    client = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    RCLCPP_INFO(this->get_logger(), "denergize gripper");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    // denergize gripper
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = request->FUN_SET_DIGITAL_OUT;
    request->pin = request->PIN_DOUT0;
    request->state = request->STATE_TOOL_VOLTAGE_0V;
    auto result = client->async_send_request(request);
    request->fun = request->FUN_SET_DIGITAL_OUT;
    request->pin = request->PIN_DOUT1;
    request->state = request->STATE_TOOL_VOLTAGE_0V;
    result = client->async_send_request(request);
    // open gripper
    request->fun = request->FUN_SET_DIGITAL_OUT;
    request->pin = request->PIN_DOUT1;
    request->state = request->STATE_TOOL_VOLTAGE_24V;
    result = client->async_send_request(request);
}

void GripperCMD::gripper_FSM(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    if(!gripper_state){
        RCLCPP_INFO(this->get_logger(), "gripper close");
        // denergize gripper
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun = request->FUN_SET_DIGITAL_OUT;
        request->pin = request->PIN_DOUT1;
        request->state = request->STATE_TOOL_VOLTAGE_0V;
        auto result = client->async_send_request(request);
        // close gripper
        std::this_thread::sleep_for(50ms);
        request->fun = request->FUN_SET_DIGITAL_OUT;
        request->pin = request->PIN_DOUT0;
        request->state = request->STATE_TOOL_VOLTAGE_24V;
        result = client->async_send_request(request);
        gripper_state = 1;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "gripper open");
        // denergize gripper
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun = request->FUN_SET_DIGITAL_OUT;
        request->pin = request->PIN_DOUT0;
        request->state = request->STATE_TOOL_VOLTAGE_0V;
        auto result = client->async_send_request(request);
        std::this_thread::sleep_for(50ms);
        // open gripper
        request->fun = request->FUN_SET_DIGITAL_OUT;
        request->pin = request->PIN_DOUT1;
        request->state = request->STATE_TOOL_VOLTAGE_24V;
        result = client->async_send_request(request);
        gripper_state = 0;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<GripperCMD> node = std::make_shared<GripperCMD>("gripper_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}