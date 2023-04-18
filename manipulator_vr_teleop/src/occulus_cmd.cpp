#include<rclcpp/rclcpp.hpp>
#include <manipulator_vr_teleop_interface/msg/pos_rot.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;


geometry_msgs::msg::TwistStamped::ConstSharedPtr calculateTwistCommand();

class QuestNode: public rclcpp::Node
{
    public:
        QuestNode(const std::string &node_name, const rclcpp::NodeOptions &options=rclcpp::NodeOptions()): Node(node_name, options){

            RCLCPP_INFO(rclcpp::get_logger(node_name), "QuestNode intialize");
            // Declare publisher
            twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);
            occulus_pose_sub_ = this->create_subscription<manipulator_vr_teleop_interface::msg::PosRot>("pos_rot", 10, std::bind(&QuestNode::topic_callback, this, _1));
            timer_ = this->create_wall_timer(10ms, std::bind(&QuestNode::publish_ref_pose, this));
            // TF listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        void topic_callback(const manipulator_vr_teleop_interface::msg::PosRot & msg) {
            x_ref = x_origin + msg.pos_x;
            y_ref = y_origin + msg.pos_y;
            z_ref = z_origin + msg.pos_z;
            q_x = msg.rot_z;
            q_y = msg.rot_x;
            q_z = -msg.rot_y;
            q_w = msg.rot_w;
        }

        void publish_ref_pose() {
            geometry_msgs::msg::TransformStamped t;
            std::string fromFrameRel = "occulus";
            std::string toFrameRel = "base_link";
            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
            try {
            t = tf_buffer_->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
            }
            auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            msg->header.stamp = this->now();
            msg->header.frame_id = "base_link";
            msg->twist.linear.x = p*(x_ref-t.transform.translation.x);
            msg->twist.linear.y = p*(y_ref-t.transform.translation.y);
            msg->twist.linear.z = p*(z_ref-t.transform.translation.z);
            
            Eigen::Quaterniond q_current = Eigen::Quaterniond(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
            Eigen::Quaterniond q_desired = Eigen::Quaterniond(q_w, q_x, q_y, q_z);

            Eigen::Quaterniond q_error = q_desired * q_current.inverse();

            // Convert axis-angle to angular velocity
            Eigen::AngleAxisd axis_angle(q_error);
            // // Cache the angular error, for rotation tolerance checking
            auto angular_error_ = axis_angle.angle();

            double ang_vel_magnitude = p*angular_error_;
            msg->twist.angular.x = ang_vel_magnitude * axis_angle.axis()[0];
            msg->twist.angular.y = ang_vel_magnitude * axis_angle.axis()[1];
            msg->twist.angular.z = ang_vel_magnitude * axis_angle.axis()[2];
            twist_cmd_pub_->publish(std::move(msg));
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
        rclcpp::Subscription<manipulator_vr_teleop_interface::msg::PosRot>::SharedPtr occulus_pose_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        float p = 10.0;
        float x_origin = 0.4;
        float y_origin= 0.0;
        float z_origin = 0.3;
        float x_ref = x_origin;
        float y_ref = y_origin;
        float z_ref = z_origin;
        float q_x = 0.5;
        float q_y = 0.5;
        float q_z = 0.5;
        float q_w = 0.5;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(false);
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<QuestNode> node = std::make_shared<QuestNode>("quest_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}