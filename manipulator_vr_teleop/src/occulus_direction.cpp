#include<rclcpp/rclcpp.hpp>
#include<unity_robotics_demo_msgs/msg/pos_rot.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

class QuestNode: public rclcpp::Node
{
    public:
        QuestNode(const std::string &node_name, const rclcpp::NodeOptions &options=rclcpp::NodeOptions()): Node(node_name, options){

            RCLCPP_INFO(rclcpp::get_logger(node_name), "QuestNode intialize");
            // Declare publisher
            occulus_pose_sub_ = this->create_subscription<unity_robotics_demo_msgs::msg::PosRot>("pos_rot", 10, std::bind(&QuestNode::topic_callback, this, _1));
            // TF broadcaster
            tf_broadcaster_= std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        void topic_callback(const unity_robotics_demo_msgs::msg::PosRot & msg) {
            x_ref = x_origin + msg.pos_x;
            y_ref = y_origin + msg.pos_y;
            z_ref = z_origin + msg.pos_z;
            // Eigen::Quaterniond q(msg.rot_w, msg.rot_x, msg.rot_y, msg.rot_z);
            // double angle = M_PI / 2.0;
            // Eigen::Vector3d axis(0, 1, 0);
            // Eigen::Matrix3d R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
            // q = q * R;
            // q.normalize();
            Eigen::Quaterniond q_oculus(msg.rot_w, msg.rot_y, msg.rot_z, msg.rot_x);
            
            q_x = q_oculus.coeffs().x();
            q_y = q_oculus.coeffs().y();
            q_z = q_oculus.coeffs().z();
            q_w = q_oculus.coeffs().w();
            // RCLCPP_INFO(this->get_logger(), "x: %f y: %f z: %f w: %f",q_x,q_y,q_z,q_w);
            // q_x = msg.rot_x;
            // q_y = msg.rot_y;
            // q_z = msg.rot_z;
            // q_w = msg.rot_w;
            geometry_msgs::msg::TransformStamped t;

            // Read message content and assign it to
            // corresponding tf variables
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "base_link";

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            t.transform.rotation.x = q_x;
            t.transform.rotation.y = q_y;
            t.transform.rotation.z = q_z;
            t.transform.rotation.w = q_w;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }

    private:
        rclcpp::Subscription<unity_robotics_demo_msgs::msg::PosRot>::SharedPtr occulus_pose_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
    std::shared_ptr<QuestNode> node = std::make_shared<QuestNode>("direction_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}