#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // Change back to Twist
#include <std_msgs/Float64.h>

namespace gazebo {

class ControlPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
        this->model = _model;

        ROS_INFO_STREAM("Loading control plugin for model: " << _model->GetName());

        this->joints = {
            this->model->GetJoint("wheel_front_left_joint"),
            this->model->GetJoint("wheel_front_right_joint"),
            this->model->GetJoint("wheel_rear_left_joint"),
            this->model->GetJoint("wheel_rear_right_joint"),
            this->model->GetJoint("steer_front_left_joint"),
            this->model->GetJoint("steer_front_right_joint"),
            this->model->GetJoint("steer_rear_left_joint"),
            this->model->GetJoint("steer_rear_right_joint")
        };

        for (const auto& joint : this->joints) {
            if (!joint) {
                ROS_ERROR("One or more joints are missing. Plugin will not function correctly.");
                return;
            }
        }

        // Connect to Gazebo's update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ControlPlugin::OnUpdate, this));

        // Ensure ROS is initialized
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "control_plugin", ros::init_options::NoSigintHandler);
        }

        // Create a ROS node handle
        this->rosNode.reset(new ros::NodeHandle("control_plugin"));

        // Subscribe to position commands for steering (std_msgs/Float64)
        this->steerFrontLeftSub = this->rosNode->subscribe<std_msgs::Float64>(
            "/cmd_steer_front_left", 1, &ControlPlugin::OnReceived_SteerFrontLeft, this);
        this->steerFrontRightSub = this->rosNode->subscribe<std_msgs::Float64>(
            "/cmd_steer_front_right", 1, &ControlPlugin::OnReceived_SteerFrontRight, this);
        this->steerBackLeftSub = this->rosNode->subscribe<std_msgs::Float64>(
            "/cmd_steer_back_left", 1, &ControlPlugin::OnReceived_SteerBackLeft, this);
        this->steerBackRightSub = this->rosNode->subscribe<std_msgs::Float64>(
            "/cmd_steer_back_right", 1, &ControlPlugin::OnReceived_SteerBackRight, this);

        // Subscribe to velocity commands for wheel movement (geometry_msgs/Twist)
        this->velFrontLeftSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_front_left", 1, &ControlPlugin::OnReceived_VelFrontLeft, this);
        this->velFrontRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_front_right", 1, &ControlPlugin::OnReceived_VelFrontRight, this);
        this->velBackLeftSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_back_left", 1, &ControlPlugin::OnReceived_VelBackLeft, this);
        this->velBackRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_back_right", 1, &ControlPlugin::OnReceived_VelBackRight, this);

        ROS_INFO("Control plugin loaded successfully.");
    }

    void OnReceived_SteerFrontLeft(const std_msgs::Float64::ConstPtr& msg) {
        this->steerFrontLeft_Position = msg->data;
    }

    void OnReceived_SteerFrontRight(const std_msgs::Float64::ConstPtr& msg) {
        this->steerFrontRight_Position = msg->data;
    }

    void OnReceived_SteerBackLeft(const std_msgs::Float64::ConstPtr& msg) {
        this->steerBackLeft_Position = msg->data;
    }

    void OnReceived_SteerBackRight(const std_msgs::Float64::ConstPtr& msg) {
        this->steerBackRight_Position = msg->data;
    }

    void OnReceived_VelFrontLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontLeft_Linear = msg->linear.x;
        ROS_INFO("Received front left velocity: %f", this->velFrontLeft_Linear);
    }

    void OnReceived_VelFrontRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontRight_Linear = msg->linear.x;
    }

    void OnReceived_VelBackLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackLeft_Linear = msg->linear.x;
    }

    void OnReceived_VelBackRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackRight_Linear = msg->linear.x;
    }

    void OnUpdate() {
        // Apply wheel velocities
        this->joints[0]->SetVelocity(0, this->velFrontLeft_Linear);
        this->joints[1]->SetVelocity(0, this->velFrontRight_Linear);
        this->joints[2]->SetVelocity(0, this->velBackLeft_Linear);
        this->joints[3]->SetVelocity(0, this->velBackRight_Linear);

        // Apply steering positions
        this->joints[4]->SetPosition(0, this->steerFrontLeft_Position);
        this->joints[5]->SetPosition(0, this->steerFrontRight_Position);
        this->joints[6]->SetPosition(0, this->steerBackLeft_Position);
        this->joints[7]->SetPosition(0, this->steerBackRight_Position);
    }

private:
    physics::ModelPtr model;
    std::array<physics::JointPtr, 8> joints;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber velFrontLeftSub, velFrontRightSub, velBackLeftSub, velBackRightSub;
    ros::Subscriber steerFrontLeftSub, steerFrontRightSub, steerBackLeftSub, steerBackRightSub;

    double velFrontLeft_Linear = 0.0, velFrontRight_Linear = 0.0, velBackLeft_Linear = 0.0, velBackRight_Linear = 0.0;
    double steerFrontLeft_Position = 0.0, steerFrontRight_Position = 0.0, steerBackLeft_Position = 0.0, steerBackRight_Position = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}  // namespace gazebo
