#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

    void OnReceived_VelFrontLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontLeft_Linear = msg->linear.x;
        this->velFrontLeft_Angular = msg->angular.z;
    }

    void OnReceived_VelFrontRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontRight_Linear = msg->linear.x;
        this->velFrontRight_Angular = msg->angular.z;
    }

    void OnReceived_VelBackLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackLeft_Linear = msg->linear.x;
        this->velBackLeft_Angular = msg->angular.z;
    }

    void OnReceived_VelBackRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackRight_Linear = msg->linear.x;
        this->velBackRight_Angular = msg->angular.z;
    }

    void OnUpdate() {
        // Define swerve steering constraints

        double max_steer_angle = 1.57;  // 90 degrees (Pi/2

        // Apply wheel linear velocities (independent swerve drive)
        this->joints[0]->SetVelocity(0, this->velFrontLeft_Linear);
        this->joints[1]->SetVelocity(0, this->velFrontRight_Linear);
        this->joints[2]->SetVelocity(0, this->velBackLeft_Linear);
        this->joints[3]->SetVelocity(0, this->velBackRight_Linear);

        // Function to handle strict steering limits at ±90 degrees
        auto handleSteering = [&](physics::JointPtr joint, double& steer_velocity) {
            double current_angle = joint->Position(0);

            if (current_angle >= 1.57) {
                joint->SetPosition(0, 1.57);  // Lock at +90 degrees
                joint->SetVelocity(0, 0);  // Stop movement
            } else if (current_angle <= -1.57) {
                joint->SetPosition(0, -1.57);  // Lock at -90 degrees
                joint->SetVelocity(0, 0);  // Stop movement
            } else {
                joint->SetVelocity(0, steer_velocity);
            }
        };

        // Apply independent steering control for each swerve modules
        handleSteering(this->joints[4], this->velFrontLeft_Angular);
        handleSteering(this->joints[5], this->velFrontRight_Angular);
        handleSteering(this->joints[6], this->velBackLeft_Angular);
        handleSteering(this->joints[7], this->velBackRight_Angular);
        }


private:
    physics::ModelPtr model;
    std::array<physics::JointPtr, 8> joints;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber velFrontLeftSub, velFrontRightSub, velBackLeftSub, velBackRightSub;

    double velFrontLeft_Linear, velFrontRight_Linear, velBackLeft_Linear, velBackRight_Linear;
    double velFrontLeft_Angular, velFrontRight_Angular, velBackLeft_Angular, velBackRight_Angular;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}  // namespace gazebo