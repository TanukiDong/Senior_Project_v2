#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace gazebo {

class ControlPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
    this->model = _model;

    ROS_INFO_STREAM("Loading control plugin for model: " << _model->GetName());

    const std::array<std::string, 4> joint_names = {
        "wheel_front_left_joint",
        "wheel_front_right_joint",
        "wheel_rear_left_joint",
        "wheel_rear_right_joint"
    };

    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto joint = _model->GetJoint(joint_names[i]);
        if (!joint) {
            ROS_ERROR_STREAM("Failed to find joint: " << joint_names[i]);
            this->wheels[i] = nullptr;
        } else {
            ROS_INFO_STREAM("Successfully found joint: " << joint_names[i]);
            this->wheels[i] = joint;
        }
    }

    for (const auto& wheel : this->wheels) {
        if (!wheel) {
            ROS_ERROR("One or more wheel joints are missing. Plugin will not function correctly.");
            return;
        }
    }

    ROS_INFO("Control plugin loaded successfully.");

    // Connect to Gazebo's update event (called every simulation step)
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ControlPlugin::OnUpdate, this));

    // Ensure ROS is initialized (only once per process)
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "control_plugin", ros::init_options::NoSigintHandler);
    }

    // Create a ROS node handle
    this->rosNode.reset(new ros::NodeHandle("control_plugin"));

    this->velFrontLeftSub = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_front_left", 1, &ControlPlugin::OnReceived_velFrontLeft, this);
    this->velFrontRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_front_right", 1, &ControlPlugin::OnCmdVelReceived_velFrontRight, this);
    this->velBackLeftSub = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_back_left", 1, &ControlPlugin::OnCmdVelReceived_velBackLeft, this);
    this->velBackRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_back_right", 1, &ControlPlugin::OnCmdVelReceived_velBackRight, this);

    // Log success message for debugging
    gzlog << "Control Plugin loaded.\n";
}

    // Handle incoming velocity commands for the left wheel
    void OnReceived_velFrontLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontLeft = msg->linear.x;
        // this->velFrontLeft = msg->angular.z;
    }

    void OnCmdVelReceived_velFrontRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontRight = msg->linear.x;
        // this->velFrontRight = msg->angular.z;
    }

    void OnCmdVelReceived_velBackLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackLeft = msg->linear.x;
        // this->velBackLeft = msg->angular.z;
    }

    void OnCmdVelReceived_velBackRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackRight = msg->linear.x;
        // this->velBackRight = msg->angular.z;
    }

    // Function that Gazebo calls every simulation update
    void OnUpdate() {
        // Apply velocity to each wheel joint based on the received ROS messages
        this->wheels[0]->SetVelocity(0, this->velFrontLeft);
        this->wheels[1]->SetVelocity(0, this->velFrontRight);
        this->wheels[2]->SetVelocity(0, this->velBackLeft);
        this->wheels[3]->SetVelocity(0, this->velBackRight);
    }

private:
    // Gazebo components
    physics::ModelPtr model;
    std::array<physics::JointPtr, 4> wheels;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber velFrontLeftSub, velFrontRightSub, velBackLeftSub, velBackRightSub;


    // Wheel velocities
    double velFrontLeft = 0.0;
    double velFrontRight = 0.0;
    double velBackLeft = 0.0;
    double velBackRight = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}  // namespace gazebo