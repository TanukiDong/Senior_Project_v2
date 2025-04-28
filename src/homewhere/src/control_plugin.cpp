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

        // Retrieve all 8 joints: wheels first, then steering
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

        // Connect to Gazebo update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ControlPlugin::OnUpdate, this));

        // Initialize ROS if needed
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "control_plugin", ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle("control_plugin"));

        // --- Wheel velocity subscriptions ---
        // These specify forward/back linear velocity (geometry_msgs::Twist.linear.x)
        this->velFrontLeftSub  = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_front_left",  1, &ControlPlugin::OnReceived_VelFrontLeft,  this);
        this->velFrontRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_front_right", 1, &ControlPlugin::OnReceived_VelFrontRight, this);
        this->velBackLeftSub   = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_back_left",   1, &ControlPlugin::OnReceived_VelBackLeft,   this);
        this->velBackRightSub  = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_vel_back_right",  1, &ControlPlugin::OnReceived_VelBackRight,  this);

        // --- Steering velocity subscriptions ---
        // These specify steering angular velocity (geometry_msgs::Twist.angular.z)
        this->servoVelFrontLeftSub  = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_servo_vel_front_left",  1, &ControlPlugin::OnReceived_ServoVelFrontLeft,  this);
        this->servoVelFrontRightSub = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_servo_vel_front_right", 1, &ControlPlugin::OnReceived_ServoVelFrontRight, this);
        this->servoVelBackLeftSub   = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_servo_vel_back_left",   1, &ControlPlugin::OnReceived_ServoVelBackLeft,   this);
        this->servoVelBackRightSub  = this->rosNode->subscribe<geometry_msgs::Twist>(
            "/cmd_servo_vel_back_right",  1, &ControlPlugin::OnReceived_ServoVelBackRight,  this);

        ROS_INFO("Control plugin loaded successfully.");
    }


    // --- Callback for wheel drive (front left) ---
    void OnReceived_VelFrontLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontLeft_Linear = msg->linear.x;
    }
    // front right
    void OnReceived_VelFrontRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velFrontRight_Linear = msg->linear.x;
    }
    // back left
    void OnReceived_VelBackLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackLeft_Linear = msg->linear.x;
    }
    // back right
    void OnReceived_VelBackRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->velBackRight_Linear = msg->linear.x;
    }

    // --- Callback for steering velocity (front left) ---
    void OnReceived_ServoVelFrontLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        // We'll only use angular.z to pivot the steer joint
        this->servoVelFrontLeft_ = msg->angular.z;
    }
    // front right
    void OnReceived_ServoVelFrontRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->servoVelFrontRight_ = msg->angular.z;
    }
    // back left
    void OnReceived_ServoVelBackLeft(const geometry_msgs::Twist::ConstPtr& msg) {
        this->servoVelBackLeft_ = msg->angular.z;
    }
    // back right
    void OnReceived_ServoVelBackRight(const geometry_msgs::Twist::ConstPtr& msg) {
        this->servoVelBackRight_ = msg->angular.z;
    }


    void OnUpdate() {
        // Indices:
        //  0 = wheel_front_left_joint
        //  1 = wheel_front_right_joint
        //  2 = wheel_rear_left_joint
        //  3 = wheel_rear_right_joint
        //  4 = steer_front_left_joint
        //  5 = steer_front_right_joint
        //  6 = steer_rear_left_joint
        //  7 = steer_rear_right_joint

        // 1) Drive wheels
        // Optionally set torque limit (fmax)
        // e.g. this->joints[0]->SetParam("fmax", 0, 100.0);
        this->joints[0]->SetVelocity(0, this->velFrontLeft_Linear);
        this->joints[1]->SetVelocity(0, this->velFrontRight_Linear);
        this->joints[2]->SetVelocity(0, this->velBackLeft_Linear);
        this->joints[3]->SetVelocity(0, this->velBackRight_Linear);

        // 2) Steering joints
        // We'll also set velocity around axis 0 (the 1st axis).
        this->joints[4]->SetVelocity(0, this->servoVelFrontLeft_);
        this->joints[5]->SetVelocity(0, this->servoVelFrontRight_);
        this->joints[6]->SetVelocity(0, this->servoVelBackLeft_);
        this->joints[7]->SetVelocity(0, this->servoVelBackRight_);
    }

private:


    physics::ModelPtr model;
    std::array<physics::JointPtr, 8> joints;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    // Subscribers for wheel linear velocity
    ros::Subscriber velFrontLeftSub, velFrontRightSub;
    ros::Subscriber velBackLeftSub,  velBackRightSub;

    // Subscribers for steering angular velocity
    ros::Subscriber servoVelFrontLeftSub, servoVelFrontRightSub;
    ros::Subscriber servoVelBackLeftSub,  servoVelBackRightSub;

    // Drive wheel velocity commands
    double velFrontLeft_Linear   = 0.0;
    double velFrontRight_Linear  = 0.0;
    double velBackLeft_Linear    = 0.0;
    double velBackRight_Linear   = 0.0;

    // Steering angular velocity commands
    double servoVelFrontLeft_    = 0.0;
    double servoVelFrontRight_   = 0.0;
    double servoVelBackLeft_     = 0.0;
    double servoVelBackRight_    = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}  // namespace gazebo