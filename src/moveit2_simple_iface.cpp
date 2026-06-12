/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : moveit2_simple_iface.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_simple_iface.hpp"

m2SimpleIface::m2SimpleIface(const rclcpp::NodeOptions &options)
    : Node("moveit2_simple_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_simple_iface_node")), 
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper(node_) 
{   
    // NOTE: use_sim_time is now passed via launch parameters, not hardcoded
    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    this->get_parameter("config_path", config_path);
    this->get_parameter("enable_servo", enable_servo);
    this->get_parameter("dt", dt); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // TODO: Add as reconfigurable param 
    std::chrono::duration<double> SYSTEM_DT(dt);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2SimpleIface::run, this));

    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
    config              = init_config(config_path);  
    PLANNING_GROUP      = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME        = config["robot"]["ee_link_name"].as<std::string>();
    ROBOT_DESC          = config["robot"]["robot_desc"].as<std::string>();  
    PLANNING_FRAME      = config["robot"]["planning_frame"].as<std::string>(); 
    PLANNING_SCENE      = config["robot"]["planning_scene"].as<std::string>(); 
    MOVE_GROUP_NS       = config["robot"]["move_group_ns"].as<std::string>(); 
    NUM_CART_PTS        = config["robot"]["num_cart_pts"].as<int>(); 
    JOINT_STATES        = config["robot"]["joint_states"].as<std::string>(); 
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    INIT_VEL_SCALING    = 0.05;
    INIT_ACC_SCALING    = 0.05;
    eager_execution     = true;

    // Currently not used :) [ns]
    ns_ = this->get_namespace(); 	
    init_publishers(); 
    init_subscribers(); 
    init_services(); 
    init_moveit(); 
    if (enable_servo) {
        servoPtr = init_servo();
        RCLCPP_INFO(this->get_logger(), servoPtr ? "Servo initialized successfully." : "Servo init returned null!");
    } else {
        RCLCPP_WARN(this->get_logger(), "enable_servo is false — servoPtr will be null, SERVO_CTL state will not work.");
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 

    // Init anything for the old pose because it is non-existent at the beggining
    m_oldPoseCmd.pose.position.x = 5.0; 
    nodeInit = true; 

}

YAML::Node m2SimpleIface::init_config(std::string yaml_path)
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path); 
    return YAML::LoadFile(yaml_path);
}

void m2SimpleIface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1); 
    auto current_robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(ns_ + current_robot_state_name, 1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2SimpleIface::init_subscribers()
{
    auto pose_cmd_name = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
    auto cart_traj_cmd_name = config["topic"]["sub"]["cmd_traj"]["name"].as<std::string>(); 
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + pose_cmd_name, 1, std::bind(&m2SimpleIface::pose_cmd_cb, this, _1));
    ctraj_cmd_sub_ = this->create_subscription<arm_api2_msgs::msg::CartesianWaypoints>(ns_ + cart_traj_cmd_name, 1, std::bind(&m2SimpleIface::cart_poses_cb, this, _1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(ns_ + joint_states_name, 1, std::bind(&m2SimpleIface::joint_state_cb, this, _1));
    servo_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/servo_twist_cmd", 10, std::bind(&m2SimpleIface::servo_twist_cb, this, _1));
    servo_status_pub_ = this->create_publisher<moveit_msgs::msg::ServoStatus>("~/servo_status", 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!");
}

void m2SimpleIface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name  = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_planner_name  = config["srv"]["set_planner"]["name"].as<std::string>();
    auto open_gripper_name = config["srv"]["open_gripper"]["name"].as<std::string>(); 
    auto close_gripper_name= config["srv"]["close_gripper"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, std::bind(&m2SimpleIface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_  = this->create_service<arm_api2_msgs::srv::SetVelAcc>(ns_ + set_vel_acc_name, std::bind(&m2SimpleIface::set_vel_acc_cb, this, _1, _2));
    set_planner_srv_  = this->create_service<arm_api2_msgs::srv::SetStringParam>(ns_ + set_planner_name, std::bind(&m2SimpleIface::set_planner_cb, this, _1, _2));
    open_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + open_gripper_name, std::bind(&m2SimpleIface::open_gripper_cb, this, _1, _2));
    close_gripper_srv_ = this->create_service<std_srvs::srv::Trigger>(ns_ + close_gripper_name, std::bind(&m2SimpleIface::close_gripper_cb, this, _1, _2));
    add_collision_object_srv_ = this->create_service<arm_api2_msgs::srv::AddCollisionObject>(ns_ + "add_collision_object", std::bind(&m2SimpleIface::add_collision_object_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2SimpleIface::init_moveit()
{

    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC); 
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME); 
    RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);  
    // MoveIt related things!
    moveGroupInit       = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS);
    pSceneMonitorInit   = setPlanningSceneMonitor(node_, ROBOT_DESC);
    robotModelInit      = setRobotModel(node_);
    std::string interface_ns = (MOVE_GROUP_NS == "null") ? "" : MOVE_GROUP_NS;
    m_planningSceneInterface = moveit::planning_interface::PlanningSceneInterface(interface_ns);
    RCLCPP_INFO(this->get_logger(), "PlanningSceneInterface initialized!");
}

// TODO: Try to replace with auto
// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2SimpleIface::init_servo()
{
    servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
    auto servo_params = servo_param_listener_->get_params();
    RCLCPP_INFO(this->get_logger(), "Servo move_group_name: %s  command_out_topic: %s",
        servo_params.move_group_name.c_str(), servo_params.command_out_topic.c_str());
    servo_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        servo_params.command_out_topic, 10);
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener_, m_pSceneMonitorPtr);
    RCLCPP_INFO(this->get_logger(), "Servo initialized — publishing joints to: %s", servo_params.command_out_topic.c_str());
    return servo;
}

void m2SimpleIface::pose_cmd_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
   
    // hardcode this to planning frame to check if it works like that? 
    m_currPoseCmd.header.frame_id = PLANNING_FRAME; 
    m_currPoseCmd.pose = msg->pose;
    if (!utils::comparePose(m_currPoseCmd, m_oldPoseCmd)) recivCmd = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "recivCmd: " << recivCmd); 
}

void m2SimpleIface::cart_poses_cb(const arm_api2_msgs::msg::CartesianWaypoints::SharedPtr msg)
{
    // TODO: Maybe implement same check for as the pose_cmd
    m_cartesianWaypoints = msg->poses; 
    recivTraj = true; 
}

void m2SimpleIface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);};
}

void m2SimpleIface::servo_twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    latest_twist_cmd_ = *msg;
    new_twist_cmd_ = true;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
        "servo_twist_cb: lin=[" << msg->twist.linear.x << ", " << msg->twist.linear.y << ", " << msg->twist.linear.z
        << "] ang=[" << msg->twist.angular.x << ", " << msg->twist.angular.y << ", " << msg->twist.angular.z << "]");
}

void m2SimpleIface::processServoCommand()
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "processServoCommand: entered — servoPtr=%s new_twist_cmd_=%s moveGroup=%s",
        servoPtr ? "OK" : "NULL",
        new_twist_cmd_.load() ? "true" : "false",
        m_moveGroupPtr ? "OK" : "NULL");

    if (!servoPtr || !new_twist_cmd_ || !m_moveGroupPtr) return;

    // Ignore commands for 0.5s after entering servo mode to flush buffered inputs
    double time_since_entry = (this->now() - servo_entered_time_).seconds();
    if (time_since_entry < 0.5) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "Servo warmup: %.2fs / 0.50s elapsed, discarding command", time_since_entry);
        new_twist_cmd_ = false;
        return;
    }

    new_twist_cmd_ = false;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Servo twist: lin=[%.4f %.4f %.4f] ang=[%.4f %.4f %.4f] frame=%s",
        latest_twist_cmd_.twist.linear.x, latest_twist_cmd_.twist.linear.y, latest_twist_cmd_.twist.linear.z,
        latest_twist_cmd_.twist.angular.x, latest_twist_cmd_.twist.angular.y, latest_twist_cmd_.twist.angular.z,
        latest_twist_cmd_.header.frame_id.c_str());

    bool all_zero = (std::abs(latest_twist_cmd_.twist.linear.x) < 1e-6 &&
                     std::abs(latest_twist_cmd_.twist.linear.y) < 1e-6 &&
                     std::abs(latest_twist_cmd_.twist.linear.z) < 1e-6 &&
                     std::abs(latest_twist_cmd_.twist.angular.x) < 1e-6 &&
                     std::abs(latest_twist_cmd_.twist.angular.y) < 1e-6 &&
                     std::abs(latest_twist_cmd_.twist.angular.z) < 1e-6);
    if (all_zero) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Servo: zero-velocity command — move a joystick axis to send motion");
        return;
    }

    try {
        moveit_servo::TwistCommand twist_cmd;
        twist_cmd.frame_id = latest_twist_cmd_.header.frame_id.empty() ? EE_LINK_NAME : latest_twist_cmd_.header.frame_id;
        twist_cmd.velocities[0] = latest_twist_cmd_.twist.linear.x;
        twist_cmd.velocities[1] = latest_twist_cmd_.twist.linear.y;
        twist_cmd.velocities[2] = latest_twist_cmd_.twist.linear.z;
        twist_cmd.velocities[3] = latest_twist_cmd_.twist.angular.x;
        twist_cmd.velocities[4] = latest_twist_cmd_.twist.angular.y;
        twist_cmd.velocities[5] = latest_twist_cmd_.twist.angular.z;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Servo: calling getCurrentState...");
        auto current_state = m_moveGroupPtr->getCurrentState(1.0);
        if (!current_state) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo: could not get current robot state");
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Servo: calling getNextJointState...");
        servoPtr->setCommandType(moveit_servo::CommandType::TWIST);
        moveit_servo::KinematicState next_state = servoPtr->getNextJointState(current_state, twist_cmd);

        // Get and publish servo status
        auto status     = servoPtr->getStatus();
        auto status_str = servoPtr->getStatusMessage();
        int8_t status_code = static_cast<int8_t>(status);

        moveit_msgs::msg::ServoStatus status_msg;
        status_msg.code    = status_code;
        status_msg.message = status_str;
        servo_status_pub_->publish(status_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Servo status code=%d: %s", status_code, status_str.c_str());

        if (status == moveit_servo::StatusCode::INVALID ||
            status == moveit_servo::StatusCode::HALT_FOR_SINGULARITY ||
            status == moveit_servo::StatusCode::HALT_FOR_COLLISION ||
            status == moveit_servo::StatusCode::JOINT_BOUND) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Servo halted — not publishing trajectory (code=%d: %s)", status_code, status_str.c_str());
            return;
        }

        if (next_state.joint_names.empty() || next_state.positions.size() == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servo: getNextJointState returned empty state — joints=%zu pos_size=%ld",
                next_state.joint_names.size(), next_state.positions.size());
            return;
        }

        // Build and publish JointTrajectory
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp    = this->now();
        traj_msg.header.frame_id = PLANNING_FRAME;
        traj_msg.joint_names     = next_state.joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (size_t i = 0; i < next_state.positions.size(); ++i) {
            point.positions.push_back(next_state.positions[i]);
            point.velocities.push_back(0.0);  // position-only controller requires zero final velocity
        }
        point.time_from_start = rclcpp::Duration::from_seconds(dt);
        traj_msg.points.push_back(point);

        servo_trajectory_pub_->publish(traj_msg);
        last_servo_state_ = next_state;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Servo: published trajectory for %zu joints to %s (status=%d)",
            next_state.joint_names.size(), servo_trajectory_pub_->get_topic_name(), status_code);

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Servo: processServoCommand exception: %s", e.what());
    }
}

void m2SimpleIface::open_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req; // Unused parameter
    try {
        // Use MoveIt's gripper planning group with named state "open"
        auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, moveit::planning_interface::MoveGroupInterface::Options("gripper", "robot_description", MOVE_GROUP_NS == "null" ? "" : MOVE_GROUP_NS));
        
        gripper_group->setNamedTarget("open");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            gripper_group->execute(plan);
            res->success = true;
            res->message = "Gripper open command executed successfully";
            RCLCPP_INFO_STREAM(this->get_logger(), "Gripper open command executed successfully");
        } else {
            res->success = false;
            res->message = "Failed to plan gripper open motion";
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to plan gripper open motion");
        }
    } catch (const std::exception& e) {
        res->success = false;
        res->message = std::string("Failed to open gripper: ") + e.what();
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open gripper: " << e.what());
    } 
}

void m2SimpleIface::close_gripper_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req; // Unused parameter
    try {
        // Use MoveIt's gripper planning group with named state "close"
        auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, moveit::planning_interface::MoveGroupInterface::Options("gripper", "robot_description", MOVE_GROUP_NS == "null" ? "" : MOVE_GROUP_NS));
        
        gripper_group->setNamedTarget("close");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            gripper_group->execute(plan);
            res->success = true;
            res->message = "Gripper close command executed successfully";
            RCLCPP_INFO_STREAM(this->get_logger(), "Gripper close command executed successfully");
        } else {
            res->success = false;
            res->message = "Failed to plan gripper close motion";
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to plan gripper close motion");
        }
    } catch (const std::exception& e) {
        res->success = false;
        res->message = std::string("Failed to close gripper: ") + e.what();
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to close gripper: " << e.what());
    }
}

void m2SimpleIface::set_vel_acc_cb(const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req, 
                                   const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
{
    if(req->max_vel < 0 || req->max_acc < 0 || req->max_vel > 1 || req->max_acc > 1)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Velocity and acceleration must be in the range [0, 1]!");
        return;
    }
    max_vel_scaling_factor = float(req->max_vel);
    max_acc_scaling_factor = float(req->max_acc);
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set velocity and acceleration to " << max_vel_scaling_factor << " " << max_acc_scaling_factor);
}

void m2SimpleIface::set_planner_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req,
                                   const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
    std::string planner_string = req->value;
    
    // Parse the planner string (format: "planner_id_type", e.g., "pilz_LIN", "ompl_RRT")
    size_t underscore_pos = planner_string.find('_');
    if (underscore_pos == std::string::npos)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid planner format. Expected 'planner_type' (e.g., 'pilz_LIN', 'ompl_RRT')");
        return;
    }
    
    std::string planner_prefix = planner_string.substr(0, underscore_pos);
    std::string planner_type = planner_string.substr(underscore_pos + 1);
    
    // Map short names to full planner IDs
    std::string planner_id;
    if (planner_prefix == "pilz")
    {
        planner_id = "pilz_industrial_motion_planner";
    }
    else if (planner_prefix == "ompl")
    {
        planner_id = "ompl";
    }
    else
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown planner: " << planner_prefix << ". Supported: 'pilz', 'ompl'");
        return;
    }
    
    // Set the planner
    try
    {
        m_moveGroupPtr->setPlanningPipelineId(planner_id);
        m_moveGroupPtr->setPlannerId(planner_type);
        
        current_planner_id_ = planner_id;
        current_planner_type_ = planner_type;
        
        res->success = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Successfully set planner to: " << planner_id << " / " << planner_type);
    }
    catch (const std::exception& e)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set planner: " << e.what());
    }
}

void m2SimpleIface::add_collision_object_cb(const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Request> req,
                                            const std::shared_ptr<arm_api2_msgs::srv::AddCollisionObject::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Adding collision object to planning scene");
    
    // Create a collision object message
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = PLANNING_FRAME;
    collision_object.id = req->id;
    
    // Define primitive based on type
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = req->primitive_type;
    
    // Set dimensions based on type
    if (req->primitive_type == shape_msgs::msg::SolidPrimitive::BOX) {
        if (req->dimensions.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "BOX requires 3 dimensions [x, y, z]");
            res->success = false;
            res->message = "BOX requires 3 dimensions [x, y, z]";
            return;
        }
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = req->dimensions[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = req->dimensions[1];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = req->dimensions[2];
    }
    else if (req->primitive_type == shape_msgs::msg::SolidPrimitive::SPHERE) {
        if (req->dimensions.size() != 1) {
            RCLCPP_ERROR(this->get_logger(), "SPHERE requires 1 dimension [radius]");
            res->success = false;
            res->message = "SPHERE requires 1 dimension [radius]";
            return;
        }
        primitive.dimensions.resize(1);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = req->dimensions[0];
    }
    else if (req->primitive_type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        if (req->dimensions.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "CYLINDER requires 2 dimensions [height, radius]");
            res->success = false;
            res->message = "CYLINDER requires 2 dimensions [height, radius]";
            return;
        }
        primitive.dimensions.resize(2);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = req->dimensions[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = req->dimensions[1];
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported primitive type: %d", req->primitive_type);
        res->success = false;
        res->message = "Unsupported primitive type";
        return;
    }
    
    // Define pose of the object
    geometry_msgs::msg::Pose object_pose;
    object_pose.position = req->position;
    object_pose.orientation = req->orientation;
    
    // If orientation is zero (default), set to identity
    if (object_pose.orientation.w == 0.0 && 
        object_pose.orientation.x == 0.0 && 
        object_pose.orientation.y == 0.0 && 
        object_pose.orientation.z == 0.0) {
        object_pose.orientation.w = 1.0;
    }
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    
    // Add the collision object to the scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    
    m_planningSceneInterface.addCollisionObjects(collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", req->id.c_str());
    res->success = true;
    res->message = "Collision object added successfully";
}

void m2SimpleIface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                                    const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state); 
    
    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        robotState  = (state)wantedIndex_; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
}

bool m2SimpleIface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    // check if moveNs is empty
    if (moveNs == "null") moveNs=""; 

    //https://github.com/moveit/moveit2/issues/496
    m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(nodePtr, 
        moveit::planning_interface::MoveGroupInterface::Options(
            groupName,
            "robot_description",
            moveNs));

    const double POS_TOL = 0.002;   // meters, relaxed for rounded poses
    const double ORI_TOL = 0.01;    // radians (~0.5 deg)
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME);
    m_moveGroupPtr->setGoalPositionTolerance(POS_TOL);
    m_moveGroupPtr->setGoalOrientationTolerance(ORI_TOL);
    m_moveGroupPtr->startStateMonitor();
    m_moveGroupPtr->setMaxVelocityScalingFactor(INIT_VEL_SCALING);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(INIT_ACC_SCALING);
    // executor
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});
    RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!"); 
    return true; 
}

/* This is not neccessary*/
bool m2SimpleIface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
    robot_model_loader::RobotModelLoader robot_model_loader(nodePtr);
    kinematic_model = robot_model_loader.getModel(); 
    // Find nicer way to do this
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    m_robotStatePtr = kinematic_state;
    m_robotStatePtr->setToDefaultValues();
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model frame is: " << kinematic_model->getModelFrame().c_str());
    return true;
}

// TODO: Add service to add collision objects to the scene 
bool m2SimpleIface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
    m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr, name); 
    m_pSceneMonitorPtr->startSceneMonitor(PLANNING_SCENE); 
    if (m_pSceneMonitorPtr->getPlanningScene())
    {
        m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES); 
        m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
        m_pSceneMonitorPtr->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
        m_pSceneMonitorPtr->startSceneMonitor(); 
        m_pSceneMonitorPtr->providePlanningSceneService(); 
    }
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!"); 
        return EXIT_FAILURE; 
    }
    
    //TODO: Check what's difference between planning_Scene and planning_scene_monitor
    RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
    return true; 
}

void m2SimpleIface::execMove(bool async=false)
{   
    geometry_msgs::msg::PoseStamped cmdPose_ = utils::normalizeOrientation(m_currPoseCmd);    
    m_moveGroupPtr->clearPoseTargets(); 
    m_moveGroupPtr->setPoseTarget(cmdPose_.pose, EE_LINK_NAME); 
    RCLCPP_INFO_STREAM(this->get_logger(), "poseTarget is: " << cmdPose_.pose.position.x << " " << cmdPose_.pose.position.y << " " << cmdPose_.pose.position.z); 
    execPlan(async); 
    
    // Thread-safe update of old pose
    {
        std::lock_guard<std::mutex> lock(pose_cmd_mutex_);
        m_oldPoseCmd = cmdPose_;
    }
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Executing commanded path!"); 

}

void m2SimpleIface::execPlan(bool async=false)
{
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = planWithPlanner(plan, eager_execution);

    if (success) {
        if (async) {
            // Store plan as member variable to keep it alive during async execution
            m_async_plan_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
            // Make a shared_ptr copy of the trajectory to ensure it stays alive
            auto trajectory_copy = std::make_shared<moveit_msgs::msg::RobotTrajectory>(m_async_plan_ptr->trajectory);
            RCLCPP_INFO(this->get_logger(), "Starting async execution, plan at: %p, trajectory at: %p", 
                        static_cast<void*>(m_async_plan_ptr.get()), static_cast<void*>(trajectory_copy.get()));
            m_moveGroupPtr->asyncExecute(*trajectory_copy);
            // Keep trajectory_copy alive by storing it
            m_async_trajectory_ptr = trajectory_copy;
        }
        else {
            m_moveGroupPtr->execute(plan);
        }
    }else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!"); 
    }
}

void m2SimpleIface::planExecCartesian(bool async=false)
{   
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    // TODO: Move this method to utils.cpp
    std::vector<geometry_msgs::msg::Pose> cartesianWaypoints = utils::createCartesianWaypoints(m_currPoseState.pose, m_currPoseCmd.pose, NUM_CART_PTS); 
    // TODO: create Cartesian plan, use as first point currentPose 4 now, and as end point use targetPoint 
    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    // plan Cartesian path
    m_moveGroupPtr->computeCartesianPath(cartesianWaypoints, eefStep, jumpThr, trajectory);
    execTrajectory(trajectory, async); 
    m_oldPoseCmd = m_currPoseCmd; 
}

void m2SimpleIface::execCartesian(bool async=false)
{   
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);
    
    // TODO: create Cartesian plan, use as first point currentPose 4 now, and as end point use targetPoint 
    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    // plan Cartesian path
    m_moveGroupPtr->computeCartesianPath(m_cartesianWaypoints, eefStep, jumpThr, trajectory);
    execTrajectory(trajectory, async); 
    m_oldPoseCmd = m_currPoseCmd; 
}

void m2SimpleIface::execTrajectory(moveit_msgs::msg::RobotTrajectory trajectory, bool async=false)
{
    if (async) {
        // Store trajectory as member variable to keep it alive during async execution
        m_async_trajectory_ptr = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
        RCLCPP_INFO(this->get_logger(), "Starting async trajectory execution, storing at: %p", static_cast<void*>(m_async_trajectory_ptr.get()));
        m_moveGroupPtr->asyncExecute(*m_async_trajectory_ptr);
        RCLCPP_INFO_STREAM(this->get_logger(), "Executing trajectory asynchronously!");
    }
    else{
        m_moveGroupPtr->execute(trajectory);
    }
}

void m2SimpleIface::getArmState() 
{   
    if (!m_robotStatePtr) {
        RCLCPP_WARN(this->get_logger(), "Robot state pointer is null!");
        return;
    }
    
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_values;
    m_robotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
    
    // Get current state with timeout to prevent blocking
    m_robotStatePtr = m_moveGroupPtr->getCurrentState(0.1); // 100ms timeout
    if (!m_robotStatePtr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get current state");
        return;
    }
    
    // by default timeout is 10 secs
    m_robotStatePtr->update();
    
    Eigen::Isometry3d currentPose_ = m_robotStatePtr->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    m_currPoseState.header.stamp = this->now();
    m_currPoseState.header.frame_id = PLANNING_FRAME;
}

bool m2SimpleIface::planWithPlanner(moveit::planning_interface::MoveGroupInterface::Plan &plan, bool eagerExecution)
{
    std::vector<std::pair<std::string, std::string>> planners = {
        {"pilz_industrial_motion_planner", "LIN"},
        {"ompl", "EST"},
        {"ompl", "PRM"}
    };
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> all_plans;

    bool success = false;
    int tries_per_planner = 3;
    for (int i = 0; i < int(tries_per_planner * planners.size()); i++) {
        int planner_index = i / tries_per_planner;
        int planner_try = i % tries_per_planner;

        m_moveGroupPtr->setPlanningPipelineId(planners[planner_index].first);
        m_moveGroupPtr->setPlannerId(planners[planner_index].second);

        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        success = static_cast<bool>(m_moveGroupPtr->plan(plan_));

        if (success) {
            RCLCPP_INFO(this->get_logger(), "%s found plan %d with %d points",
                planners[planner_index].second.c_str(), i, int(plan_.trajectory.joint_trajectory.points.size()));
            if (eagerExecution) {
                plan = plan_;
                return true;
            }
            all_plans.push_back(plan_);
        } else {
            RCLCPP_INFO(this->get_logger(), "%s failed to find plan %d",
                planners[planner_index].second.c_str(), i);
        }

        if (planner_try == tries_per_planner - 1 && all_plans.size() >= 3) {
            RCLCPP_INFO(this->get_logger(), "Found %d plans, stopping planning", int(all_plans.size()));
        }
    }

    if (all_plans.empty()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "All planners failed!");
        return false;
    }

    auto best_plan = std::min_element(all_plans.begin(), all_plans.end(), [](auto const& a, auto const& b) {
        return a.trajectory.joint_trajectory.points.size() < b.trajectory.joint_trajectory.points.size();
    });

    plan = *best_plan;
    RCLCPP_INFO(this->get_logger(), "Best plan selected with %d points.", int(best_plan->trajectory.joint_trajectory.points.size()));
    return true;
}

bool m2SimpleIface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);
    robot_state_pub_->publish(utils::stateToMsg(robotState));

    rclcpp::Clock steady_clock; 
    int LOG_STATE_TIMEOUT=10000; 

    // STATE MACHINE
    if (robotState == IDLE)
    {   
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in IDLE mode."); 
    }
    else{
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in " << stateNames[robotState] << " mode."); 
    }

    // Check if servo active, to deactivate before sending to another pose 
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setCollisionChecking(false); servoEntered=false;}

    if (robotState == JOINT_TRAJ_CTL)
    {
       if (recivCmd) {
           execMove(async);
           recivCmd = false;
       }
    }

    if (robotState == CART_TRAJ_CTL)
    {   
        // TODO: Beware if both are true at the same time, shouldn't occur, 
        if (recivCmd) {
            planExecCartesian(async);
            recivCmd = false;
        }

        if (recivTraj) {
            execCartesian(async);
            recivTraj = false;
        }
    }

    if (robotState == SERVO_CTL)
    {
        if (!servoPtr) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "SERVO_CTL: servoPtr is null — servo was not initialized. "
                "Check that enable_servo:=true is passed to the node.");
            return true;
        }
        if (!servoEntered)
        {
            // Clear any buffered twist commands before starting
            latest_twist_cmd_ = geometry_msgs::msg::TwistStamped();
            latest_twist_cmd_.header.frame_id = "base_link";
            new_twist_cmd_ = false;

            try {
                // Prime servo with zero-twist to initialise its internal state
                moveit_servo::TwistCommand zero_twist;
                zero_twist.frame_id = "base_link";
                zero_twist.velocities.fill(0.0);
                auto current_state = m_moveGroupPtr->getCurrentState(1.0);
                if (current_state) {
                    servoPtr->setCommandType(moveit_servo::CommandType::TWIST);
                    servoPtr->getNextJointState(current_state, zero_twist);
                    RCLCPP_INFO(this->get_logger(), "Servo: zero-twist init OK");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Servo: could not get current robot state for zero-twist init");
                }

                // Enable collision checking only if the planning scene is already populated.
                // The CollisionMonitor needs a valid scene — if it isn't ready yet it logs an
                // error and disables itself, so we guard here to avoid a misleading error message.
                if (m_pSceneMonitorPtr && m_pSceneMonitorPtr->getPlanningScene()) {
                    servoPtr->setCollisionChecking(true);
                    RCLCPP_INFO(this->get_logger(), "Servo: collision checking enabled");
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Servo: planning scene not ready yet — collision checking disabled for now. "
                        "It will be enabled once the scene is available.");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Servo: init failed: %s", e.what());
            }

            servo_entered_time_ = this->now();
            servoEntered = true;
            RCLCPP_INFO(this->get_logger(),
                "Servo mode active — publish TwistStamped to ~/servo_twist_cmd");
        }
        processServoCommand();
    }

    return true;     
}



