/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Crobotic Solutions d.o.o.
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

/*      Title       : arm_joy.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Joystick control code.
 */

#include "arm_api2/arm_joy.hpp"

// ============================================================
// Joystick layout — change the 6 axis indices here
// Run `ros2 topic echo /joy --field axes --once` to inspect
// ============================================================
static constexpr int AXIS_LIN_X = 0;  // left  stick X  → linear.x
static constexpr int AXIS_LIN_Y = 1;  // left  stick Y  → linear.y
static constexpr int AXIS_LIN_Z = 4;  // right stick Y  → linear.z
static constexpr int AXIS_ANG_X = 0;  // left  stick X  → angular.x (roll)
static constexpr int AXIS_ANG_Y = 1;  // left  stick Y  → angular.y (pitch)
static constexpr int AXIS_ANG_Z = 3;  // right stick X  → angular.z (yaw)
// ============================================================
static constexpr int   BTN_POSITION     = 5;    // R1  — hold for position mode
static constexpr int   BTN_ORIENTATION  = 4;    // L1  — hold for orientation mode
static constexpr int   AXIS_OPEN_GRIPPER  = 5;    // RT (axes[5]) — hold to open
static constexpr int   AXIS_CLOSE_GRIPPER = 2;    // LT (axes[2]) — hold to close
static constexpr int   AXIS_DPAD_Y       = 7;     // D-pad up/down → speed scale
static constexpr float C_LIN             = 0.3f;  // max linear  velocity [m/s]
static constexpr float C_ANG             = 0.2f;  // max angular velocity [rad/s]
static constexpr float DEADBAND          = 0.05f;
static constexpr float GRIPPER_MAX       = 0.035f; // half-gap at full open [m] (slider max 70mm / 2)
static constexpr float GRIPPER_STEP      = 3.5e-4f; // increment per 20ms tick (~2s full travel)

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

    setScaleFactor(1); 
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));

    enableJoy_ = true; 


}

void JoyCtl::init()
{
    // Allow overriding the servo twist topic at launch time, e.g.:
    //   ros2 run arm_api2 joy_ctl --ros-args -p servo_twist_topic:=/moveit2_simple_iface/servo_twist_cmd
    this->declare_parameter<std::string>("servo_twist_topic", "/moveit2_iface_node/delta_twist_cmds");
    std::string servo_twist_topic = this->get_parameter("servo_twist_topic").as_string();

    cmdVelPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_twist_topic, 1);
    joySub_    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyCtl::joy_callback, this, _1));
    publish_timer_ = this->create_wall_timer(20ms, std::bind(&JoyCtl::publish_timer_cb, this));

    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/gripper_controller/joint_trajectory", 1);

    RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl — publishing twist to: %s", servo_twist_topic.c_str());
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    const std::vector<float>& axes = msg->axes;

    bool pos_mode = msg->buttons.at(BTN_POSITION)    == 1;
    bool ori_mode = msg->buttons.at(BTN_ORIENTATION) == 1;

    int LOG_T = 5000;
    if (pos_mode)       RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_T, "Position mode");
    else if (ori_mode)  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_T, "Orientation mode");

    float sF = getScaleFactor();
    if (axes.at(AXIS_DPAD_Y) ==  1.0f && sF < 100) { sF += 1; RCLCPP_INFO_STREAM(this->get_logger(), "Scale: " << sF); }
    if (axes.at(AXIS_DPAD_Y) == -1.0f && sF >   1) { sF -= 1; RCLCPP_INFO_STREAM(this->get_logger(), "Scale: " << sF); }
    setScaleFactor(sF);

    auto db = [](float v){ return std::abs(v) < DEADBAND ? 0.0f : v; };

    last_twist_msg_.header.frame_id = "link6";
    last_twist_msg_.twist = geometry_msgs::msg::Twist();

    if (pos_mode) {
        last_twist_msg_.twist.linear.x  =  -db(axes.at(AXIS_LIN_Z)) * sF * C_LIN;
        last_twist_msg_.twist.linear.y  =  db(axes.at(AXIS_LIN_X)) * sF * C_LIN;
        last_twist_msg_.twist.linear.z  =  db(axes.at(AXIS_LIN_Y)) * sF * C_LIN;
    } else if (ori_mode) {
        last_twist_msg_.twist.angular.x =  db(axes.at(AXIS_ANG_Z)) * sF * C_ANG;
        last_twist_msg_.twist.angular.y =  db(axes.at(AXIS_ANG_Y)) * sF * C_ANG;
        last_twist_msg_.twist.angular.z =  db(axes.at(AXIS_ANG_X)) * sF * C_ANG;
    }

    // Track gripper trigger state — axes rest at 1.0, pressed toward -1.0
    // amount = (1 - axis) / 2  →  0.0 at rest, 1.0 fully pressed
    trig_open_held_  = axes.at(AXIS_OPEN_GRIPPER)  < 0.9f;
    trig_close_held_ = axes.at(AXIS_CLOSE_GRIPPER) < 0.9f;
}

void JoyCtl::publish_timer_cb()
{
    last_twist_msg_.header.stamp = this->get_clock()->now();
    cmdVelPub_->publish(last_twist_msg_);

    // Gripper — accumulate position at 50Hz while button held, then publish JTC
    if (trig_open_held_)  gripper_pos_ = std::min(gripper_pos_ + GRIPPER_STEP, GRIPPER_MAX);
    if (trig_close_held_) gripper_pos_ = std::max(gripper_pos_ - GRIPPER_STEP, 0.0f);

    trajectory_msgs::msg::JointTrajectory gtraj;
    gtraj.joint_names = {"joint7", "joint8"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {gripper_pos_, -gripper_pos_};
    pt.time_from_start = rclcpp::Duration::from_seconds(0.02);
    gtraj.points.push_back(pt);
    gripper_pub_->publish(gtraj);
}

// Methods that set scale factor
void JoyCtl::setScaleFactor(int value)
{
    scale_factor = value; 
}

int JoyCtl::getScaleFactor() const
{
    return scale_factor; 
}

void JoyCtl::setEnableJoy(bool val) 
{
    enableJoy_ = val; 
}

bool JoyCtl::getEnableJoy() const
{
    return enableJoy_; 
}