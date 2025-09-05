#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>
#include <iostream>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub;
geometry_msgs::msg::PointStamped target_point;
bool received_point = false;

// ---------- /bottle_point 콜백 ----------
void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (msg->point.x > -0.15 && msg->point.x < 0.15 && msg->point.z != 0.0) {
    msg->point.z = msg->point.z * 1.3;
    target_point = *msg;
    received_point = true;
    RCLCPP_INFO(node->get_logger(), "객체 좌표 수신: (%.3f, %.3f, %.3f)", msg->point.x, msg->point.y, msg->point.z);
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("pick_and_place_input");
  auto logger = rclcpp::get_logger("pick_and_place");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm(node, "arm");
  MoveGroupInterface gripper(node, "gripper");

  sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "bottle_point", 10, point_callback);

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // ----------초기 위치 복귀 ----------
  arm.setNamedTarget("home");
  if (arm.move()) {
    RCLCPP_INFO(logger, "암이 홈 위치로 복귀");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(logger, "홈 위치로 이동 실패");
  }

  // ---------- 회전 탐색 루프 ----------
  RCLCPP_INFO(logger, "회전 탐색 시작...");
double angle = 0.0;
const double angle_step = 0.1;
const double wait_sec = 0.001;
const double max_angle = M_PI;  // 180도

bool scanned = false;

// 왼쪽 스캔 (0 → +π)
for (angle = 0.0; angle <= max_angle; angle += angle_step) {
  if (received_point) break;

  std::map<std::string, double> joint_goal;
  joint_goal["joint1"] = angle;
  arm.setJointValueTarget(joint_goal);
  arm.move();

  std::this_thread::sleep_for(std::chrono::duration<double>(wait_sec));
  rclcpp::spin_some(node);
}
scanned = received_point;

if (!scanned) {
  RCLCPP_INFO(logger, "왼쪽 탐색 실패 → 오른쪽 탐색 시도...");

  // 오른쪽 스캔 (+π → -π)
  for (angle = max_angle; angle >= -max_angle; angle -= angle_step) {
    if (received_point) break;

    std::map<std::string, double> joint_goal;
    joint_goal["joint1"] = angle;
    arm.setJointValueTarget(joint_goal);
    arm.move();

    std::this_thread::sleep_for(std::chrono::duration<double>(wait_sec));
    rclcpp::spin_some(node);
  }
  scanned = received_point;
}

if (!scanned) {
  RCLCPP_ERROR(logger, "양쪽 탐색 실패: 객체를 찾지 못했습니다.");
  rclcpp::shutdown();
  return 1;
}

RCLCPP_INFO(logger, "객체 인식됨! 회전 중단.");

  // 카메라 → link5 변환
geometry_msgs::msg::PointStamped link5_frame_point;
try {
  link5_frame_point = tf_buffer.transform(target_point, "link5", tf2::durationFromSec(1.0));
} catch (const tf2::TransformException &ex) {
  RCLCPP_ERROR(logger, "카메라 → link5 좌표 변환 실패: %s", ex.what());
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::shutdown();
  return 1;
}

// link5 → base_link 변환
geometry_msgs::msg::PointStamped base_point;
try {
  base_point = tf_buffer.transform(link5_frame_point, "base_link", tf2::durationFromSec(1.0));
} catch (const tf2::TransformException &ex) {
  RCLCPP_ERROR(logger, "link5 → base_link 좌표 변환 실패: %s", ex.what());
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::shutdown();
  return 1;
}

  double x = base_point.point.x;
  double y = base_point.point.y;
  double z = base_point.point.z;

  // ---------- 목표 위치 설정 및 이동 ----------
  arm.setPositionTarget(x, y, z);
  arm.setGoalPositionTolerance(0.01);
  arm.setPlanningTime(10.0);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "이동: x=%.3f y=%.3f z=%.3f", x, y, z);
    arm.execute(plan);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(logger, "경로 계획 실패!");
    rclcpp::shutdown();
    return 1;
  }

  // ---------- 그리퍼 닫기 ----------
  gripper.setNamedTarget("close");
  if (gripper.move()) {
    RCLCPP_INFO(logger, "그리퍼 닫힘");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(logger, "그리퍼 닫기 실패");
  }

  // ---------- 원위치 복귀 ----------
  arm.setNamedTarget("init");
  if (arm.move()) {
    RCLCPP_INFO(logger, "암이 홈 위치로 복귀");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(logger, "홈 위치로 이동 실패");
  }

  // ---------- 그리퍼 열기 ----------
  gripper.setNamedTarget("open");
  if (gripper.move()) {
    RCLCPP_INFO(logger, "그리퍼 열림");
  } else {
    RCLCPP_ERROR(logger, "그리퍼 열기 실패");
  }

  rclcpp::shutdown();
  return 0;
}
