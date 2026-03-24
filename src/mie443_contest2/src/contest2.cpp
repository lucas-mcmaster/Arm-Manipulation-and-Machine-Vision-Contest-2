#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── Arm-only FSM states ───────────────────────────────────────────────────────
enum class RobotState {
    PICKUP_OBJECT,
    PLACE_IN_BIN,
    DONE
};

// ── UPDATE BEFORE RUNNING ─────────────────────────────────────────────────────
// Object position in the arm base frame (metres)
const double OBJ_ARM_X = 0.113;
const double OBJ_ARM_Y = 0.003;
const double OBJ_ARM_Z = 0.192;

// AprilTag ID of the bin to place into
const int TEST_TAG_ID = 0;

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Arm isolated test node started");

    // Initialize arm and tag detector
    ArmController arm(node);
    AprilTagDetector tag_detector(node);

    // Arm state variables
    bool objectInArm  = false;
    bool objectPlaced = false;
    int  pickAttempts = 0;
    const int MAX_PICK_ATTEMPTS = 3;

    // Start directly in PICKUP_OBJECT
    RobotState currentState = RobotState::PICKUP_OBJECT;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        switch (currentState)
        {
            case RobotState::PICKUP_OBJECT: {
                bool pickSuccess = false;

                // Step 0: Move up to avoid knocking over cup
                if (!arm.moveToCartesianPose(
                    0.024, -0.192, 0.301,
                    -0.432, -0.467, -0.555, 0.535))
                {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Pre-grasp hover unreachable — retrying");
                    pickAttempts++;
                    break;
                }
                
                // Step 1: Move arm to hover position above object
                if (!arm.moveToCartesianPose(
                    0.131, -0.000, 0.201,
                    -0.001, 0.004, 0.077, 0.997))
                {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Pre-grasp hover unreachable — retrying");
                    pickAttempts++;
                    break;
                }

                // Step 2: Open gripper before descending
                if (!arm.openGripper()) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to open gripper");
                    pickAttempts++;
                    break;
                }

                // Step 3: Descend straight down to object
                else if (!arm.moveToCartesianPose(
                    0.132, 0.000, 0.169,
                    0.002, -0.016, 0.076, 0.997))
                {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Grasp pose unreachable");
                    pickAttempts++;
                    break;
                }
                // Step 4: Close gripper to grip the object
                else if (!arm.closeGripper()) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to close gripper");
                    pickAttempts++;
                    break;
                }

                // Step 5: Lift arm above object
                else if (!arm.moveToCartesianPose(
                    0.132, -0.017, 0.212,
                    0.006, -0.113, 0.003, 0.994))
                {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Post-grasp lift unreachable — retrying");
                    pickAttempts++;
                    break;
                }

                // Step 5b: Translate to front position (keep step 5 orientation)
                else if (!arm.moveToCartesianPose(
                    0.030, -0.136, 0.212,
                    -0.083, -0.094, -0.689, 0.714))
                {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 5b translation failed — keeping object");
                    pickSuccess = true;
                    pickAttempts = 0;
                }
                // Step 6a: Translate to front position (keep step 5 orientation)
                else if (!arm.moveToCartesianPose(
                    0.045, -0.277, 0.038,
                    0.097, 0.112, -0.683, 0.716))
                {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 6a translation failed — keeping object");
                    pickSuccess = true;
                    pickAttempts = 0;
                }
                
                // Step 5b: Translate to front position (keep step 5 orientation)
                else if (!arm.moveToCartesianPose(
                    0.030, -0.136, 0.212,
                    -0.083, -0.094, -0.689, 0.714))
                {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 5b translation failed — keeping object");
                    pickSuccess = true;
                    pickAttempts = 0;
                }
              
                else {
                    pickSuccess = true;
                    pickAttempts = 0;
                }

                if (pickSuccess) {
                    objectInArm = true;
                    RCLCPP_INFO(node->get_logger(), "PICKUP: Object successfully picked up!");
                    currentState = RobotState::DONE;
                } else {
                    pickAttempts++;
                    RCLCPP_WARN(node->get_logger(),
                        "PICKUP: Attempt %d/%d failed.", pickAttempts, MAX_PICK_ATTEMPTS);
                    if (pickAttempts >= MAX_PICK_ATTEMPTS) {
                        RCLCPP_ERROR(node->get_logger(),
                            "PICKUP: Max attempts reached — aborting.");
                        pickAttempts = 0;
                        objectInArm = false;
                        currentState = RobotState::DONE;
                    }
                }

                break;
            }

            case RobotState::PLACE_IN_BIN:
                RCLCPP_INFO(node->get_logger(), "Placing object in bin");
                {
                    int tag_id = TEST_TAG_ID;

                    RCLCPP_INFO(node->get_logger(),
                        "PLACE: Looking for AprilTag ID %d ...", tag_id);

                    // Step 1: Check AprilTag is visible (timeout 2 s)
                    std::vector<int> visible = tag_detector.getVisibleTags({tag_id}, 2000);
                    if (visible.empty()) {
                        RCLCPP_WARN(node->get_logger(),
                            "PLACE: AprilTag %d not visible yet — retrying next loop", tag_id);
                        break;
                    }

                    // Step 2: Get bin pose from AprilTag (in base_link frame)
                    auto bin_pose_opt = tag_detector.getTagPose(tag_id, 2000);
                    if (!bin_pose_opt.has_value()) {
                        RCLCPP_WARN(node->get_logger(),
                            "PLACE: Could not get pose for tag %d — retrying", tag_id);
                        break;
                    }
                    geometry_msgs::msg::Pose bin_pose = bin_pose_opt.value();

                    RCLCPP_INFO(node->get_logger(),
                        "PLACE: Bin localised — camera frame (%.3f, %.3f, %.3f)",
                        bin_pose.position.x,
                        bin_pose.position.y,
                        bin_pose.position.z);

                    // Step 3: Convert AprilTag pose (camera/base_link frame) → arm base frame.
                    //   Camera Z (depth forward) maps to arm X (reach forward).
                    //   Camera X (lateral)       maps to arm -Y.
                    //   Bin top rim is ~5 cm above the tag centre height.
                    double bin_arm_x = bin_pose.position.z - 0.05;
                    double bin_arm_y = -bin_pose.position.x;
                    double bin_arm_z =  bin_pose.position.y + 0.05;

                    bool placeSuccess = false;

                    // Step 4: Pre-place hover — 10 cm above bin opening, gripper pointing down
                    if (!arm.moveToCartesianPose(
                        bin_arm_x, bin_arm_y, bin_arm_z + 0.10,
                        0.0, M_PI / 2.0, 0.0))
                    {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Pre-place hover unreachable — retrying");
                    }
                    // Step 5: Lower object into bin
                    else if (!arm.moveToCartesianPose(
                        bin_arm_x, bin_arm_y, bin_arm_z,
                        0.0, M_PI / 2.0, 0.0))
                    {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Could not lower into bin — retrying");
                    }
                    // Step 6: Open gripper to release object
                    else if (!arm.openGripper()) {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Gripper release failed — retrying");
                    }
                    else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(400));

                        // Step 7: Retract arm up and clear of bin
                        arm.moveToCartesianPose(
                            bin_arm_x, bin_arm_y, bin_arm_z + 0.15,
                            0.0, M_PI / 2.0, 0.0);

                        placeSuccess = true;
                    }

                    if (placeSuccess) {
                        objectPlaced = true;
                        objectInArm  = false;
                        RCLCPP_INFO(node->get_logger(), "PLACE: Object successfully placed in bin!");
                        currentState = RobotState::DONE;
                    } else {
                        break; // retry from top of PLACE_IN_BIN next loop
                    }
                }
                break;

            case RobotState::DONE:
                RCLCPP_INFO_ONCE(node->get_logger(),
                    "Arm test complete. objectInArm=%d objectPlaced=%d",
                    objectInArm, objectPlaced);
                break;

            default:
                RCLCPP_ERROR(node->get_logger(), "CRITICAL ERROR: FSM entered unknown state.");
                currentState = RobotState::DONE;
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(node->get_logger(), "Arm isolated test node shutting down");
    rclcpp::shutdown();
    return 0;
}