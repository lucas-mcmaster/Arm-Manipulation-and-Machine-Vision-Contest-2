#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include "mie443_contest2/navigation.h"
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
#include <geometry_msgs/msg/pose_stamped.hpp>

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
// const int TEST_TAG_ID = 0;

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
    Navigation nav(node);
    ArmController arm(node);
    AprilTagDetector tag_detector(node);
    std::vector<int> candidate_tags={0,1,2,3,4};
    tag_detector.setReferenceFrame("oakd_rgb_camera_optical_frame");

    // Arm state variables
    bool objectInArm  = false;
    bool objectPlaced = false;
    int  pickAttempts  = 0;
    int  placeAttempts = 0;
    const int MAX_PICK_ATTEMPTS  = 3;
    const int MAX_PLACE_ATTEMPTS = 3;

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
                
                // Step 7: Go back inside to clear workspace for next steps
                else if (!arm.moveToCartesianPose(
                    0.030, -0.136, 0.212,
                    -0.083, -0.094, -0.689, 0.714))
                {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 7 translation back into robot failed — keeping object");
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
                    currentState = RobotState::PLACE_IN_BIN;
                } else {
                    pickAttempts++;
                    RCLCPP_WARN(node->get_logger(),
                        "PICKUP: Attempt %d/%d failed.", pickAttempts, MAX_PICK_ATTEMPTS);
                    if (pickAttempts >= MAX_PICK_ATTEMPTS) {
                        RCLCPP_ERROR(node->get_logger(),
                            "PICKUP: Max attempts reached — aborting.");
                        pickAttempts = 0;
                        objectInArm = false;
                        currentState = RobotState::PLACE_IN_BIN;
                    }
                }

                break;
            }

            case RobotState::PLACE_IN_BIN:
                RCLCPP_INFO(node->get_logger(), "Placing object in bin");
                {
                    if (placeAttempts >= MAX_PLACE_ATTEMPTS) {
                        RCLCPP_ERROR(node->get_logger(),
                            "PLACE: Max attempts reached — aborting.");
                        placeAttempts = 0;
                        currentState = RobotState::DONE;
                        break;
                    }

                    RCLCPP_INFO(node->get_logger(),
                        "PLACE: Looking for AprilTag ID...");

                    auto visible_tags = tag_detector.getVisibleTags(candidate_tags);
                    std::optional<geometry_msgs::msg::Pose> selected_pose;
                    int selected_tag_id = -1;
                    if (!visible_tags.empty()) {
                        for (int tag_id : visible_tags) {
                            auto bin_pose = tag_detector.getTagPose(tag_id);
                            if (bin_pose.has_value()) {
                                RCLCPP_INFO(node->get_logger(),
                                    "%s -> tag%d: pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
                                    tag_detector.getReferenceFrame().c_str(), tag_id,
                                    bin_pose->position.x, bin_pose->position.y, bin_pose->position.z,
                                    bin_pose->orientation.x, bin_pose->orientation.y, bin_pose->orientation.z, bin_pose->orientation.w);
                                selected_pose = bin_pose;
                                selected_tag_id = tag_id;
                                break; // use the first valid tag pose
                            }
                        }
                    } else {
                        RCLCPP_INFO(node->get_logger(), "No tags visible");
                    }
                    RCLCPP_INFO(node->get_logger(), "---------------------------------");
                    if (!selected_pose.has_value()) {
                        RCLCPP_WARN(node->get_logger(),
                            "PLACE: No valid tag pose available — retrying.");
                        break;
                    }

                    // ── Manual transform: oakd_rgb_camera_optical_frame → arm_mount ──
                    //
                    // tf2_echo oakd_rgb_camera_optical_frame arm_mount gives T_oakd_arm:
                    //   t = [0.000, 0.018, 0.066]
                    //   q(xyzw) = [0.500, -0.500, 0.500, 0.500]
                    //
                    // doTransform needs the INVERSE (T_arm_oakd) so that:
                    //   pose_in (in oakd) → pose_out (in arm_mount)
                    //
                    // Inverse:  t = [-0.066, 0.0, 0.018]
                    //           q(xyzw) = [-0.500, 0.500, -0.500, 0.500]

                    // const auto& bin_pose = selected_pose.value();

                    // RCLCPP_INFO(node->get_logger(),
                    //     "DEBUG [oakd frame] tag%d: pos(%.4f, %.4f, %.4f)",
                    //     selected_tag_id,
                    //     bin_pose.position.x, bin_pose.position.y, bin_pose.position.z);

                    // geometry_msgs::msg::TransformStamped oakd_to_arm;
                    // oakd_to_arm.header.frame_id = "arm_mount";                        // target
                    // oakd_to_arm.child_frame_id  = "oakd_rgb_camera_optical_frame";    // source
                    // oakd_to_arm.transform.translation.x =  -0.066;
                    // oakd_to_arm.transform.translation.y =   0.0;
                    // oakd_to_arm.transform.translation.z =   0.018;
                    // oakd_to_arm.transform.rotation.x    =  -0.5;
                    // oakd_to_arm.transform.rotation.y    =   0.5;
                    // oakd_to_arm.transform.rotation.z    =  -0.5;
                    // oakd_to_arm.transform.rotation.w    =   0.5;

                    // geometry_msgs::msg::PoseStamped pose_in;
                    // pose_in.header.frame_id = "oakd_rgb_camera_optical_frame";
                    // pose_in.pose = bin_pose;

                    // geometry_msgs::msg::PoseStamped pose_out;
                    // tf2::doTransform(pose_in, pose_out, oakd_to_arm);

                    // double bin_arm_x = pose_out.pose.position.x;
                    // double bin_arm_y = pose_out.pose.position.y;
                    // double bin_arm_z = pose_out.pose.position.z + 0.05;

                    // RCLCPP_INFO(node->get_logger(),
                    //     "DEBUG [arm_mount frame] tag%d: pos(%.4f, %.4f, %.4f) (z includes +0.05 offset)",
                    //     selected_tag_id, bin_arm_x, bin_arm_y, bin_arm_z);


                    bool placeSuccess = false;

                    // Move forward 20cm towards april tag
                    // nav.moveToGoal(goal_x, goal_y, goal_phi)

                    // Hard-coded move forward gripper to prepare drop off
                    if (!arm.moveToCartesianPose(
                        0.024, -0.192, 0.301,
                        -0.432, -0.467, -0.555,0.535))
                    {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Step 3 pre-place hover failed");
                        placeAttempts++;
                        break;
                    }
                    // Open gripper
                    else if (!arm.openGripper()) {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Step 6 gripper release failed");
                        placeAttempts++;
                        break;
                    }
                    else {
                        // Wait for object to drop before moving
                        std::this_thread::sleep_for(std::chrono::milliseconds(400));

                        // Step 8: Retract arm back inside robot
                        if (!arm.moveToCartesianPose(
                            0.030, -0.136, 0.212,
                            -0.083, -0.094, -0.689, 0.714))
                        {
                            RCLCPP_ERROR(node->get_logger(), "PLACE: Step 8 retract inside failed");
                            placeAttempts++;
                            break;
                        }
                        else {
                            placeSuccess = true;
                        }
                    }

                    if (placeSuccess) {
                        objectPlaced = true;
                        objectInArm  = false;
                        placeAttempts = 0;
                        RCLCPP_INFO(node->get_logger(), "PLACE: Object successfully placed in bin!");
                        currentState = RobotState::DONE;
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