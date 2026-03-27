#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
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
#include <limits>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Struct for returned values from YOLO detection
struct YoloDetection {
    std::string name;
    float confidence;
    float x;
    float y;
    float phi;
    bool valid;
};

/* Defining FSM states */
enum class RobotState {
    INIT,
    PICKUP_OBJECT,
    NAVIGATE_SCENE,
    DETECT_SCENE_OBJECT,
    CHECK_MATCH,
    PLACE_IN_BIN,
    RETURN_HOME,
    WRITE_OUTPUTS,
    DONE
};

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters
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

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    rclcpp::QoS amcl_qos(rclcpp::KeepLast(10));
    amcl_qos.reliable();
    amcl_qos.transient_local();
    
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        amcl_qos,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Initialize external classes
    Navigation nav(node);
    YoloInterface yolo(node);
    ArmController arm(node);
    
    // AprilTagDetector tag_detector(node);
    // tag_detector.setReferenceFrame("oakd_rgb_camera_optical_frame");
    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};

    // FSM initialization
    RobotState currentState = RobotState::INIT;

    // Necessary variables
    int boxCounter = 0; 
    float initial_x = 0.0, initial_y = 0.0, initial_phi = 0.0;
    float x = 0.0, y = 0.0, phi = 0.0;
    bool objectInArm = false;
    bool objectPlaced = false; 
    float yoloConfidenceScore = 0.0;
    std::string yoloObjectName = ""; 
    std::string targetObject = ""; 
    std::string manipulableObjectName = ""; 
    float manipulableObjectConfidence = -1.0f; 
    int sceneDetectAttempts = 0;
    int pickAttempts = 0;
    int placeAttempts = 0;
    const int MAX_PICK_ATTEMPTS = 3;
    const int MAX_PLACE_ATTEMPTS = 3;

    std::vector<YoloDetection> sceneDetections;
    sceneDetections.reserve(5);

    // -----------------------------------------------------------------------
    // Navigable-distance path planning setup
    // -----------------------------------------------------------------------
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    auto path_planner_client = rclcpp_action::create_client<ComputePathToPose>(node, "compute_path_to_pose");

    auto getNavigablePathLength = [&](double from_x, double from_y, double to_x, double to_y) -> double {
        if (!path_planner_client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(node->get_logger(), "compute_path_to_pose server not available; falling back to Euclidean");
            return std::sqrt(std::pow(to_x - from_x, 2) + std::pow(to_y - from_y, 2));
        }

        auto goal = ComputePathToPose::Goal();
        goal.use_start = true;
        goal.start.header.frame_id = "map";
        goal.start.header.stamp = node->now();
        goal.start.pose.position.x = from_x;
        goal.start.pose.position.y = from_y;
        goal.start.pose.orientation.w = 1.0; 

        goal.goal.header.frame_id = "map";
        goal.goal.header.stamp = node->now();
        goal.goal.pose.position.x = to_x;
        goal.goal.pose.position.y = to_y;
        goal.goal.pose.orientation.w = 1.0;

        auto goal_handle_future = path_planner_client->async_send_goal(goal);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
            return std::sqrt(std::pow(to_x - from_x, 2) + std::pow(to_y - from_y, 2));
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            return std::sqrt(std::pow(to_x - from_x, 2) + std::pow(to_y - from_y, 2));
        }

        auto result_future = path_planner_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
            return std::sqrt(std::pow(to_x - from_x, 2) + std::pow(to_y - from_y, 2));
        }

        auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) return -1.0;

        const auto& poses = result.result->path.poses;
        double length = 0.0;
        for (size_t i = 1; i < poses.size(); ++i) {
            length += std::sqrt(std::pow(poses[i].pose.position.x - poses[i-1].pose.position.x, 2) + 
                                std::pow(poses[i].pose.position.y - poses[i-1].pose.position.y, 2));
        }
        return length;
    };

    std::vector<int> visitOrder;
    auto buildVisitOrder = [&](double start_x, double start_y) {
        int n = static_cast<int>(boxes.coords.size());
        std::vector<bool> visited(n, false);
        visitOrder.clear();
        visitOrder.reserve(n);

        float offset = 0.5;
        double cx = start_x, cy = start_y;

        for (int step = 0; step < n; ++step) {
            int best = -1;
            double bestCost = std::numeric_limits<double>::max();

            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                double cost = getNavigablePathLength(
                    cx, cy,
                    boxes.coords[j][0] + (offset * cos(boxes.coords[j][2])), 
                    boxes.coords[j][1] + (offset * sin(boxes.coords[j][2]))
                );

                if (cost < 0.0) cost = std::numeric_limits<double>::max();
                if (cost < bestCost) { bestCost = cost; best = j; }
            }

            if (best == -1) {
                for (int j = 0; j < n; ++j) { if (!visited[j]) { best = j; break; } }
            }

            visited[best] = true;
            visitOrder.push_back(best);
            cx = boxes.coords[best][0] + (offset * cos(boxes.coords[best][2]));
            cy = boxes.coords[best][1] + (offset * sin(boxes.coords[best][2]));

            RCLCPP_INFO(node->get_logger(), "[PathPlan] step %d -> box %d (navigable dist=%.2f m)", step, best, bestCost);
        }
    };

    // -----------------------------------------------------------------------
    // Main FSM Loop
    // -----------------------------------------------------------------------
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        // Wait for valid AMCL pose
        while (x == 0.0 && y == 0.0 && phi == 0.0) {
            rclcpp::spin_some(node);
            x = robotPose.x; y = robotPose.y; phi = robotPose.phi;
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "INIT: Waiting for valid AMCL pose...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        switch(currentState) 
        {
            case RobotState::INIT:
                initial_x = x; initial_y = y; initial_phi = phi;
                RCLCPP_INFO(node->get_logger(), "Start pose saved. Building navigable visit order...");
                buildVisitOrder(static_cast<double>(initial_x), static_cast<double>(initial_y));
                currentState = RobotState::PICKUP_OBJECT;
                break;
            
            case RobotState::PICKUP_OBJECT: {
                bool pickSuccess = false;

                // Step 0: Move up to avoid knocking over cup
                if (!arm.moveToCartesianPose(0.024, -0.192, 0.301, -0.432, -0.467, -0.555, 0.535)) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Step 0 hover unreachable");
                    pickAttempts++; break;
                }
                // Step 1: Move arm to hover position above object
                else if (!arm.moveToCartesianPose(0.131, -0.000, 0.201, -0.001, 0.004, 0.077, 0.997)) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Step 1 hover unreachable");
                    pickAttempts++; break;
                }
                // Step 2: Open gripper before descending
                else if (!arm.openGripper()) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to open gripper");
                    pickAttempts++; break;
                }
                // Step 3: Descend straight down to object
                else if (!arm.moveToCartesianPose(0.132, 0.000, 0.169, 0.002, -0.016, 0.076, 0.997)) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Grasp pose unreachable");
                    pickAttempts++; break;
                }
                // Step 4: Close gripper to grip the object
                else if (!arm.closeGripper()) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to close gripper");
                    pickAttempts++; break;
                }
                // Step 5: Lift arm above object
                else if (!arm.moveToCartesianPose(0.132, -0.017, 0.212, 0.006, -0.113, 0.003, 0.994)) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Post-grasp lift unreachable");
                    pickAttempts++; break;
                }
                // Step 5b: Translate to front position
                else if (!arm.moveToCartesianPose(0.030, -0.136, 0.212, -0.083, -0.094, -0.689, 0.714)) {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 5b translation failed");
                    pickAttempts++; break;
                }
                // Step 6a: Translate to front position in front of OAK-D camera
                else if (!arm.moveToCartesianPose(0.045, -0.277, 0.038, 0.097, 0.112, -0.683, 0.716)) {
                    RCLCPP_WARN(node->get_logger(), "PICKUP: Step 6a translation failed");
                    pickAttempts++; break;
                }
                else {
                    // Arm is now holding object in front of the OAK-D camera
                    RCLCPP_INFO(node->get_logger(), "PICKUP: Arm in position, running YOLO OAK-D detection...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Allow camera feed to settle
                    
                    manipulableObjectName = yolo.getObjectName(CameraSource::OAKD, true);
                    manipulableObjectConfidence = yolo.getConfidence();
                    
                    if (manipulableObjectName.empty() || manipulableObjectConfidence < 0.0f) {
                        RCLCPP_WARN(node->get_logger(), "PICKUP: OAK-D failed to detect object. Will retry loop.");
                        pickAttempts++; break;
                    }

                    targetObject = manipulableObjectName;
                    RCLCPP_INFO(node->get_logger(), "PICKUP: Successfully Latched Target -> '%s' (%.2f)", 
                                targetObject.c_str(), manipulableObjectConfidence);

                    // Step 7: Go back inside to clear workspace for navigation
                    if (!arm.moveToCartesianPose(0.030, -0.136, 0.212, -0.083, -0.094, -0.689, 0.714)) {
                        RCLCPP_WARN(node->get_logger(), "PICKUP: Step 7 translation back into robot failed");
                        pickAttempts++; break;
                    }

                    pickSuccess = true;
                    pickAttempts = 0;
                }

                if (pickSuccess) {
                    objectInArm = true;
                    RCLCPP_INFO(node->get_logger(), "PICKUP: Object successfully picked up and identified!");
                    currentState = RobotState::NAVIGATE_SCENE;
                } else if (pickAttempts >= MAX_PICK_ATTEMPTS) {
                    RCLCPP_ERROR(node->get_logger(), "PICKUP: Max attempts reached — skipping, continuing empty-handed.");
                    pickAttempts = 0;
                    objectInArm = false;
                    currentState = RobotState::NAVIGATE_SCENE;
                }
                break;
            }
            
            case RobotState::NAVIGATE_SCENE:
                if (boxCounter < static_cast<int>(visitOrder.size())) {
                    double offset = 0.5; // m
                    int targetIdx = visitOrder[boxCounter];
                    double goal_phi = boxes.coords[targetIdx][2];
                    double goal_x = boxes.coords[targetIdx][0] + offset * cos(goal_phi);
                    double goal_y = boxes.coords[targetIdx][1] + offset * sin(goal_phi);

                    // Spin robot 180 degrees to face back toward the box and normalize
                    goal_phi += M_PI;
                    goal_phi = atan2(sin(goal_phi), cos(goal_phi));

                    RCLCPP_INFO(node->get_logger(), "[NAVIGATE_SCENE] Moving to box %d (visit %d/%zu)", targetIdx, boxCounter + 1, visitOrder.size());

                    bool navSuccess = nav.moveToGoal(goal_x, goal_y, goal_phi);

                    if (navSuccess) {
                        RCLCPP_INFO(node->get_logger(), "[NAVIGATE_SCENE] Box %d reached.", targetIdx);
                        currentState = RobotState::DETECT_SCENE_OBJECT;
                    } else {
                        RCLCPP_WARN(node->get_logger(), "[NAVIGATE_SCENE] Failed to reach box %d. Skipping.", targetIdx);
                        boxCounter++;
                    }
                } else {
                    currentState = RobotState::RETURN_HOME;
                }
                break;

            case RobotState::DETECT_SCENE_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "DETECT_SCENE_OBJECT: calling YOLO");
                
                yoloObjectName = yolo.getObjectName(CameraSource::OAKD, true);
                float confidence = yolo.getConfidence();
                
                // Use "clock" instead of "mouse" as requested
                bool objects_allowed = (yoloObjectName == "clock") || (yoloObjectName == "cup") || 
                                       (yoloObjectName == "bottle") || (yoloObjectName == "motorcycle") || 
                                       (yoloObjectName == "potted plant");

                if (!yoloObjectName.empty() && confidence >= 0.0f && objects_allowed) {
                    int targetIdx = visitOrder[boxCounter];
                    sceneDetections.push_back({
                        yoloObjectName, confidence, 
                        static_cast<float>(boxes.coords[targetIdx][0]), 
                        static_cast<float>(boxes.coords[targetIdx][1]), 
                        static_cast<float>(boxes.coords[targetIdx][2]), true
                    });
                    
                    sceneDetectAttempts = 0;
                    RCLCPP_INFO(node->get_logger(), "Object Identified: %s", yoloObjectName.c_str());
                    currentState = RobotState::CHECK_MATCH;
                } else {
                    sceneDetectAttempts++;
                    if (sceneDetectAttempts >= 5) {
                        RCLCPP_WARN(node->get_logger(), "Scene detection failed after %d attempts; skipping box.", sceneDetectAttempts);
                        sceneDetectAttempts = 0;
                        boxCounter++;
                        currentState = RobotState::NAVIGATE_SCENE;
                    }
                }
                break;
            }
            
            case RobotState::CHECK_MATCH:
                if (!objectPlaced && yoloObjectName == targetObject) {
                    RCLCPP_INFO(node->get_logger(), "Object Matches Manipulable Target!");
                    currentState = RobotState::PLACE_IN_BIN;
                } else {
                    RCLCPP_INFO(node->get_logger(), "No match or already placed. Moving to next box to map environment.");
                    boxCounter++;
                    currentState = RobotState::NAVIGATE_SCENE;
                }
                break;
            
            case RobotState::PLACE_IN_BIN:
                RCLCPP_INFO(node->get_logger(), "Placing object in bin");
                {
                    if (placeAttempts >= MAX_PLACE_ATTEMPTS) {
                        RCLCPP_ERROR(node->get_logger(),
                            "PLACE: Max attempts reached — aborting.");
                        placeAttempts = 0;
                        currentState = RobotState::NAVIGATE_SCENE;
                        break;
                    }

                    double offset = 0.2; // m
                    int targetIdx = visitOrder[boxCounter];
                    double goal_phi = boxes.coords[targetIdx][2];
                    double goal_x = boxes.coords[targetIdx][0] + offset * cos(goal_phi);
                    double goal_y = boxes.coords[targetIdx][1] + offset * sin(goal_phi);

                    // Spin robot 180 degrees to face back toward the box and normalize
                    goal_phi += M_PI;
                    goal_phi = atan2(sin(goal_phi), cos(goal_phi));

                    RCLCPP_INFO(node->get_logger(), "[NAVIGATE_SCENE] Moving to box %d (visit %d/%zu)", targetIdx, boxCounter + 1, visitOrder.size());

                    bool navSuccess = nav.moveToGoal(goal_x, goal_y, goal_phi);

                    if (navSuccess) {
                        RCLCPP_INFO(node->get_logger(), "[NAVIGATE_SCENE] Box %d reached.", targetIdx);
                    } else {
                        RCLCPP_WARN(node->get_logger(), "[NAVIGATE_SCENE] Failed to reach box %d. Skipping.", targetIdx);
                        boxCounter++;
                    }

                    // RCLCPP_INFO(node->get_logger(),
                    //     "PLACE: Looking for AprilTag ID...");

                    // auto visible_tags = tag_detector.getVisibleTags(candidate_tags);
                    // std::optional<geometry_msgs::msg::Pose> selected_pose;
                    // int selected_tag_id = -1;
                    // if (!visible_tags.empty()) {
                    //     for (int tag_id : visible_tags) {
                    //         auto bin_pose = tag_detector.getTagPose(tag_id);
                    //         if (bin_pose.has_value()) {
                    //             RCLCPP_INFO(node->get_logger(),
                    //                 "%s -> tag%d: pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
                    //                 tag_detector.getReferenceFrame().c_str(), tag_id,
                    //                 bin_pose->position.x, bin_pose->position.y, bin_pose->position.z,
                    //                 bin_pose->orientation.x, bin_pose->orientation.y, bin_pose->orientation.z, bin_pose->orientation.w);
                    //             selected_pose = bin_pose;
                    //             selected_tag_id = tag_id;
                    //             break; // use the first valid tag pose
                    //         }
                    //     }
                    // } else {
                    //     RCLCPP_INFO(node->get_logger(), "No tags visible");
                    // }
                    // RCLCPP_INFO(node->get_logger(), "---------------------------------");
                    // if (!selected_pose.has_value()) {
                    //     RCLCPP_WARN(node->get_logger(),
                    //         "PLACE: No valid tag pose available — retrying.");
                    //     break;
                    // }

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
                        currentState = RobotState::NAVIGATE_SCENE;
                    }
                }
                break;
            
            case RobotState::RETURN_HOME:
                RCLCPP_INFO(node->get_logger(), "[RETURN_HOME] Navigating back to start: x=%.2f, y=%.2f, phi=%.2f", initial_x, initial_y, initial_phi);
                if (nav.moveToGoal(static_cast<double>(initial_x), static_cast<double>(initial_y), static_cast<double>(initial_phi))) {
                    RCLCPP_INFO(node->get_logger(), "[RETURN_HOME] Successfully returned to start position.");
                } else {
                    RCLCPP_WARN(node->get_logger(), "[RETURN_HOME] Could not reach start exactly.");
                }
                currentState = RobotState::WRITE_OUTPUTS;
                break;

            case RobotState::WRITE_OUTPUTS: {
                RCLCPP_INFO(node->get_logger(), "Writing output files...");
                // Saving in the working directory as requested
                std::ofstream out("contest2_output.txt");
                if (!out.is_open()) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to open ./contest2_output.txt for writing");
                } else {
                    out << "Pickup: " << targetObject << " (" << manipulableObjectConfidence << ")\n";
                    out << "Scene Objects:\n";
                    for (size_t i = 0; i < sceneDetections.size(); ++i) {
                        const auto& d = sceneDetections[i];
                        out << i << ": " << d.name << " (" << d.confidence << ") @ x="
                            << d.x << " y=" << d.y << " phi=" << d.phi << "\n";
                    }
                }
                currentState = RobotState::DONE;
                break;
            }

            case RobotState::DONE:
                // Idle state until node termination
                break;
            
            default:
                RCLCPP_ERROR(node->get_logger(), "CRITICAL ERROR: FSM entered unknown state.");
                currentState = RobotState::DONE; 
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
