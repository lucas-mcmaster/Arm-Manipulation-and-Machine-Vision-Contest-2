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

/*Defining FSM states in a class -- from research this is better than if/else statements as it
will allow us to adjust the priority and add states more easily if needed*/
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

    // // ARM: Load the arm URDF and SRDF directly as node parameters so that
    // // MoveGroupInterface builds the SO-ARM101 model
    // {
    //     std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
    //     std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
    //     if (urdf_file.is_open()) {
    //         std::stringstream ss;
    //         ss << urdf_file.rdbuf();
    //         node->declare_parameter("robot_description", ss.str());
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
    //     }

    //     std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
    //     std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
    //     if (srdf_file.is_open()) {
    //         std::stringstream ss;
    //         ss << srdf_file.rdbuf();
    //         node->declare_parameter("robot_description_semantic", ss.str());
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
    //     }
    // }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    // RobotPose robotPose(0, 0, 0);
    
    // AMCL: Force transient_local durability (as requested).
    // rclcpp::QoS amcl_qos(rclcpp::KeepLast(10));
    // amcl_qos.reliable();
    // amcl_qos.transient_local();
    
    // auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     "/amcl_pose",
    //     amcl_qos,
    //     std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    // );

    // NAV2: Initialize box coordinates
    // Boxes boxes;
    // if(!boxes.load_coords()) {
    //     RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
    //     return -1;
    // }

    // for(size_t i = 0; i < boxes.coords.size(); ++i) {
    //     RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
    //                 i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    // }
    

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy

    //initializing external classes from other files
    // Navigation nav(node); //these names can be changes as you'd like
    YoloInterface yolo(node);
    // ArmController arm(node);
    // AprilTagDetector tag_detector(node);

    //FSM initialization
    RobotState currentState = RobotState::DETECT_SCENE_OBJECT;

    //Initializing necessary variables -- everyone feel free to add to this as needed
    // int boxCounter=0; //box counter to know when we have gone to each box 
    // float initial_x=0.0, initial_y=0.0, initial_phi=0.0; //these are starting coordinates to store from AMCL to have robot return to at the end
    // float x=0.0, y=0.0, phi=0.0; //there are variables for current x,y and phi position
    // bool objectInArm=false; //bool to check if the object is in the robot's arm--to do at start
    // bool objectPlaced=false; //bool to check if object has been placed into bin
    float yoloConfidenceScore=0.0; // confidence score from yolo
    std::string yoloObjectName=""; // object identified from yolo
    int sceneDetectAttempts=0; // retry counter for scene object detection
    auto sceneDetectStart = std::chrono::steady_clock::time_point::min();
    auto lastSceneDetectAttempt = std::chrono::steady_clock::time_point::min();
    bool finished = false;

    // ── Manipulable object coordinates in the arm base frame ─────────────────
    // UPDATE BEFORE RUNNING.
    // const double OBJ_ARM_X = 0.15;   // metres forward from arm base
    // const double OBJ_ARM_Y = 0.15;   // metres lateral
    // const double OBJ_ARM_Z = 0.15;   // metres height (top plate surface)

    // Retry counter for arm pick attempts
    // int pickAttempts = 0;
    // const int MAX_PICK_ATTEMPTS = 3;
    // auto sceneDetectStart = std::chrono::steady_clock::time_point::min();
    // auto lastSceneDetectAttempt = std::chrono::steady_clock::time_point::min();

    // Struct for returned values from YOLO detection in helper below
    struct YoloDetection {
        std::string name;
        float confidence;
        bool valid;
    };

    // Scene detections (store first successful result for output)
    std::vector<YoloDetection> sceneDetections;
    sceneDetections.reserve(1);

    // -----------------------------------------------------------------------
    // Bido's section – navigable-distance path planning
    // -----------------------------------------------------------------------
    // Action client for Nav2's ComputePathToPose — asks the global planner to
    // plan a path without executing it, so we can measure real path length.
    // using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    // auto path_planner_client = rclcpp_action::create_client<ComputePathToPose>(
    //     node, "compute_path_to_pose");

    // // Helper: ask Nav2 global planner for the path length between two map
    // // poses.  Returns -1.0 on failure (unreachable / server unavailable).
    // auto getNavigablePathLength = [&](double from_x, double from_y,
    //                                   double to_x,   double to_y) -> double {
    //     // Need planner server running; give it up to 2 s on first call
    //     if (!path_planner_client->wait_for_action_server(
    //             std::chrono::seconds(2))) {
    //         RCLCPP_WARN(node->get_logger(),
    //             "compute_path_to_pose server not available; falling back to Euclidean");
    //         double dx = to_x - from_x, dy = to_y - from_y;
    //         return std::sqrt(dx*dx + dy*dy);
    //     }

    //     // Build goal: use explicit start pose so we don't need the robot to
    //     // be at from_x/from_y right now.
    //     auto goal = ComputePathToPose::Goal();
    //     goal.use_start = true;

    //     goal.start.header.frame_id = "map";
    //     goal.start.header.stamp    = node->now();
    //     goal.start.pose.position.x = from_x;
    //     goal.start.pose.position.y = from_y;
    //     goal.start.pose.orientation.w = 1.0;  // heading doesn't affect length

    //     goal.goal.header.frame_id  = "map";
    //     goal.goal.header.stamp     = node->now();
    //     goal.goal.pose.position.x  = to_x;
    //     goal.goal.pose.position.y  = to_y;
    //     goal.goal.pose.orientation.w = 1.0;

    //     goal.planner_id = "";  // use default planner (GridBased / NavFn)

    //     // Send goal and wait (blocking, planning-only — no motion)
    //     auto goal_handle_future = path_planner_client->async_send_goal(goal);
    //     if (rclcpp::spin_until_future_complete(node, goal_handle_future,
    //             std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_WARN(node->get_logger(),
    //             "compute_path_to_pose: send_goal timed out");
    //         double dx = to_x - from_x, dy = to_y - from_y;
    //         return std::sqrt(dx*dx + dy*dy);
    //     }

    //     auto goal_handle = goal_handle_future.get();
    //     if (!goal_handle) {
    //         RCLCPP_WARN(node->get_logger(),
    //             "compute_path_to_pose: goal rejected");
    //         double dx = to_x - from_x, dy = to_y - from_y;
    //         return std::sqrt(dx*dx + dy*dy);
    //     }

    //     auto result_future = path_planner_client->async_get_result(goal_handle);
    //     if (rclcpp::spin_until_future_complete(node, result_future,
    //             std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_WARN(node->get_logger(),
    //             "compute_path_to_pose: result timed out");
    //         double dx = to_x - from_x, dy = to_y - from_y;
    //         return std::sqrt(dx*dx + dy*dy);
    //     }

    //     auto result = result_future.get();
    //     if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    //         RCLCPP_WARN(node->get_logger(),
    //             "compute_path_to_pose: planning failed (unreachable?)");
    //         return -1.0;
    //     }

    //     // Sum arc length of the returned path
    //     const auto& poses = result.result->path.poses;
    //     double length = 0.0;
    //     for (size_t i = 1; i < poses.size(); ++i) {
    //         double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
    //         double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
    //         length += std::sqrt(dx*dx + dy*dy);
    //     }
    //     RCLCPP_DEBUG(node->get_logger(),
    //         "Navigable length (%.2f,%.2f)->(%.2f,%.2f) = %.3f m",
    //         from_x, from_y, to_x, to_y, length);
    //     return length;
    // };

    // // Greedy nearest-navigable-neighbour ordering.
    // // Called once from INIT with the real AMCL starting pose.
    // // visitOrder[i] = index into boxes.coords for the i-th stop.
    // std::vector<int> visitOrder;
    // auto buildVisitOrder = [&](double start_x, double start_y) {
    //     int n = static_cast<int>(boxes.coords.size());
    //     std::vector<bool> visited(n, false);
    //     visitOrder.clear();
    //     visitOrder.reserve(n);

    //     double cx = start_x, cy = start_y;

    //     for (int step = 0; step < n; ++step) {
    //         int    best     = -1;
    //         double bestCost = std::numeric_limits<double>::max();

    //         for (int j = 0; j < n; ++j) {
    //             if (visited[j]) continue;
    //             double cost = getNavigablePathLength(
    //                 cx, cy,
    //                 boxes.coords[j][0], boxes.coords[j][1]);

    //             // -1 means unreachable; treat as worst case
    //             if (cost < 0.0) cost = std::numeric_limits<double>::max();

    //             if (cost < bestCost) { bestCost = cost; best = j; }
    //         }

    //         // If all remaining goals are unreachable, fall back to Euclidean
    //         // distance to avoid invalid indexing (best == -1).
    //         if (best == -1 || bestCost == std::numeric_limits<double>::max()) {
    //             best = -1;
    //             bestCost = std::numeric_limits<double>::max();
    //             for (int j = 0; j < n; ++j) {
    //                 if (visited[j]) continue;
    //                 double dx = boxes.coords[j][0] - cx;
    //                 double dy = boxes.coords[j][1] - cy;
    //                 double cost = std::sqrt(dx*dx + dy*dy);
    //                 if (cost < bestCost) { bestCost = cost; best = j; }
    //             }
    //             if (best == -1) {
    //                 RCLCPP_WARN(node->get_logger(),
    //                     "[PathPlan] No remaining boxes to visit (unexpected).");
    //                 break;
    //             }
    //             RCLCPP_WARN(node->get_logger(),
    //                 "[PathPlan] All navigable paths failed; falling back to Euclidean ordering.");
    //         }

    //         visited[best] = true;
    //         visitOrder.push_back(best);
    //         cx = boxes.coords[best][0];
    //         cy = boxes.coords[best][1];

    //         RCLCPP_INFO(node->get_logger(),
    //             "[PathPlan] step %d -> box %d  (x=%.2f, y=%.2f, phi=%.2f)  "
    //             "navigable dist=%.2f m",
    //             step, best,
    //             boxes.coords[best][0],
    //             boxes.coords[best][1],
    //             boxes.coords[best][2],
    //             bestCost);
    //     }
    // };
    // -----------------------------------------------------------------------

    // YOLO cooldown (1s)
    auto lastYoloTime = std::chrono::steady_clock::time_point::min();

    // Helper for YOLO detection in FSM cases (Nick's sections)
    auto runYoloDetection = [&](CameraSource camera, bool save_image) -> YoloDetection {
        auto now = std::chrono::steady_clock::now();
        if (now - lastYoloTime < std::chrono::seconds(1)) {
            // RCLCPP_INFO(node->get_logger(), "YOLO: cooldown active, skipping detection");
            return {"", -1.0f, false};
        }
        lastYoloTime = now;

        // Save detection
        std::string detected = yolo.getObjectName(camera, save_image);

        // Get detection confidence
        float confidence = yolo.getConfidence();

        // Warn about no valid detection or negative confidence
        if (detected.empty() || confidence < 0.0f) {
            RCLCPP_WARN(node->get_logger(), "YOLO: no valid detection");
            return {"", -1.0f, false};
        }

        // Store, print, and return detection
        yoloConfidenceScore = confidence;
        RCLCPP_INFO(node->get_logger(), "YOLO: %s (%.2f)", detected.c_str(), confidence);
        return {detected, confidence, true};
    };

    while(rclcpp::ok() && !finished && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        /***YOUR CODE HERE***/
        //Keeping track of pose consistently 
        

        // while (x == 0.0 && y == 0.0 && phi == 0.0) {
        //     x=robotPose.x;
        //     y=robotPose.y;
        //     phi=robotPose.phi;
        //     RCLCPP_INFO_THROTTLE(node->get_logger(),
        //                 *node->get_clock(), 2000,   // log every 2s to avoid spam
        //                 "INIT: Waiting for valid AMCL pose...");       
        // }

        RCLCPP_INFO_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "FSM: currentState=%d", static_cast<int>(currentState));

        switch(currentState) //using switch function for FSM. Similar to If/Else but better practice and easier to change according to google
        {
            // case RobotState::INIT:

            //     initial_x   = x;
            //     initial_y   = y;
            //     initial_phi = phi;
            //     RCLCPP_INFO(node->get_logger(),
            //         "Start pose saved. x=%.2f, y=%.2f, phi=%.2f. "
            //         "Building navigable visit order...",
            //         initial_x, initial_y, initial_phi);

            //     buildVisitOrder(
            //         static_cast<double>(initial_x),
            //         static_cast<double>(initial_y));

            //     currentState = RobotState::NAVIGATE_SCENE;
            //     break;
            
            // case RobotState::PICKUP_OBJECT: {
            //     // Nick + Ahmed's section

            //     bool pickSuccess = false;

            //     // Step 1: Move arm to hover position above object FIRST (for wrist cam to see it clearly)
            //     if (!arm.moveToCartesianPose(
            //         OBJ_ARM_X, OBJ_ARM_Y, OBJ_ARM_Z + 0.08, //WE SHOULD UP THIS --- HAVE ROBOT A DECENT HEIGHT ABOVE SO GOAL DOES NOT GET REJECTED AND ROBOT DOES NOT HIT CUP ON THE WAY TO IT -Lucas
            //         0.0, M_PI / 2.0, 0.0))
            //     {
            //         RCLCPP_ERROR(node->get_logger(), "PICKUP: Pre-grasp hover unreachable — retrying");
            //         pickAttempts++;
            //         break;
            //     }

            //     // Step 2: Take YOLO picture from wrist camera now that arm is positioned above object
            //     if (!manipulableObjectLatched) {
            //         YoloDetection det = runYoloDetection(CameraSource::WRIST, true);
            //         if (!det.valid) {
            //             // Try again next loop — arm stays at hover position
            //             break;
            //         }

            //         // Latch first valid detection
            //         manipulableObjectName = det.name;
            //         manipulableObjectConfidence = det.confidence;
            //         manipulableObjectLatched = true;
            //         targetObject = manipulableObjectName;

            //         RCLCPP_INFO(node->get_logger(),
            //             "PICKUP: Detected '%s' (%.2f) — proceeding to grasp",
            //             manipulableObjectName.c_str(), manipulableObjectConfidence);
            //     }

            //     // Step 3: Open gripper before descending
            //     if (!arm.openGripper()) {
            //         RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to open gripper");
            //     }
            //     // Step 4: Descend straight down to object
            //     else if (!arm.moveToCartesianPose(
            //         OBJ_ARM_X, OBJ_ARM_Y, OBJ_ARM_Z,
            //         0.0, M_PI / 2.0, 0.0))
            //     {
            //         RCLCPP_ERROR(node->get_logger(), "PICKUP: Grasp pose unreachable");
            //     }
            //     // Step 5: Close gripper to grip the object
            //     else if (!arm.closeGripper()) {
            //         RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to close gripper");
            //     }
            //     else {
            //         // Step 6: Brief pause to confirm grip is stable
            //         std::this_thread::sleep_for(std::chrono::milliseconds(500));

            //         // Step 7: Lift to safe carry height (15 cm above object)
            //         if (!arm.moveToCartesianPose(
            //             OBJ_ARM_X, OBJ_ARM_Y, OBJ_ARM_Z + 0.15,
            //             0.0, M_PI / 2.0, 0.0))
            //         {
            //             RCLCPP_WARN(node->get_logger(),
            //                 "PICKUP: Lift step failed — object may still be gripped, continuing");
            //         }
            //         pickSuccess = true;
            //         pickAttempts = 0;
            //     }

            //     if (pickSuccess) {
            //         objectInArm = true;
            //         RCLCPP_INFO(node->get_logger(), "PICKUP: Object successfully picked up!");
            //         currentState = RobotState::NAVIGATE_SCENE;
            //     } else {
            //         pickAttempts++;
            //         RCLCPP_WARN(node->get_logger(),
            //             "PICKUP: Attempt %d/%d failed.", pickAttempts, MAX_PICK_ATTEMPTS);
            //         if (pickAttempts >= MAX_PICK_ATTEMPTS) {
            //             RCLCPP_ERROR(node->get_logger(),
            //                 "PICKUP: Max attempts reached — skipping pickup, continuing without object.");
            //             pickAttempts = 0;
            //             objectInArm = false;
            //             currentState = RobotState::NAVIGATE_SCENE;
            //         }
            //     }
            
            //     break;
            // }
            
            // case RobotState::NAVIGATE_SCENE:
            //     // ---------------------------------------------------------------
            //     // Bido's section – navigate to next scene object using the
            //     // pre-computed nearest-neighbour visit order.
            //     // ---------------------------------------------------------------
            //     if (boxCounter < static_cast<int>(visitOrder.size()))
            //     {

            //         // offset to avoid collision and nav2 failure
            //         double offset = 0.5; // m

            //         // Retrieve optimised destination from visitOrder
            //         int targetIdx = visitOrder[boxCounter];
            //         double goal_phi = boxes.coords[targetIdx][2];
            //         double goal_x   = boxes.coords[targetIdx][0] + offset * cos(goal_phi);
            //         double goal_y   = boxes.coords[targetIdx][1] + offset * sin(goal_phi);


            //         // Fallback navigation with no offset
            //         // double goal_x   = boxes.coords[targetIdx][0];
            //         // double goal_y   = boxes.coords[targetIdx][1];

            //         goal_phi += M_PI;

            //         // Normalize goal_phi
            //         if (goal_phi > M_PI) goal_phi -= 2*M_PI;

            //         RCLCPP_INFO(node->get_logger(),
            //             "[NAVIGATE_SCENE] Moving to box %d (visit %d/%zu)  "
            //             "x=%.2f, y=%.2f, phi=%.2f",
            //             targetIdx, boxCounter + 1, visitOrder.size(),
            //             goal_x, goal_y, goal_phi);

            //         // nav.moveToGoal() blocks until Nav2 reports success/failure.
            //         bool navSuccess = nav.moveToGoal(goal_x, goal_y, goal_phi);

            //         if (navSuccess) {
            //             RCLCPP_INFO(node->get_logger(),
            //                 "[NAVIGATE_SCENE] Box %d reached successfully.", targetIdx);
            //             currentState = RobotState::DETECT_SCENE_OBJECT;
            //         } else {
            //             // Navigation failed (obstacle, timeout, etc.).
            //             // Log and skip this box to avoid getting stuck, then
            //             // increment the counter so we try the next location.
            //             RCLCPP_WARN(node->get_logger(),
            //                 "[NAVIGATE_SCENE] Failed to reach box %d. Skipping.", targetIdx);
            //             boxCounter++;
            //             // Stay in NAVIGATE_SCENE to attempt next box on next loop.
            //         }
            //     }
            //     else
            //     {
            //         // All boxes visited
            //         if (!objectPlaced) {
            //             RCLCPP_WARN(node->get_logger(),
            //                 "[NAVIGATE_SCENE] Checked all boxes but didn't place object.");
            //         }
            //         currentState = RobotState::RETURN_HOME;
            //     }
            //     // ---------------------------------------------------------------
            //     break;

            case RobotState::DETECT_SCENE_OBJECT:
                //Nick's section
                RCLCPP_INFO(node->get_logger(), "DETECT_SCENE_OBJECT: calling YOLO");
                {
                    auto yolo_call_start = std::chrono::steady_clock::now();
                    yoloObjectName = yolo.getObjectName(CameraSource::OAKD, true);
                    float confidence = yolo.getConfidence();
                    bool has_name = !yoloObjectName.empty();
                    bool conf_ok = confidence >= 0.0f;

                    // List of allowed objects
                    bool objects_allowed =
                        (yoloObjectName == "mouse") ||
                        (yoloObjectName == "cup") ||
                        (yoloObjectName == "bottle") ||
                        (yoloObjectName == "motorcycle") ||
                        (yoloObjectName == "potted plant");

                    YoloDetection det{yoloObjectName, confidence, has_name && conf_ok && objects_allowed};
                    auto yolo_call_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - yolo_call_start).count();

                    RCLCPP_INFO(node->get_logger(),
                        "DETECT_SCENE_OBJECT: name='%s' confidence=%.3f valid=%d (call %lld ms)",
                        yoloObjectName.c_str(), confidence, static_cast<int>(det.valid),
                        static_cast<long long>(yolo_call_ms));
                    if (!has_name) {
                        RCLCPP_WARN(node->get_logger(),
                            "DETECT_SCENE_OBJECT: invalid because name is empty");
                    }
                    if (!conf_ok) {
                        RCLCPP_WARN(node->get_logger(),
                            "DETECT_SCENE_OBJECT: invalid because confidence < 0 (%.3f)",
                            confidence);
                    }
                    if (!det.valid) {
                        break;
                    }

                    // Store scene object in vector if space left
                    yoloObjectName = det.name;
                    yoloConfidenceScore = det.confidence;
                    if (sceneDetections.empty()) {
                        sceneDetections.push_back(det);
                    }
                }

                //Once a valid object is identified switch states to check match      
                RCLCPP_INFO(node->get_logger(), "Object Identified");
                RCLCPP_INFO(node->get_logger(), "DETECT_SCENE_OBJECT: transition -> WRITE_OUTPUTS");
                currentState = RobotState::WRITE_OUTPUTS;
                break;
            
            // case RobotState::CHECK_MATCH:
            //     //Nick's section also - check if object matches what we need 
            //     if(yoloObjectName==targetObject) //if there is a match go to placing object state
            //     {
            //         RCLCPP_INFO(node->get_logger(), "Object Matches!");
            //         currentState=RobotState::PLACE_IN_BIN;
            //         // NOTE: boxCounter is incremented AFTER placement succeeds
            //         // (see PLACE_IN_BIN), so we do NOT increment it here.
            //     }
            //     else //otherwise move on to the next bin
            //     {
            //         boxCounter++;
            //         currentState=RobotState::NAVIGATE_SCENE;
            //     }
            //     break;
            
            // case RobotState::PLACE_IN_BIN: //this section to localize bin with AprilTag and then place object
            //     //Ahmed's section as well
            //     RCLCPP_INFO(node->get_logger(), "Placing object in bin");

            //     // Ahmed's section — AprilTag localisation + arm place sequence
            //     {
            //         // The tag ID matches the original box index from coords.xml.
            //         // boxCounter has NOT been incremented yet (that happens after success),
            //         // so visitOrder[boxCounter] gives the correct current box index.
            //         int tag_id = visitOrder[boxCounter];

            //         RCLCPP_INFO(node->get_logger(),
            //             "PLACE: Looking for AprilTag ID %d ...", tag_id);

            //         // Step 1: Check AprilTag is visible (timeout 2 s)
            //         std::vector<int> visible = tag_detector.getVisibleTags({tag_id}, 2000);
            //         if (visible.empty()) {
            //             RCLCPP_WARN(node->get_logger(),
            //                 "PLACE: AprilTag %d not visible yet — retrying next loop", tag_id);
            //             break; // stay in PLACE_IN_BIN, try again
            //         }

            //         // Step 2: Get bin pose from AprilTag (in base_link frame)
            //         auto bin_pose_opt = tag_detector.getTagPose(tag_id, 2000);
            //         if (!bin_pose_opt.has_value()) {
            //             RCLCPP_WARN(node->get_logger(),
            //                 "PLACE: Could not get pose for tag %d — retrying", tag_id);
            //             break;
            //         }
            //         geometry_msgs::msg::Pose bin_pose = bin_pose_opt.value();

            //         RCLCPP_INFO(node->get_logger(),
            //             "PLACE: Bin localised — camera frame (%.3f, %.3f, %.3f)",
            //             bin_pose.position.x,
            //             bin_pose.position.y,
            //             bin_pose.position.z);

            //         // Step 3: Convert AprilTag pose (camera/base_link frame) → arm base frame.
            //         //   Camera Z (depth forward) maps to arm X (reach forward).
            //         //   Camera X (lateral)       maps to arm -Y.
            //         //   Bin top rim is ~5 cm above the tag centre height.
            //         double bin_arm_x = bin_pose.position.z - 0.05;  // depth → arm forward, back off tag face
            //         double bin_arm_y = -bin_pose.position.x;         // lateral flip
            //         double bin_arm_z =  bin_pose.position.y + 0.05;  // bin top rim height

            //         bool placeSuccess = false;

            //         // Step 4: Pre-place hover — 10 cm above bin opening, gripper pointing down
            //         if (!arm.moveToCartesianPose(
            //             bin_arm_x, bin_arm_y, bin_arm_z + 0.10,
            //             0.0, M_PI / 2.0, 0.0))
            //         {
            //             RCLCPP_ERROR(node->get_logger(), "PLACE: Pre-place hover unreachable — retrying");
            //         }
            //         // Step 5: Lower object into bin
            //         else if (!arm.moveToCartesianPose(
            //             bin_arm_x, bin_arm_y, bin_arm_z,
            //             0.0, M_PI / 2.0, 0.0))
            //         {
            //             RCLCPP_ERROR(node->get_logger(), "PLACE: Could not lower into bin — retrying");
            //         }
            //         // Step 6: Open gripper to release object
            //         else if (!arm.openGripper()) {
            //             RCLCPP_ERROR(node->get_logger(), "PLACE: Gripper release failed — retrying");
            //         }
            //         else {
            //             std::this_thread::sleep_for(std::chrono::milliseconds(400));

            //             // Step 7: Retract arm up and clear of bin
            //             arm.moveToCartesianPose(
            //                 bin_arm_x, bin_arm_y, bin_arm_z + 0.15,
            //                 0.0, M_PI / 2.0, 0.0);

            //             placeSuccess = true;
            //         }

            //         if (placeSuccess) {
            //             objectPlaced = true;
            //             objectInArm  = false;
            //             RCLCPP_INFO(node->get_logger(), "PLACE: Object successfully placed in bin!");
            //         } else {
            //             break; // retry from top of PLACE_IN_BIN next loop
            //         }
            //     }

            //     //objectPlaced=true;
            //     if (objectPlaced)//once its in the bin we continue to next box
            //     {
            //         boxCounter++; // Bido: increment here, after successful placement
            //         currentState=RobotState::NAVIGATE_SCENE;
            //         break;
            //     }
            //     else
            //     {
            //         //add some fallback code to try again
            //         break;
            //     }
            
            // case RobotState::RETURN_HOME:
            //     // ---------------------------------------------------------------
            //     // Bido's section – navigate back to the recorded starting pose.
            //     // ---------------------------------------------------------------
            //     RCLCPP_INFO(node->get_logger(),
            //         "[RETURN_HOME] Navigating back to start: x=%.2f, y=%.2f, phi=%.2f",
            //         initial_x, initial_y, initial_phi);

            //     {
            //         bool homeSuccess = nav.moveToGoal(
            //             static_cast<double>(initial_x),
            //             static_cast<double>(initial_y),
            //             static_cast<double>(initial_phi));

            //         if (homeSuccess) {
            //             RCLCPP_INFO(node->get_logger(),
            //                 "[RETURN_HOME] Successfully returned to start position.");
            //         } else {
            //             RCLCPP_WARN(node->get_logger(),
            //                 "[RETURN_HOME] Could not reach start exactly – proceeding to output.");
            //         }
            //     }
            //     // ---------------------------------------------------------------

            //     currentState = RobotState::WRITE_OUTPUTS;
            //     break;

            case RobotState::WRITE_OUTPUTS: //state to save txt file. Might move this cause if we don't finish it won't write the data
                RCLCPP_INFO(node->get_logger(), "Writing output files...");
                // Write txt file with manipulable object info and all scene objects + locations
                {
                    std::ofstream out("/home/nicolas-rebollo/YOLO Images/contest2_output.txt");
                    if (!out.is_open()) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to open contest2_output.txt for writing");
                    } else {
                        out << "Detected: " << yoloObjectName << " (" << yoloConfidenceScore << ")\n";
                        out << "Scene Objects:\n";
                        for (size_t i = 0; i < sceneDetections.size(); ++i) {
                            const auto& d = sceneDetections[i];
                            out << i << ": " << d.name << " (" << d.confidence << ")\n";
                        }
                    }
                }

                // Switch state to DONE
                currentState = RobotState::DONE;
                finished = true;
                break;

            case RobotState::DONE: //done state once we have completed contest in case there is still time remaining
                // Idle state, do nothing until timer runs out
                break;
            
            default:
                RCLCPP_ERROR(node->get_logger(), "CRITICAL ERROR: FSM entered unknown state. Switching to Idle State.");
                currentState = RobotState::DONE; // Force into a safe, idle state
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
