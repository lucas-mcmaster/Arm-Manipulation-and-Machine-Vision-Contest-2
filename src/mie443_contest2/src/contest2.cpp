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

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
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

    // Execute strategy

    //initializing external classes from other files
    Navigation nav(node); //these names can be changes as you'd like

    //FSM initialization
    RobotState currentState = RobotState::INIT;

    //Initializing necessary variables -- everyone feel free to add to this as needed
    int boxCounter=0; //box counter to know when we have gone to each box 
    float initial_x=0.0, initial_y=0.0, initial_phi=0.0; //these are starting coordinates to store from AMCL to have robot return to at the end
    float x=0.0, y=0.0, phi=0.0; //there are variables for current x,y and phi position
    bool objectInArm=false; //bool to check if the object is in the robot's arm--to do at start
    bool objectPlaced=false; //bool to check if object has been placed into bin
    float yoloConfidenceScore=0.0; // confidence score from yolo
    std::string yoloObjectName=""; //object identified from yolo
    std::string targetObject=""; //object that we need to use 
    std::string manipulableObjectName=""; // manipulable object (on top of turtlebot) identified from yolo
    float manipulableObjectConfidence=-1.0f; // manipulable object confidence identified from yolo
    bool manipulableObjectLatched=false; // latch first valid pickup detection to avoid overwriting

    // Struct for returned values from YOLO detection in helper below
    struct YoloDetection {
        std::string name;
        float confidence;
        float x;
        float y;
        float phi;
        bool valid;
    };

    // -----------------------------------------------------------------------
    // Bido's section – navigable-distance path planning
    // -----------------------------------------------------------------------
    // Action client for Nav2's ComputePathToPose — asks the global planner to
    // plan a path without executing it, so we can measure real path length.
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    auto path_planner_client = rclcpp_action::create_client<ComputePathToPose>(
        node, "compute_path_to_pose");

    // Helper: ask Nav2 global planner for the path length between two map
    // poses.  Returns -1.0 on failure (unreachable / server unavailable).
    auto getNavigablePathLength = [&](double from_x, double from_y,
                                      double to_x,   double to_y) -> double {
        // Need planner server running; give it up to 2 s on first call
        if (!path_planner_client->wait_for_action_server(
                std::chrono::seconds(2))) {
            RCLCPP_WARN(node->get_logger(),
                "compute_path_to_pose server not available; falling back to Euclidean");
            double dx = to_x - from_x, dy = to_y - from_y;
            return std::sqrt(dx*dx + dy*dy);
        }

        // Build goal: use explicit start pose so we don't need the robot to
        // be at from_x/from_y right now.
        auto goal = ComputePathToPose::Goal();
        goal.use_start = true;

        goal.start.header.frame_id = "map";
        goal.start.header.stamp    = node->now();

        //MIGHT NOT BE CORRECT-SHOULD BE GETTING START POSE FROM AMCL?
        goal.start.pose.position.x = from_x; //LOOK AT THIS LATER - Lucas
        goal.start.pose.position.y = from_y;

        goal.start.pose.orientation.w = 1.0;  // heading doesn't affect length

        goal.goal.header.frame_id  = "map";
        goal.goal.header.stamp     = node->now();
        goal.goal.pose.position.x  = to_x;
        goal.goal.pose.position.y  = to_y;
        goal.goal.pose.orientation.w = 1.0;

        goal.planner_id = "";  // use default planner (GridBased / NavFn)

        // Send goal and wait (blocking, planning-only — no motion)
        auto goal_handle_future = path_planner_client->async_send_goal(goal);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future,
                std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node->get_logger(),
                "compute_path_to_pose: send_goal timed out");
            double dx = to_x - from_x, dy = to_y - from_y;
            return std::sqrt(dx*dx + dy*dy);
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_WARN(node->get_logger(),
                "compute_path_to_pose: goal rejected");
            double dx = to_x - from_x, dy = to_y - from_y;
            return std::sqrt(dx*dx + dy*dy);
        }

        auto result_future = path_planner_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, result_future,
                std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node->get_logger(),
                "compute_path_to_pose: result timed out");
            double dx = to_x - from_x, dy = to_y - from_y;
            return std::sqrt(dx*dx + dy*dy);
        }

        auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(node->get_logger(),
                "compute_path_to_pose: planning failed (unreachable?)");
            return -1.0;
        }

        // Sum arc length of the returned path
        const auto& poses = result.result->path.poses;
        double length = 0.0;
        for (size_t i = 1; i < poses.size(); ++i) {
            double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
            double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
            length += std::sqrt(dx*dx + dy*dy);
        }
        RCLCPP_DEBUG(node->get_logger(),
            "Navigable length (%.2f,%.2f)->(%.2f,%.2f) = %.3f m",
            from_x, from_y, to_x, to_y, length);
        return length;
    };

    // Greedy nearest-navigable-neighbour ordering.
    // Called once from INIT with the real AMCL starting pose.
    // visitOrder[i] = index into boxes.coords for the i-th stop.
    std::vector<int> visitOrder;
    auto buildVisitOrder = [&](double start_x, double start_y) {
        int n = static_cast<int>(boxes.coords.size());
        std::vector<bool> visited(n, false); //TAKE A LOOK AT THIS REINIT EACH FUNC CALL MIGHT CAUSE ISSUES?? - Lucas
        visitOrder.clear();
        visitOrder.reserve(n);

        float offset=0.5; //offset to add to path plan goal so not in box

        double cx = start_x, cy = start_y;

        for (int step = 0; step < n; ++step) {
            int    best     = -1;
            double bestCost = std::numeric_limits<double>::max();

            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                double cost = getNavigablePathLength(
                    cx, cy,
                    boxes.coords[j][0]+(offset*cos(boxes.coords[j][2])), boxes.coords[j][1]+(offset*sin(boxes.coords[j][2])));

                // -1 means unreachable; treat as worst case
                if (cost < 0.0) cost = std::numeric_limits<double>::max();

                if (cost < bestCost) { bestCost = cost; best = j; }
            }

            if (best == -1) {
                //Fallback: just pick the first unvisited box if best stays -1 to prevent seg fault
                for (int j = 0; j < n; ++j) {
                    if (!visited[j]) { best = j; break; }
                }
            }

            visited[best] = true;
            visitOrder.push_back(best);
            cx = boxes.coords[best][0] + (offset*cos(boxes.coords[best][2]));
            cy = boxes.coords[best][1] + (offset*sin(boxes.coords[best][2]));

            RCLCPP_INFO(node->get_logger(),
                "[PathPlan] step %d -> box %d  (x=%.2f, y=%.2f, phi=%.2f)  "
                "navigable dist=%.2f m",
                step, best,
                boxes.coords[best][0],
                boxes.coords[best][1],
                boxes.coords[best][2],
                bestCost);
        }
    };
    // -----------------------------------------------------------------------

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        /***YOUR CODE HERE***/
        //Keeping track of pose consistently 

        //LETS PRINT THESE TO SEE POSE CONSTANTLY?
        //Waiting for AMCL pose to change
        while (x == 0.0 && y == 0.0 && phi == 0.0) {
            rclcpp::spin_some(node); //spinning node in while for AMCL updates
            x=robotPose.x;
            y=robotPose.y;
            phi=robotPose.phi;
            RCLCPP_INFO_THROTTLE(node->get_logger(),
                        *node->get_clock(), 2000,   // log every 2s to avoid spam
                        "INIT: Waiting for valid AMCL pose...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); //Prevent CPU throttling from constant node spinning     
        }

        switch(currentState) //using switch function for FSM. Similar to If/Else but better practice and easier to change according to google
        {
            case RobotState::INIT:
                initial_x=x;
                initial_y=y;
                initial_phi=phi;
                RCLCPP_INFO(node->get_logger(),
                    "Start pose saved. x=%.2f, y=%.2f, phi=%.2f. "
                    "Building navigable visit order...",
                    initial_x, initial_y, initial_phi);

                // ---------------------------------------------------------------
                // Bido's section – build visit order using real navigable distances
                // from the Nav2 global planner (obstacle-aware path lengths).
                // ---------------------------------------------------------------

                //TAKE A LOOK AT THIS -- MAYBE STATIC CASTING TO DOUBLE CAUSING ISSUES?? - Lucas
                buildVisitOrder(
                    static_cast<double>(initial_x),
                    static_cast<double>(initial_y));
                // ---------------------------------------------------------------

                RCLCPP_INFO(node->get_logger(),
                    "Visit order ready (%zu boxes). Transitioning to NAVIGATE_SCENE.",
                    visitOrder.size());
                currentState=RobotState::NAVIGATE_SCENE;
                break;
            
            case RobotState::NAVIGATE_SCENE:
                // ---------------------------------------------------------------
                // Bido's section – navigate to next scene object using the
                // pre-computed nearest-neighbour visit order.
                // ---------------------------------------------------------------
                if (boxCounter < static_cast<int>(visitOrder.size())) //DO WE NEED STATIC_CAST INT HERE? - Lucas
                {
                    /// offset to avoid collision and nav2 failure
                    double offset = 0.5; // m

                    // Retrieve optimised destination from visitOrder
                    int targetIdx = visitOrder[boxCounter];
                    double goal_phi = boxes.coords[targetIdx][2];
                    double goal_x   = boxes.coords[targetIdx][0] + offset * cos(goal_phi);
                    double goal_y   = boxes.coords[targetIdx][1] + offset * sin(goal_phi);


                    RCLCPP_INFO(node->get_logger(),
                        "[NAVIGATE_SCENE] Moving to box %d (visit %d/%zu)  "
                        "x=%.2f, y=%.2f, phi=%.2f",
                        targetIdx, boxCounter + 1, visitOrder.size(),
                        goal_x, goal_y, goal_phi);

                    //Spinning the robot 180 degrees to face back toward the box
                    goal_phi = goal_phi + M_PI;

                    //Normalize the angle to keep it strictly between -PI and +PI (Nav2 prefers this)
                    goal_phi = atan2(sin(goal_phi), cos(goal_phi));

                    // nav.moveToGoal() blocks until Nav2 reports success/failure.
                    bool navSuccess = nav.moveToGoal(goal_x, goal_y, goal_phi);

                    if (navSuccess) {
                        RCLCPP_INFO(node->get_logger(),
                            "[NAVIGATE_SCENE] Box %d reached successfully.", targetIdx);
                             boxCounter++;
                             //staying in navigate scene for testing
                    } else {
                        // Navigation failed (obstacle, timeout, etc.).
                        // Log and skip this box to avoid getting stuck, then
                        // increment the counter so we try the next location.
                        RCLCPP_WARN(node->get_logger(),
                            "[NAVIGATE_SCENE] Failed to reach box %d. Skipping.", targetIdx);
                        boxCounter++;
                        // Stay in NAVIGATE_SCENE to attempt next box on next loop.
                    }
                }

                else
                {
                    // All boxes visited
                    if (!objectPlaced) {
                        RCLCPP_WARN(node->get_logger(),
                            "[NAVIGATE_SCENE] Checked all boxes but didn't place object.");
                    }
                    currentState = RobotState::RETURN_HOME;
                }
                // ---------------------------------------------------------------
                break;

            
            case RobotState::RETURN_HOME:
                // ---------------------------------------------------------------
                // Bido's section – navigate back to the recorded starting pose.
                // ---------------------------------------------------------------
                RCLCPP_INFO(node->get_logger(),
                    "[RETURN_HOME] Navigating back to start: x=%.2f, y=%.2f, phi=%.2f",
                    initial_x, initial_y, initial_phi);

                {
                    bool homeSuccess = nav.moveToGoal(
                        static_cast<double>(initial_x),
                        static_cast<double>(initial_y),
                        static_cast<double>(initial_phi));

                    if (homeSuccess) {
                        RCLCPP_INFO(node->get_logger(),
                            "[RETURN_HOME] Successfully returned to start position.");
                    } else {
                        RCLCPP_WARN(node->get_logger(),
                            "[RETURN_HOME] Could not reach start exactly – proceeding to output.");
                    }
                }
                // ---------------------------------------------------------------

                // Switch state to DONE
                currentState = RobotState::DONE;
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
