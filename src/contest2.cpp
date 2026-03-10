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
    YoloInterface yolo(node);
    ArmController arm(node);
    AprilTagDetector tag_detector(node, "base_link"); // use base_link so AprilTag pose is in robot frame

    // ── Manipulable object coordinates in arm base frame ──────────────────────
    // These values are provided by the TAs on contest day.
    // Update them here (or load from a param/XML) before running.
    const double OBJ_ARM_X = 0.15;   // metres — forward from arm base
    const double OBJ_ARM_Y = 0.00;   // metres — lateral
    const double OBJ_ARM_Z = 0.05;   // metres — height above arm base (top plate surface)

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

    // Index of the scene object that matched the manipulable object (set in CHECK_MATCH)
    int matchedBoxIndex = -1;

    // ── Manipulable object coordinates in arm_mount (base) frame ──────────────
    // *** REPLACE THESE with the values provided by the TAs on contest day ***
    const double MANIP_OBJ_X = 0.15;   // metres — forward from arm mount
    const double MANIP_OBJ_Y = 0.00;   // metres — lateral offset
    const double MANIP_OBJ_Z = 0.05;   // metres — height (top-plate level)

    // ── Fixed offset: robot base_link → arm_mount (from URDF) ────────────────
    // Used to convert AprilTag world-frame pose → arm base frame.
    // *** Verify against so101.urdf ***
    const double ARM_MOUNT_FORWARD = 0.10;  // metres forward of base centre
    const double ARM_MOUNT_UP      = 0.30;  // metres above ground

    // Struct for returned values from YOLO detection in helper below
    struct YoloDetection {
        std::string name;
        float confidence;
        float x;
        float y;
        float phi;
        bool valid;
    };

    // Scene detections (5 objects in maze)
    std::vector<YoloDetection> sceneDetections;
    sceneDetections.reserve(5);

    // YOLO cooldown (1s)
    auto lastYoloTime = std::chrono::steady_clock::time_point::min();

    // Helper for YOLO detection in FSM cases (Nick's sections)
    auto runYoloDetection = [&](CameraSource camera, bool save_image) -> YoloDetection {
        auto now = std::chrono::steady_clock::now();
        if (now - lastYoloTime < std::chrono::seconds(1)) {
            RCLCPP_INFO(node->get_logger(), "YOLO: cooldown active, skipping detection");
            return {"", -1.0f, x, y, phi, false};
        }
        lastYoloTime = now;

        // Save detection
        std::string detected = yolo.getObjectName(camera, save_image);

        // Get detection confidence
        float confidence = yolo.getConfidence();

        // Warn about no valid detection or negative confidence
        if (detected.empty() || confidence < 0.0f) {
            RCLCPP_WARN(node->get_logger(), "YOLO: no valid detection");
            return {"", -1.0f, x, y, phi, false};
        }

        // Store, print, and return detection
        yoloConfidenceScore = confidence;
        RCLCPP_INFO(node->get_logger(), "YOLO: %s (%.2f)", detected.c_str(), confidence);
        return {detected, confidence, x, y, phi, true};
    };

    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        
        /***YOUR CODE HERE***/
        //Keeping track of pose consistently 
        x=robotPose.x;
        y=robotPose.y;
        phi=robotPose.phi;

        switch(currentState) //using switch function for FSM. Similar to If/Else but better practice and easier to change according to google
        {
            case RobotState::INIT: //case is pretty much the "if". Here it implies if currentState==INIT
                initial_x=x;
                initial_y=y;
                initial_phi=phi;
                //outputting initial pose
                RCLCPP_INFO(node->get_logger(), "Start pose saved. x=%.2f, y=%.2f, phi=%.2f. Transitioning to Object Pickup", initial_x, initial_y, initial_phi);
                currentState=RobotState::PICKUP_OBJECT; //this transitions FSM to next state to pick up the object off the robot
                break; //uses break to ensure you do not just go to the next case
            
            case RobotState::PICKUP_OBJECT: //code to pick up object right away so it does not fall when moving
                // Nick + Ahmed's section

                // NOTE: may want fallback here for if pickup fails. Set targetObject to some default and retry to
                // pickup object a certain amount of times maybe

                RCLCPP_INFO(node->get_logger(), "Detecting and picking up object.");
                // TODO: (Nick) Call yolo.getObjectName(CameraSource::WRIST, true) and store yolo object name
                YoloDetection det = runYoloDetection(CameraSource::WRIST, true); // save image of manipulable object
                if (!det.valid) {
                    // Try again next loop if detection is invalid
                    break;
                }

                // If detection is valid, store object name and confidence (latched on first valid)
                if (!manipulableObjectLatched) {
                    manipulableObjectName = det.name;
                    manipulableObjectConfidence = det.confidence;
                    manipulableObjectLatched = true;

                    // Also set targetObject to Manipulable object
                    targetObject = manipulableObjectName;
                }

                // Ahmed's section — pick sequence using arm_controller
                {
                    // Step 1: Open gripper before approaching
                    if (!arm.openGripper()) {
                        RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to open gripper — retrying next loop");
                        break;
                    }

                    // Step 2: Move to pre-grasp pose (hover above object, gripper pointing down)
                    //         pitch = PI/2 orients the gripper straight down
                    bool pre_grasp_ok = arm.moveToCartesianPose(
                        OBJ_ARM_X,
                        OBJ_ARM_Y,
                        OBJ_ARM_Z + 0.08,   // 8 cm above object centre
                        0.0, M_PI / 2.0, 0.0
                    );
                    if (!pre_grasp_ok) {
                        RCLCPP_ERROR(node->get_logger(), "PICKUP: Pre-grasp pose unreachable — retrying next loop");
                        break;
                    }

                    // Step 3: Descend straight down to grasp height
                    bool grasp_ok = arm.moveToCartesianPose(
                        OBJ_ARM_X,
                        OBJ_ARM_Y,
                        OBJ_ARM_Z,
                        0.0, M_PI / 2.0, 0.0
                    );
                    if (!grasp_ok) {
                        RCLCPP_ERROR(node->get_logger(), "PICKUP: Grasp pose unreachable — retrying next loop");
                        break;
                    }

                    // Step 4: Close gripper to grip the object
                    if (!arm.closeGripper()) {
                        RCLCPP_ERROR(node->get_logger(), "PICKUP: Failed to close gripper — retrying next loop");
                        break;
                    }

                    // Brief pause to confirm grip is stable
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));

                    // Step 5: Lift object up to safe carry height
                    bool lift_ok = arm.moveToCartesianPose(
                        OBJ_ARM_X,
                        OBJ_ARM_Y,
                        OBJ_ARM_Z + 0.15,   // 15 cm carry height clears obstacles
                        0.0, M_PI / 2.0, 0.0
                    );
                    if (!lift_ok) {
                        RCLCPP_WARN(node->get_logger(), "PICKUP: Lift failed — object may still be gripped, continuing");
                    }

                    objectInArm = true;
                    RCLCPP_INFO(node->get_logger(), "PICKUP: Object successfully picked up!");
                }

                //*****once the object is picked up set objectInArm=true;
                if(objectInArm) //moving on to the next state if object is in the arm
                {
                    currentState=RobotState::NAVIGATE_SCENE;
                    break;
                }
                else //if object not in arm we repeat this state until we get it.
                {
                    //Could be good to have some fallback code here. Like reverse 0.2 metres then try again in case object dropped
                    break;
                }
            
            case RobotState::NAVIGATE_SCENE:
                //Bido's section
                //probably best to define some functions outside here then call them tbh. Just so this doesn't get too bloated
                
                if(boxCounter<boxes.coords.size()) //if statement to make sure we check all boxes
                {
                    //add navigation code here. Gemini TODO: nav.moveToGoal(boxes.coords[current_box_index][0], ...)

                    //once navigation to box is done switch state to detecting object (YOLO)
                    //if(navIsDone)
                    RCLCPP_INFO(node->get_logger(), "Box successfully reached! Transitioning to YOLO detection");
                    currentState=RobotState::DETECT_SCENE_OBJECT;
                    break;
                }
                else //if we have hit every box then we return to start. Switching state to return to start
                {
                    if(!objectPlaced)//if statement just to let us know in terminal if object was successfully placed
                    {
                        RCLCPP_WARN(node->get_logger(), "Checked all boxes but didn't place object.");
                    }
                    currentState=RobotState::RETURN_HOME
                }
                break;

            case RobotState::DETECT_SCENE_OBJECT:
                //Nick's section
                // TODO: (Nick) Call yolo.getObjectName(CameraSource::OAKD, true) and store yolo object name
                RCLCPP_INFO(node->get_logger(), "Detecting scene object.");
                
                {
                    YoloDetection det = runYoloDetection(CameraSource::OAKD, false); // no scene object images saved
                    if (!det.valid) {
                        // Try again next loop if detection is invalid
                        break;
                    }

                    // Store scene object in vector if space left
                    yoloObjectName = det.name;
                    yoloConfidenceScore = det.confidence;
                    if (sceneDetections.size() < 5) {
                        sceneDetections.push_back(det);
                    }
                }

                //Once a valid object is identified switch states to check match      
                RCLCPP_INFO(node->get_logger(), "Object Identified");
                currentState = RobotState::CHECK_MATCH;
                break;
            
            case RobotState::CHECK_MATCH:
                //Nick's section also - check if object matches what we need 
                //template I used but feel free to change below
                if(yoloObjectName==targetObject) //if there is a match go to placing object state
                {
                    RCLCPP_INFO(node->get_logger(), "Object Matches!");
                    currentState=RobotState::PLACE_IN_BIN;
                    boxCounter++; //incrementing the boxCounter to get ready to move on after object placement
                }
                else //otherwise we continue onwards to the next bin immediately
                {
                    boxCounter++;
                    currentState=RobotState::NAVIGATE_SCENE;
                }
                break;
            
            case RobotState::PLACE_IN_BIN: //this section to localize bin with AprilTag and then place object
                //Ahmed's section as well
                RCLCPP_INFO(node->get_logger(), "Placing object in bin");

                // Ahmed's section — AprilTag localisation + arm place sequence
                {
                    // Step 1: Identify the AprilTag on the bin in front of the current scene object.
                    //         Tag IDs are assumed to match the box index (0-indexed).
                    //         Adjust tag_id mapping if the TAs use a different numbering scheme.
                    int tag_id = boxCounter - 1; // boxCounter was already incremented in CHECK_MATCH
                    std::vector<int> candidates = {tag_id};

                    RCLCPP_INFO(node->get_logger(), "PLACE: Looking for AprilTag ID %d ...", tag_id);

                    std::vector<int> visible = tag_detector.getVisibleTags(candidates, 2000);
                    if (visible.empty()) {
                        RCLCPP_WARN(node->get_logger(), "PLACE: AprilTag %d not visible yet — retrying next loop", tag_id);
                        break; // stay in PLACE_IN_BIN, try again
                    }

                    // Step 2: Get bin pose in base_link frame from the AprilTag
                    auto bin_pose_opt = tag_detector.getTagPose(tag_id, 2000);
                    if (!bin_pose_opt.has_value()) {
                        RCLCPP_WARN(node->get_logger(), "PLACE: Could not get pose for tag %d — retrying", tag_id);
                        break;
                    }
                    geometry_msgs::msg::Pose bin_pose = bin_pose_opt.value();

                    RCLCPP_INFO(node->get_logger(),
                        "PLACE: Bin localised at base_link (%.3f, %.3f, %.3f)",
                        bin_pose.position.x, bin_pose.position.y, bin_pose.position.z);

                    // Step 3: Convert bin pose (base_link frame) → arm base frame.
                    //         The AprilTag is mounted on the front face of the bin.
                    //         We target the bin opening centre: offset 5 cm back from the tag face.
                    double bin_arm_x = bin_pose.position.z - 0.05;   // depth forward (camera Z = arm X)
                    double bin_arm_y = -bin_pose.position.x;          // lateral (camera X = -arm Y)
                    double bin_arm_z =  bin_pose.position.y + 0.08;   // bin top rim height

                    // Step 4: Move to pre-place hover (above bin opening)
                    bool pre_place_ok = arm.moveToCartesianPose(
                        bin_arm_x,
                        bin_arm_y,
                        bin_arm_z + 0.10,   // 10 cm above bin rim
                        0.0, M_PI / 2.0, 0.0
                    );
                    if (!pre_place_ok) {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Pre-place pose unreachable — retrying");
                        break;
                    }

                    // Step 5: Lower object into bin
                    bool lower_ok = arm.moveToCartesianPose(
                        bin_arm_x,
                        bin_arm_y,
                        bin_arm_z,
                        0.0, M_PI / 2.0, 0.0
                    );
                    if (!lower_ok) {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Could not lower into bin — retrying");
                        break;
                    }

                    // Step 6: Open gripper to release object
                    if (!arm.openGripper()) {
                        RCLCPP_ERROR(node->get_logger(), "PLACE: Gripper release failed — retrying");
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(400));

                    // Step 7: Retract arm back to carry height (clear of bin)
                    arm.moveToCartesianPose(
                        bin_arm_x,
                        bin_arm_y,
                        bin_arm_z + 0.15,
                        0.0, M_PI / 2.0, 0.0
                    );

                    objectPlaced = true;
                    objectInArm  = false;
                    RCLCPP_INFO(node->get_logger(), "PLACE: Object successfully placed in bin!");
                }

                //objectPlaced=true;
                if (objectPlaced)//once its in the bin we continue to next box
                {
                    currentState=RobotState::NAVIGATE_SCENE;
                    break;
                }
                else
                {
                    //add some fallback code to try again
                    break;
                }
            
            case RobotState::RETURN_HOME:
                //Bido's section also
                RCLCPP_INFO(node->get_logger(), "Returning to start coordinates");
                //gemini recommended: TODO: nav.moveToGoal(start_x, start_y, start_phi)


                currentState = RobotState::WRITE_OUTPUTS; //last state to save all the data to txt file. We could do this as we go instead
                break;

            case RobotState::WRITE_OUTPUTS: //state to save txt file. Might move this cause if we don't finish it won't write the data
                RCLCPP_INFO(node->get_logger(), "Writing output files...");
                // Write txt file with manipulable object info and all scene objects + locations
                {
                    std::ofstream out("/home/nicolas-rebollo/YOLO Images/contest2_output.txt");
                    if (!out.is_open()) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to open contest2_output.txt for writing");
                    } else {
                        out << "Pickup: " << manipulableObjectName << " (" << manipulableObjectConfidence << ")\n";
                        out << "Scene Objects:\n";
                        for (size_t i = 0; i < sceneDetections.size(); ++i) {
                            const auto& d = sceneDetections[i];
                            out << i << ": " << d.name << " (" << d.confidence << ") @ x="
                                << d.x << " y=" << d.y << " phi=" << d.phi << "\n";
                        }
                    }
                }

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