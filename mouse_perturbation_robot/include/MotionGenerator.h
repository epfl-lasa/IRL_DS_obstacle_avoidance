#ifndef __MOTION_GENERATOR_H__
#define __MOTION_GENERATOR_H__

#include <fstream>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include <iostream>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Quaternion.h" // the speed in quaternion form
#include <dynamic_reconfigure/server.h> // in python, chagne node parameters
#include "geometry_msgs/PointStamped.h" 
#include <rosserial_mbed/Adc.h>
#include "MouseInterface.h"
#include "mouse_perturbation_robot/MouseMsg.h"
#include "mouse_perturbation_robot/MouseMsgPassIRL.h"
#include "DSObstacleAvoidance.h"
#include <mouse_perturbation_robot/obstacleAvoidance_paramsConfig.h>

#define MAX_XY_REL 350                    // Max mouse velocity [-]
#define MIN_X_REL 200                    // Min mouse velocity used as threshold [-]
#define MIN_Y_REL 200
#define MIN_Z_REL 320                     // Set this threshold to be a large value

#define PERTURBATION_VELOCITY 15.05f      // PErturbation velocity
#define MAX_PERTURBATION_OFFSET 0.1f      // Max perturnation offset [m]
#define MIN_PERTURBATION_OFFSET 0.05f     // Min perturbation offset [m]
#define TARGET_TOLERANCE 0.05f            // Tolerance radius for reaching a target [m]
#define NB_TARGETS 4                      // Number of targets [-]
#define MAX_RHO 8.0f //8
#define MIN_RHO 0.5f
#define MAX_ETA 1.6f //1.6
#define MIN_ETA 0.8f
#define BINARY_INPUT
#define NUM_LIMIT 30000 // 6 --- use the first 5 as training ... then later five to test is converge..

// #define PROTOCAL_DEBUG

// #define DELAY_INTRODUCE 10

#define PROTOCAL_RELEASE_INCREASE // disable the y and z mouse (disable the PROTOCAL_DEBUG) (Experiment)

// #define LISTEN_EEG // Test the brain activity decoder 

// [enable during subject testing]
// #define LISTEN_EEG_OPTI // Test use EEG only at the end of trail, [during the trail, we still use the mouse).

class MotionGenerator 
{

    private:
    //===========================================
    // 1 or 2 for obstacle number
    const int _numObstacle = 2;

    // random generate rho and sf at each end of trails
    // const bool _randomInsteadIRL = true;
    bool _randomInsteadIRL = true;

    // if use iiwa instead of the lwr
    const bool _iiwaInsteadLwr = false;

    // to configer using in my PC or in the kuka lwr PC (the MouseInterface node is not working with kuka lwr PC.)
    const bool _boolSpacenav = 1; // in my PC, do not use the spacenav | while in the lab pc, use the space navigator

    // arduino
    const bool  _useArduino = true; 

    // enable it when "evaluate" the algorithm [random in the large range of rho and sf]
    const bool _randomWholeRange = true;

    //===========================================

    // State phase enum
    // INIT: Initial phase where the user get used to what a clean motion is
    // CLEAN_MOTION: Clean motion phase
    // PAUSE: Phase reached when a target is reached to slow down the robot
    // JERKY_MOTION: PErturbation phase
    enum State {INIT = 0, CLEAN_MOTION = 1, PAUSE = 2 , JERKY_MOTION = 3}; 
    // Target enum
    // There is four targets available. Only A and B are used for the back and forth motion
    enum Target {A = 0, B = 1, C = 2, D = 3};

    // ROS variables
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;
    float dt; // for iiwa, position from velocity

    // Subscribers and publishers declaration
    ros::Subscriber _subRealPose;           // Subscribe to robot current pose
    ros::Subscriber _subRealTwist;          // Subscribe to robot current pose
    ros::Subscriber _subMouse;              // Subscribe to foot mouse data
    ros::Subscriber _subSpaceNav;              // Subscribe to space nav 3d mouse
    ros::Subscriber _subIRL;                // Subscribe to the parameter generating node
    ros::Subscriber _subPositionObs;        // Subscribe to the obstacle position
    ros::Subscriber _subPositionTar;        // Subscribe to the target position
    ros::Subscriber _subMessageEEG;         // EEG
    ros::Subscriber _subMessageWeight;      // Sub the weight from EEG side
    ros::Subscriber _subMessageEEGOpti;
    
    ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
    ros::Publisher _pubDesiredTwist;        // Publish desired twist
    ros::Publisher _pubFeedBackToParameter; // Publish feed back to rho and sf generator
    ros::Publisher _pubMouseMsgIRL;
    ros::Publisher _pubTarPosition;
    ros::Publisher _pubObsPosition;
    ros::Publisher _pubCommand;
    ros::Publisher _pubDebugTrigger;
    ros::Publisher _pubWeights;
    
    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgPositionObs;  // added for position input from another node obs
    geometry_msgs::Pose _msgPositionTar;  // added for position input from another node tar
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    mouse_perturbation_robot::MouseMsg _msgMouse;
    // Messages for trajectory
    geometry_msgs::PoseArray _msgRealPoseArray;
    sensor_msgs::Joy _msgSpacenav;
    std_msgs::Float64MultiArray _msgCommand;
    geometry_msgs::Pose _msgCommandiiwa;
    std_msgs::Float32 _msgWeight;

    // passing the message (mouse)
    mouse_perturbation_robot::MouseMsgPassIRL _msgMouseIRL;

    // End effector state variables
    Eigen::Vector3f _x;      // Current position [m] (3x1)
    Eigen::Vector3f _x0;     // Initial end effector postion (3x1)
    Eigen::Vector3f _v;      // Current end effector velocity [m/s] (3x1)
    Eigen::Matrix3f _wRb;    // Current rotation matrix (3x3)
    Eigen::Vector4f _q;      // Current end effector quaternion (4x1)

    // End effector desired variables
    Eigen::Vector3f _xd;        // Desired position [m] (3x1)
    Eigen::Vector3f _vd;        // Desired velocity [m/s] (3x1)
    Eigen::Vector4f _qd;        // Desired end effector quaternion (4x1)
    Eigen::Vector3f _omegad;    // Desired angular velocity [rad/s] (3x1)

    // Motion variables
    Eigen::Vector3f _xp;                              // Last position in clean motion [m] (3x1)
    Eigen::Vector3f _mouseVelocity;                   // Mouse velocity X,Y data [-]
    Eigen::Matrix<float,3,NB_TARGETS> _targetOffset;  // Position offsets of the targets with respect to initial position [m] (3*NB_TARGETS)
    Eigen::Vector3f _perturbationOffset;              // Perturbation offset [m] (3x1)
    Eigen::Vector3f _perturbationDirection;           // Direction of the perturbation offset (3x1)
    Eigen::Vector3f _motionDirection;                 // Direction of motion (3x1)
    double _initTime;                                 // Initial time for each phase [s]
    double _reachedTime;                              // Time when reaching a target [s]
    double _lastMouseTime;                            // Last time when a mouse command was received [s]  
    double _phaseDuration;                            // Duration of a phase [s]  
    double _pauseDuration;                            // Duration of the pause after reaching a target [s]
    double _minCleanMotionDuration;                   // Min duration of the clean motion phase [s]
    double _maxCleanMotionDuration;                   // Max duration of the clean motion phase [s]
    double _jerkyMotionDuration;                      // Duration of the jerky motion phase [s]
    double _initDuration;                             // Duration of the initial phase [s]
    double _commandLagDuration;                       // Duration for lag between mouse command and robot response [s]
    uint32_t _trialCount;                             // Number of times a target was reached [-]
    uint32_t _perturbationCount;                      // Number of perturbation phases accomplished [-]
    uint8_t _lastMouseEvent;                          // Last mouse event
    uint8_t _errorButtonCounter;                      // Counter for persistence of key press
    //uint8_t _eventLogger;                             // Stores event messages sent to arduino
    int _eventLogger;
    uint8_t _brainLogger;                             // Sending the result of brain decoding
    std_msgs::Int8 _eventLoggerP;

    //Booleans
    bool _firstRealPoseReceived;      // Monitor the first robot pose update
    bool _firstMouseEventReceived;    // Monitor the first mouse event recevied
    bool _firstSpacenavDataReceived;    // Monitor the first mouse event recevied
    bool _stop;                       // Monitor CTRL+C event
    bool _perturbation;               // Monitor the execution of a perturbation phase
    bool _mouseControlledMotion;      // Monitor the use of mouse controlled motion
    bool _mouseInUse;                 // Monitor if the mouse is in use
    bool _perturbationFlag;           // Flag to set whether random perturbations occur
    bool _switchingTrajectories;      // Flag to set whether the obstacle parameters can randomly change
    bool _errorButtonPressed;         // Monitor the keyboard
    bool _ifSentTraj;                 // 
    bool _updateIRLParameter;         // if update parameter from the IRL node, if not then use the one more previous parameter set. 
    bool _obsPositionInput;           // if the obstacle position is decided by the input or not
    bool _recievedObsPositionInput;   // if the obstacle and target position is modified by the other node
    bool _recievedTarPositionInput;
    bool _delayIntroduce;             // if the delay is introduced
    bool _ifWeightEEGReveive;         // 
    bool _boolReverseMsgEEGOpti;      // 

    // Arduino related variables
    int farduino;
    bool trigger_raised;
    ros::Time trigger_begin;

    int _msgEEG;                      // The binary EEG signal
    int _msgEEGOpti;
    int temp_counter_test;

    // iiwa
    Eigen::Vector3f _desiredNextPosition;

    // Other variables
    static MotionGenerator* me;   // Pointer on the instance
    std::mutex _mutex;            // Mutex variable
    std::ofstream _outputFile;    // File used to log data
    State _state;                 // Current state phase
    Target _currentTarget;        // Current target
    Target _previousTarget;       // Previous target
    
    std_msgs::Float32 _msg_para_up;
    Obstacle _obs;
    Obstacle _obs2;
    
    DSObstacleAvoidance obsModulator;

    dynamic_reconfigure::Server<mouse_perturbation_robot::obstacleAvoidance_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<mouse_perturbation_robot::obstacleAvoidance_paramsConfig>::CallbackType _dynRecCallback;

    int _indexx;
    int _indexy;

    int _numOfDemo;
    int _numOfDemoCounter;

    double _rhosfSave [20][2];
    double init_sf;
    double init_rho;

    int _delayInterval;
    geometry_msgs::Pose _msgMouseI;

    std_msgs::String _msgMessageEEG;
    std_msgs::String _msgMessageEEGOpti;

    bool _indicatorRand;

    int _numOfErrorTrails;
    int _numOfCorrectTrails;
    int temp_counter;

  public:
    // Class constructor
    MotionGenerator(ros::NodeHandle &n, double frequency);

        // Initialize node
        bool init();

        // Run node main loop
        void run();

    private:
    // Stop node callback 
        static void stopNodeCallback(int sig);

    // Compute command to be sent to passive ds controller
    void computeCommand();

    // Generate back and forth motion
    void backAndForthMotion();

    // Generate motion between multiple targets based on mouse input
    void mouseControlledMotion();

    // Process mouse events
    void processMouseEvents();

    // Process cursor events
    void processCursorEvent(float relX, float relY, float relZ, bool newEvent);
    
    // Publish data to topics
    void publishData();

    // Publish data to topics
    void logData();

    // Callback to update the real robot twist
    void updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg);    

    // Callback to update the real robot pose
    void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update mouse data
    void updateMouseData(const mouse_perturbation_robot::MouseMsg::ConstPtr& msg);

    // Callback to update the mouse data by use the space navigator
    void updateSpacenavData(const sensor_msgs::Joy::ConstPtr& msg);

    // Callback to update the parameter rho and safety factor
    void updateIRLParameter(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Convert quaternion to rotation matrix
    Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);
    
    // Initialize arduino communication
    void initArduino();

    // Close arduino communication
    void closeArduino();

    // Send value to arduino
    void sendValueArduino(uint8_t value);

    // Dynamic reconfigure callback
    void dynamicReconfigureCallback(mouse_perturbation_robot::obstacleAvoidance_paramsConfig &config, uint32_t level);

    // Detect keyboard presses
    int getch();

    // send message for parameter update
    //void sendMsgForParameterUpdate(const std_msgs::Float32 value);
    void sendMsgForParameterUpdate();

    // send the target and obstable position for marker 
    void sendTarPosition();
    void sendObsPosition(bool if_obs);

    // receive position from the other node
    void subPositionObs(const geometry_msgs::Pose::ConstPtr& msg);

    // reveive the target position
    void subPositionTar(const geometry_msgs::Pose::ConstPtr& msg);

    // EEG signal
    void subMessageEEG(const std_msgs::String::ConstPtr& msg);

    // The posterior probability 
    void subMessageWeight(const std_msgs::String::ConstPtr& msg);

    void subMessageEEGOpti(const std_msgs::String::ConstPtr& msg);

    // Dyncmic reconfigure the rho and eta by mouse
    void changeRhoEta(int indcator);

};


#endif
