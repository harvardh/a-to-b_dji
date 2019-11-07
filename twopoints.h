//
//  twopoints.h
//  twopoints
//
//  Created by Harvard Virgil Humphrey on 2019/11/6.
//  Copyright © 2019 Harvard Virgil Humphrey. All rights reserved.
//

#ifndef twopoints_h
#define twopoints_h
#include <thread>
#include <chrono>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <qualisys/Subject.h>
//#include <state_graph_builder/graph.h>/
//#include <state_graph_builder/posegraph.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <deque>
#include <string>
#include <algorithm>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

typedef struct ServiceAck{ // Struct type for the DJI ROS services
  bool result;
  int cmd_set;
  int cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack):
      result(res), cmd_set(set), cmd_id(id), ack_data(ack) {};
  ServiceAck() {};
}ServiceAck;

struct vector2D { // Makeshift 2D vector
    double x;
    double y;
};

vector2D transform2D(vector2D input,double angle);

struct vector3D { // Makeshift 3D Vector
    double x;
    double y;
    double z;
};

struct missionplan  {
    int mode;
    int nwaypoints;
    std::vector<vector3D> waypts;
};
struct pose { // State vector for all ROS operations in Cartesian coordinates
    std_msgs::Header header;
    double x;
    double y;
    double z;
    double theta;
};

struct cmd_input    { // U vector for all ROS operations in ENU reference frame
    double EAST;
    double NORTH;
    double UP;
    double yaw;
    uint8_t flag;
};
struct controller   { // Controller gain bank
    double xygain;
    double vgain;
    double yawgain;
};

class mission   { // Mission class, contains all parameters important to the mission. Does not consist of important drone information
public:
    missionplan flightplan;
    int phase; // Phase of the journey, based on number of waypoints
    bool qualisys_health; // Motion-capture health boolean. Evaluated each attempt to calculate control input
    int gps_health; // GPS health (0-5) For GPS state to work, health needs to be above 3.
    int nwaypoints; // Total number of waypoints
    double datum_angle; // Datum angle of M-Air
    pose delta_state; // Struct for the error between target and current state
    pose gps_state; // Current state under GPS measurements
    pose qualisys_state; // Current state under motion-capture measurement
    pose target; // Target state
    ros::Time phase_start; // Start time of current journey phase
    ros::Time mission_start; // Start time of overall journey
    ros::Time linger_start;
    ros::Duration linger_time;
    bool waypoint_reached; // Establishes whether the waypoint has been reached
    bool mission_started; // Whether mission has commenced (not to be tampered with after it is made TRUE)
    bool phase_started; // Whether a phase is currently in operation
    bool mission_complete; // evaluates whether whole mission is completed (Landing sequence commences upon this becoming TRUE)
    // Mission methods
    mission();
    ~mission();
    void setTarget(double x,double y,double z, double yaw); // Reassign current target
    void reset_phase(); // Reset all booleans when we enter new phase
    void initiate_phase(); // Set the "started" boolean to true and record the time
    void start_mission(); // Set the "mission_started" boolean to true and record the mission start time
    void evaluate(); // Evaluate error (most important mission function)
};

struct drone {
    int flight_status; // 
    bool is_M100; // Whether the aircraft is the m100 model
    bool airborne;
    cmd_input control_input; // Control input struct

    // Acks
    ServiceAck motors_armed; // Information on whether motors are armed
    bool local_ref; // Information on whether local reference has been set
    ServiceAck in_control; // Information on whether control authority is established by ROS
    ServiceAck takeoff_land; // Information on whether we are doing take off or landing    
};
class twopoints  {
public:
    twopoints();
    ~twopoints();
private:
    mission twopoints_mission; // Create object of type mission
    drone m100; // Drone struct for all drone-related information
    double datum_angle;
    int n;
    double distance;
    double target_height;
    // Controller
    controller gainstruct;  // Gain bank (needs defining in constructor)
    // ROS Handle
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_;
    
    // ROS messages
    //------ Subscriber Messages -----
    sensor_msgs::NavSatFix current_gps;
    geometry_msgs::PointStamped current_ENU;
    sensor_msgs::NavSatFix origin_gps;
    geometry_msgs::Quaternion current_atti;
    qualisys::Subject qualisys_msg; // Needs changing when we establish qualisys message type
    // ------ Publisher Messages -----
    sensor_msgs::Joy control_msg;
    
    // ROS Services
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_arm_service;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient query_version_service;
    ros::ServiceClient drone_task_service;
    
    // ROS Subcribers
    ros::Subscriber qualisysSub;
    ros::Subscriber gpsSub;
    ros::Subscriber attitudeSub;
    ros::Subscriber gps_healthSub;
    ros::Subscriber local_posSub;
    ros::Subscriber flight_statusSub;
    
    //ROS Publishers
    ros::Publisher dji_pub;
    
    //Publisher rates
    ros::Timer cmd_timer;
    
    // Callback functions
    // -----Subscriber callbacks--------
    void mocap_pos_callback(const qualisys::Subject::ConstPtr& msgs);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
    void local_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
    void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
    //------Publisher callbacks--------
    void cmd_dji_callback(const ros::TimerEvent& event);
    
    // Service operations
    ServiceAck armMotors();
    ServiceAck disArmMotors();
    ServiceAck obtainCtrlAuthority();
    ServiceAck releaseCtrlAuthority();
    ServiceAck takeoff();
    ServiceAck goHome();
    ServiceAck land();
    bool set_local_pos_ref();
    bool is_M100();

    // Initialisation and Shutdown
    bool M100monitoredTakeoff();
    bool M100monitoredLand();

    // State calculation functions
    void get_localgps_state();
    void get_gps_heading();
    void get_mocap_health();
    void calculate_u(); 
};

#endif /* twopoints_h */
