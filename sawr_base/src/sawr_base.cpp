/* 
 * Simple Autonomous Wheeled Robot (SAWR)
 * Motor Controller and Odometry  
 *
 * Motion platform controller using two Dynamixel MX-12W servos in "wheel" mode 
 * and a differential drive.
 *
 * Depends on the Dynamixel SDK for Linux which should be installed in /usr/local.
 *
 * Author: Michael McCool, michael.mccool@intel.com
 * Copyright 2016, Intel Corporation
 * License: see ../sawr/LICENSE.md
 */
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "boost/lexical_cast.hpp"
#include <assert.h>

// header files for Dynamixel SDK
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"

// Dynamixel MX-12W control register addresses
const int dxl_mx12w_w_addr_cw_limit = 6; // word, RW, 0 for wheel mode
const int dxl_mx12w_w_addr_ccw_limit = 8; // word, RW, 0 for wheel mode
const int dxl_mx12w_w_addr_max_torque = 14; // word, RW
const int dxl_mx12w_b_addr_alarm_led = 17; // byte, RW
const int dxl_mx12w_b_addr_alarm_shutdown = 18; // byte, RW
const int dxl_mx12w_b_addr_torque_enable = 24; // byte, RW
const int dxl_mx12w_b_addr_led = 25; // byte, RW
const int dxl_mx12w_b_addr_gain_d = 26; // byte, RW
const int dxl_mx12w_b_addr_gain_i = 27; // byte, RW
const int dxl_mx12w_b_addr_gain_p = 28; // byte, RW
const int dxl_mx12w_w_addr_target_speed = 32;  // word, RW
const int dxl_mx12w_w_addr_torque_limit = 34;  // word, RW
const int dxl_mx12w_w_addr_speed = 38; // word, R
const int dxl_mx12w_w_addr_load = 40; // word, R
const int dxl_mx12w_b_addr_voltage = 42; // byte, R
const int dxl_mx12w_b_addr_temp = 43; // byte, R
const int dxl_mx12w_b_addr_moving = 46; // byte, R
const int dxl_mx12w_w_addr_punch = 48; // word, R
const int dxl_mx12w_b_addr_accel_limit = 73; // byte, R

// Maximum speed (Dynamixel servo limit), absolute value
int dxl_mx12w_max_speed = 1023;

// Maximum torque (Dynamixel servo limit)
int dxl_mx12w_max_torque = 1023;

// Maximum acceleration (Dynamixel servo limit)
int dxl_mx12w_max_accel = 255;

// Maximum punch (Dynamixel servo limit)
int dxl_mx12w_max_punch = 1023;

// Scale factor to multiply CPS (MX-12W register value) to get RPS
const double dxl_mx12w_cps_to_rps = (117.07/60.0)/1023.0;  // (r/s)/(c/s)

// Scale factor to multiple RPS to get CPS (MX-12W register value)
const double dxl_mx12w_rps_to_cps = 1023.0/(117.07/60.0);  // (c/s)/(r/s)

using namespace std;

// PARAMETERS
//   --> these should really be given in parameter files, perhaps with
//       support for dynamic updates, with documentation, etc.
//   --> param_ just indicates this is a parameter, external name in YAML file
//       (when supported) should drop prefix
// TODO: Support reading these from parameter server

// After this many seconds, if no new commands, motors will automatically stop (s)
double param_motor_timeout = 1.0;  
// How often odometry is polled and integrated (Hz)
double param_polling_rate = 100.0;    
// Odometry is published on every nth polling loop
int param_odom_publication_divisor = 10;           
// Battery state is published on every nth polling loop
int param_batt_publication_divisor = 10000;           
// Dynamixel ID for left motor (when facing towards the front of the robot)
int param_left_motor_id = 1;       
// Dynamixel ID for right motor (when facing towards the front of the robot)
int param_right_motor_id = 2;      
// Mechanical advantage of transmission
double param_mech = 6.21882;
// Dynamixel baudrate
int param_baudrate = 115200;
// Dynamixel serial port device
//char param_device[] = "/dev/ttyUSB0"; 
char param_device[] = "/dev/ttyACM0"; 
// Tire diameter - minor radius (m)
// double param_tire_diameter = 6.89/1000.0; // mm -> m
double param_tire_diameter = 5.7/1000.0; // mm -> m
// Tire ID - inner diameter (assuming no belt drive)
double param_tire_id = 114.6;
// Diameter of wheels (m) on the outside of tires
double param_wheel_diameter = 125.815/1000.0;  // mm -> m
// Distance between wheel-ground contact points (m)
double param_wheel_base = 181.0/1000.0;  // mm -> m 
// Wheel dihedral angle (degrees; positive is inwards at top)
double param_dihedral = 10;
// P parameter for PID controller (Dynamixel units)
int param_gain_p = 8;
// I parameter for PID controller (Dynamixel units)
int param_gain_i = 0;
// D parameter for PID controller (Dynamixel units)
int param_gain_d = 8;
// Maximum motor velocity (in Dynamixel units; desired, not servo limit)
int param_max_speed = dxl_mx12w_max_speed/4; // it's not a race, avoid smashups
// Maximum motor acceleration (in Dynamixel units; desired, not servo limit))
int param_max_accel = dxl_mx12w_max_accel/8; // avoid being too jerky
// Maximum motor torque (in Dynamixel units; desired, not servo limit))
int param_max_torque = dxl_mx12w_max_torque; // can use all we can get!
// Maximum motor punch (in Dynamixel units; desired, not servo limit))
int param_max_punch = 0; // dxl_mx12w_max_punch/4; 

// DERIVED VALUES
// --> If/when YAML and/or dynamic parameter updates are supported these will have to be
//     recomputed as needed at runtime, which is why they (and the parameters above) are not
//     marked as const.

// Effective wheel diameter (taking into account dihedral angle on wheel and tire)
double effective_wheel_diameter = 
   param_wheel_diameter*cos(2*M_PI*param_dihedral)
   - 2*param_tire_diameter*(1.0-cos(2*M_PI*param_dihedral));
// Wheel circumference (m per revolution)
double wheel_circumference = M_PI*param_wheel_diameter;   
// Effective wheel circumference (m per revolution)
double effective_wheel_circumference = M_PI*effective_wheel_diameter;   
// Base circumference
double base_circumference = M_PI*param_wheel_base;

// Calibration
double param_calibration = 0.8;
//double param_calibration = 0.9;
//double param_calibration = 1.1;

// Amount to scale linear speed values in m/s to get motor speeds in cps
                                                           // -> c/s
double linear_scale = dxl_mx12w_rps_to_cps                 // (c/s)/(r/s)
                    * param_calibration                   //   = c/r
                    * (32.0/254.0)                         // W/A gear ratio
                    * param_mech                           // mechanical advantage of transmission
                    * (1.0/effective_wheel_circumference); // r/m
                                                           // <- m/s
// Amount to scale w values in twist (rad/s) to get motor speeds in cps 
double angular_scale = linear_scale                        //   m/s -> c/s
                     * base_circumference                  // m/R (R = base rev)
                     / (2.0*M_PI);                         // R/rad
                                                           // <- rad/s

// Scales for odom input (in case different from speed targets)
double odom_linear_scale = linear_scale;
double odom_angular_scale = angular_scale;

#include <signal.h>

// Signal handler; need to catch and handle signals to clean up 
// semi-gracefully
int running = 1;
void sig_handler(int signo) {
    if (signo == SIGINT) {
        running = 0;
    }
}
void register_sig_handler() {
    signal(SIGINT, sig_handler);
}

dynamixel::PortHandler *portHandler = 0; 
dynamixel::PacketHandler *packetHandler = 0;

// number of times to repeat a failed command
const int num_tries = 5;

// Dynamixel helper functions
bool motor_write_byte(int id, int addr, unsigned int value) {
    bool success = true;
    assert(0 != packetHandler);
    assert(0 != portHandler);
    uint8_t er;  
    int cr = packetHandler->write1ByteTxRx(
        portHandler, id, 
        addr, uint8_t(0xFF & value), 
        &er
    );
    if (COMM_SUCCESS != cr) {
        // TODO: should send these as strings to ROS logger
        packetHandler->printTxRxResult(cr);
        success = false;
    } else if (0 != er) {
        // TODO: should send these as strings to ROS logger
        packetHandler->printRxPacketError(er);
        success = false;
    }
    return success;
}
bool motor_write_word(int id, int addr, unsigned int value) {
    bool success = true;
    assert(0 != packetHandler);
    assert(0 != portHandler);
    uint8_t er;  
    int cr = packetHandler->write2ByteTxRx(
        portHandler, id, 
        addr, uint16_t(0xFFFF & value), 
        &er
    );
    if (COMM_SUCCESS != cr) {
        // TODO: should send these as strings to ROS logger
        packetHandler->printTxRxResult(cr);
        success = false;
    } else if (0 != er) {
        // TODO: should send these as strings to ROS logger
        packetHandler->printRxPacketError(er);
        success = false;
    }
    return success;
}
bool motor_read_byte(int id, int addr, unsigned int *value) {
    bool success = true;
    assert(0 != packetHandler);
    assert(0 != portHandler);
    uint8_t v;
    uint8_t er;  
    int cr = packetHandler->read1ByteTxRx(
      portHandler, id, 
      addr, &v, 
      &er
    );
    if (COMM_SUCCESS != cr) {
      // TODO: should send these as strings to ROS logger
      packetHandler->printTxRxResult(cr);
      success = false;
    } else if (0 != er) {
      // TODO: should send these as strings to ROS logger
      packetHandler->printRxPacketError(er);
      success = false;
    }
    *value = (unsigned int)(v);
    return success;
}
bool motor_read_word(int id, int addr, unsigned int *value) {
    bool success = true;
    assert(0 != packetHandler);
    assert(0 != portHandler);
    uint16_t v;
    uint8_t er;  
    int cr = packetHandler->read2ByteTxRx(
      portHandler, id, 
      addr, &v, 
      &er
    );
    if (COMM_SUCCESS != cr) {
      // TODO: should send these as strings to ROS logger
      packetHandler->printTxRxResult(cr);
      success = false;
    } else if (0 != er) {
      // TODO: should send these as strings to ROS logger
      packetHandler->printRxPacketError(er);
      success = false;
    }
    *value = (unsigned int)(v);
    return success;
}

// Enable (or disable) motor torque
bool motor_enable(int id, unsigned int en = 1) {
    bool success = true;
    int tries = num_tries;
    while (!motor_write_byte(id,dxl_mx12w_b_addr_torque_enable,en)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
bool motor_disable(int id) {
    return motor_enable(id,0);
}
// Set wheel mode
bool motor_set_wheel_mode(int id) {
    bool success = true;
    int tries = 5;
    while (!motor_write_word(id,dxl_mx12w_w_addr_cw_limit,0) ||
           !motor_write_word(id,dxl_mx12w_w_addr_ccw_limit,0) 
    ) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Set torque limit
bool motor_set_max_torque(int id, unsigned int torque) {
    bool success = true;
    int tries = num_tries;
    while (!motor_write_word(id,dxl_mx12w_w_addr_torque_limit,torque)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Set accleration limit
bool motor_set_max_accel(int id, unsigned int accel) {
    bool success = true;
    int tries = num_tries;
    while (!motor_write_byte(id,dxl_mx12w_b_addr_accel_limit,accel)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Set punch limit
bool motor_set_max_punch(int id, unsigned int punch) {
    bool success = true;
    int tries = num_tries;
    while (!motor_write_word(id,dxl_mx12w_w_addr_punch,punch)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Set PID parameters
bool motor_set_pid(int id, unsigned int p, unsigned int i, unsigned int d) {
    bool success = true;
    int tries = 5;
    while (!motor_write_byte(id,dxl_mx12w_b_addr_gain_p,p) || 
           !motor_write_byte(id,dxl_mx12w_b_addr_gain_i,i) || 
           !motor_write_byte(id,dxl_mx12w_b_addr_gain_d,d)
    ) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Set target speed (using cpr units, but signed; + is CCW rotation)
// also clamps speed to capability of servo
bool motor_set_target_speed(int id, double value) {
    int speed = int(round(value));
    if (speed < 0) {
      speed = dxl_mx12w_max_speed - speed + 1; // this is actually an addition
      if (speed > 2*dxl_mx12w_max_speed) speed = 2*dxl_mx12w_max_speed;
    } else {
      if (speed >= dxl_mx12w_max_speed) speed = dxl_mx12w_max_speed;
    }
    bool success = true;
    int tries = num_tries;
    // ROS_INFO_STREAM("servo "<<id<<" target: "<<value<<"->"<<speed);
    while (!motor_write_word(id,dxl_mx12w_w_addr_target_speed,speed)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    return success; 
}
// Get actual speed (using cpr units, but signed; + is CCW rotation)
bool motor_get_speed(int id, double* value) {
    unsigned int speed;
    bool success = true;
    int tries = num_tries;
    while (!motor_read_word(id,dxl_mx12w_w_addr_speed,&speed)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    if (success) {
      // convert to a signed value
      if (speed > dxl_mx12w_max_speed) {
        *value = -(double(speed) - (dxl_mx12w_max_speed + 1));
      } else {
        *value = double(speed);
      } 
    } else {
      // an error reading speed
      *value = 0.0;
    }
    // ROS_INFO_STREAM("servo "<<id<<" speed: "<<speed<<"->"<<(*value));
    return success;
}
// Get voltage (can be used to check battery level!)
bool motor_get_voltage(int id, double* value) {
    bool success = true;
    int tries = num_tries;
    unsigned int v;
    while (!motor_read_byte(id,dxl_mx12w_b_addr_voltage,&v)) {
      if (0 == --tries) {
        success = false;
        break;
      }
    }
    if (success) {
      // convert to a floating-point value
      *value = double(v)/10.0;
    } else {
      // an error reading value
      *value = 0.0;
    }
    return success;
}
// Set up battery message (but, we only know the voltage, so...)
//typedef boost::shared_ptr< ::sensor_msgs::BatteryState> BatteryStatePtr;
void battery_monitor(sensor_msgs::BatteryStatePtr& batt_msg) {
}

// Odom tf broadcaster and publisher for raw odom data
//  These need to be set up as pointers since we can't create them yet,
//  but need to access them in a callback.  Boost pointers are not needed
//  as we handle the cleanup explictly.
tf::TransformBroadcaster* odom_broadcaster = 0;
ros::Publisher* odom_pub = 0;

// Cmd_vel subscribers.  
//  As above, set up as pointers.  Multiple subscribers support
//  for multiple control channels, in particular a teleop and an autonomous
//  navigator, to operate at once.  This way we don't need an external
//  "mux" (although we could add one later, if we want).
ros::Subscriber* cmd_vel_sub = 0;
ros::Subscriber* nav_cmd_vel_sub = 0;

// Battery level publisher
//  Uses fact that battery drives servos directly.  Average of voltage at all
//  servos can be taken to increase accuracy.  Useful to monitor LiPo batteries.
ros::Publisher* batt_pub = 0;

// Flag to know if a timeout is pending
bool motor_timer_active = false;

// Last time we got a twist event
ros::Time last_twist_event_time;

// Callback for *cmd_vel "Twist" subscriptions
void twist_callback(
    const geometry_msgs::TwistConstPtr& twist_ptr
) {
    // Get twist data (note: we ignore vy!)
    double vx = twist_ptr->linear.x;
    double vw = twist_ptr->angular.z;

    // Reset timer and report twist input only if some element non-zero
    if (abs(vx) + abs(vw) > 0) {
        motor_timer_active = true;
        last_twist_event_time = ros::Time::now();
        ROS_INFO_STREAM("twist ("<<vx<<",[0];"<<vw<<")");
    }

    // Transform into 2WD differential motor velocities
    vx *= linear_scale;   // m/s to rpm to motor speed in cpi units
    vw *= angular_scale;  // rad/s to rpm to motor speed in cpi units
    double v1 =  vx - vw;
    double v2 = -vx - vw; 

    // Find maximum absolute value
    double mv = std::max(abs(v1),abs(v2));

    // Scale velocities down uniformly if too high
    if (mv > param_max_speed) {
        v1 = (v1 / mv) * param_max_speed;
        v2 = (v2 / mv) * param_max_speed;
        // Report scaled motor output
        ROS_INFO_STREAM("motor (scaled down) {"<<v1<<":"<<v2<<"} <max "<<mv<<">");
    } else {
        // Report motor output
        if (abs(v1) + abs(v2) > 0) {
           ROS_INFO_STREAM("motor {"<<v1<<":"<<v2<<"}");
        }
    }

    // Update motor speed setpoints
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_target_speed(param_left_motor_id,v1);
    (void)motor_set_target_speed(param_right_motor_id,v2);
}

int main (
    int argc, 
    char** argv
) {
    // ROS init
    ros::init(argc, argv, "sawr_base");  //node_name
    ros::NodeHandle nh;
    ROS_INFO_STREAM("sawr_base started");

    // DXL init
    portHandler = dynamixel::PortHandler::getPortHandler(param_device);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);

    // Set communications rate (motors should be configured in advance to match)
    portHandler->setBaudRate(param_baudrate);

    // Set to Wheel mode
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_wheel_mode(param_left_motor_id);
    (void)motor_set_wheel_mode(param_right_motor_id);

    // Set max torque
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_max_torque(param_left_motor_id,param_max_torque);
    (void)motor_set_max_torque(param_right_motor_id,param_max_torque);

    // Set max accel
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_max_accel(param_left_motor_id,param_max_accel);
    (void)motor_set_max_accel(param_right_motor_id,param_max_accel);

    // Set PID parameters
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_pid(param_left_motor_id,param_gain_p,param_gain_i,param_gain_d);
    (void)motor_set_pid(param_right_motor_id,param_gain_p,param_gain_i,param_gain_d);

    // Set initial target speed to zero
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_target_speed(param_left_motor_id,0);
    (void)motor_set_target_speed(param_right_motor_id,0);

    // Enable DXL Torque on both motors
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_enable(param_left_motor_id);
    (void)motor_enable(param_right_motor_id);

    // Arrange to exit cleanly 
    register_sig_handler(); 

    // PUBLISHERS

    // Create tf broadcaster for odom frame
    odom_broadcaster = new tf::TransformBroadcaster();

    // Publish "odom" topic 
    odom_pub = new ros::Publisher(nh.advertise<nav_msgs::Odometry>(
        "odom", 
        100          // queue size
    ));     

    // Publish "battery_state" topic 
    batt_pub = new ros::Publisher(nh.advertise<sensor_msgs::BatteryState>(
        "battery_state", 
        100          // queue size
    ));     
    // initialize template
    sensor_msgs::BatteryState batt_msg;
    batt_msg.present = true;
    batt_msg.location = "0";
    batt_msg.serial_number = "none";
    batt_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

    // SUBSCRIBERS

    // Subscribe to *cmd_vel "Twist" topics 
    //   => these both use the same callback.  
    //   => we do not do any prioritization (yet)
    cmd_vel_sub = new ros::Subscriber(nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 
        100,         // queue size  
        &twist_callback
    ));
    nav_cmd_vel_sub = new ros::Subscriber(nh.subscribe<geometry_msgs::Twist>(
        "nav_cmd_vel", 
        100,         // queue size  
        &twist_callback
    ));

    // TODO: should really make sure none of the above calls return errors 

    // Get starting timestamp
    ros::Time current_time, last_time;
    last_time = ros::Time::now();
    current_time = ros::Time::now(); // TODO: wait until this is not 0

    // Pretend we just got a twist event to avoid immediate timeout
    last_twist_event_time = current_time;

    // Integration variables for odometry
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;

    // Spin and wait for stuff to come in
    ros::Rate r(param_polling_rate); 
    int odom_publication_counter = param_odom_publication_divisor;
    int batt_publication_counter = param_batt_publication_divisor;
    while (ros::ok() && running) {
        // yield to ROS once per cycle
        ros::spinOnce();

        // keep track of iterations
        odom_publication_counter--;
        batt_publication_counter--;

        // Grab new timestamp
        ros::Time current_time = ros::Time::now();
        // Compute time difference
        double dt = (current_time - last_time).toSec();

        // MOTOR CONTROL TIMEOUT

        // Stop motors if motor timeout has expired.  This
        // prevents the motors running forever if communication is lost.
        if (motor_timer_active &&
            (current_time.toSec() > last_twist_event_time.toSec() + param_motor_timeout)) {
          motor_timer_active = false;
          motor_set_target_speed(param_left_motor_id,0.0);
          motor_set_target_speed(param_right_motor_id,0.0);
          ROS_INFO_STREAM("motor command timeout"); // a warning instead?

          // To avoid doing this every time around the polling loop for 
          // no reason, reset timer, as if this were an actual command.
          // ideally we would just check if the input command queue is empty, but...
          last_twist_event_time = current_time;
        }

        // ODOMETRY 

        // Get current (measured, actual) speed from each motor
        // TODO: deal with case that calls fail, at least by sending out ROS error message
        double s1, s2;
        (void)motor_get_speed(param_left_motor_id,&s1);
        (void)motor_get_speed(param_right_motor_id,&s2);

        // Transform into linear and angular speeds in m/s and rad/s
        double vw = -0.5*(s1 + s2);
        double vx =  0.5*(s1 - s2);
        vx /= odom_linear_scale;
        vw /= odom_angular_scale;

        // Print velocity *occasionally*, if it is non-zero, for debug purposes
        if (0 == odom_publication_counter) {
            if (abs(vx) + abs(vw) > 0) {
                ROS_INFO_STREAM("base vel ["<<s1<<","<<s2<<"] -> ("<<vx<<";"<<vw<<") over "<<dt<<"s");
            }
        }

        // Integrate odometry
        //   Differential drive, so vy always zero in local frame.
        //   However, velocity needs to be rotated by current orientation to 
        //   map into odometry frame.
        double dx = cos(w)*vx*dt;
        double dy = sin(w)*vx*dt;
        double dw = vw*dt;
        x += dx;
        y += dy;
        w += dw;

        // Output odometry at decimated rate
        if (0 == odom_publication_counter) {
            odom_publication_counter = param_odom_publication_divisor; 

            // Print updated odom pose (if it changes), for debug purposes
            if (abs(vx) + abs(vw) > 0) {
               ROS_INFO_STREAM("odom pose (" << x << "," << y << ";" << w << ")");
            }
 
            // All odometry is 6DOF so need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(w);
 
            // Create an odometry message 
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            //odom.child_frame_id = "base_footprint";
 
            // Set the accumulated pose as a 3D (6DOF) value
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            // Set the velocity as a twist relative to the child frame
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = vw;

            // Publish odom message 
            odom_pub->publish(odom);

            // Echo header data from odom message to odom transform header
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = odom.header.stamp; 
            odom_trans.header.frame_id = odom.header.frame_id; 
            odom_trans.child_frame_id = "base_footprint";
 
            // Echo pose data from odom message to odom transform
            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;
            odom_trans.transform.translation.z = odom.pose.pose.position.z;
            odom_trans.transform.rotation = odom.pose.pose.orientation;
 
            // Broadcast the odom transform  
            odom_broadcaster->sendTransform(odom_trans);
        }

        // Output battery state at decimated rate
        if (0 == batt_publication_counter) {
            batt_publication_counter = param_batt_publication_divisor; 

            // Update battery state message
            double voltage_1, voltage_2;
            (void)motor_get_voltage(param_left_motor_id,&voltage_1);
            (void)motor_get_voltage(param_right_motor_id,&voltage_2);
            batt_msg.voltage = 0.5*(voltage_1 + voltage_2); 
            batt_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            if (batt_msg.voltage < 3.2*3) 
                batt_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            if (batt_msg.voltage > 4.2*3) 
                batt_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;

            // Print updated battery voltage
            ROS_INFO_STREAM("battery voltage: " << batt_msg.voltage);
 
            // Finalize and publish battery state message 
            batt_msg.header.stamp = current_time;
            batt_pub->publish(batt_msg);
        }

        // Update the timestamp for next time, then sleep
        last_time = current_time;
        r.sleep();
    }

    // SHUTDOWN

    // Power down and disable motors
    // TODO: deal with case that calls fail, at least by sending out ROS error message
    (void)motor_set_target_speed(param_left_motor_id,0.0);
    (void)motor_set_target_speed(param_right_motor_id,0.0);
    (void)motor_enable(param_left_motor_id,0);
    (void)motor_enable(param_right_motor_id,0);

    // Close DXL port
    portHandler->closePort();

    // Delete ROS publishers and subscribers
    delete cmd_vel_sub; 
    cmd_vel_sub = 0;
    delete nav_cmd_vel_sub; 
    nav_cmd_vel_sub = 0;

    delete odom_pub; 
    odom_pub = 0; 
    delete odom_broadcaster;
    odom_broadcaster = 0;

    return 0;
}

