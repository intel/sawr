/* Base Bridge
 *
 * Republish topics to/from slave to make them visible to external
 * computers without direct access to the IP-over-USB network.  This also takes
 * care of interfacing to tf so the slave does not have to send that over
 * the USB link, and provides an abstraction layer in case we want to 
 * use a different base platform later.
 * => Note: a network bridge would also work here, and would be "better"
 *    in that it would allow full access to all ROS nodes on the Edison,
 *    but this has the advantage of working independent of the OS setup.
 *    It will also still work (and be useful for reducing bandwidth and 
 *    providing an abstraction layer) even if there is such a "real" 
 *    network bridge. 
 * => By convention, all topics and messages on the "real" base platform
 *    implemented on the Edison should use the topic prefix "fetchbase_"
 *    and none on the UP should.   
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "boost/lexical_cast.hpp"

using namespace std;

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

// Odom tf broadcaster, and pub and sub for raw odom data
//  These need to be set up as pointers since we can't create them yet,
//  but need to access them in a callback.  Boost pointers are not needed
//  as we handle the cleanup explictly.
tf::TransformBroadcaster* odom_broadcaster = 0;
ros::Publisher* odom_pub = 0;
ros::Subscriber* fetchbase_odom_sub = 0;

// Cmd_vel pub/sub.  
//  As above, set up as pointers.  There is one publisher (subscribed
//  to by the Edison) but multiple subscribers on the UP.  This allows
//  for multiple control channels, such as a teleop and an autonomous
//  navigator, to operate at once.  This way we don't need an external
//  "mux" (although we could add one later, if we want).
ros::Publisher* fetchbase_cmd_vel_pub = 0;
ros::Subscriber* cmd_vel_sub = 0;
ros::Subscriber* nav_cmd_vel_sub = 0;

// Tme-stamped transform to send over tf
bool odom_trans_init = false;
geometry_msgs::TransformStamped odom_trans;

// Callback for fetchbase_odom "Odometry" subscription
void fetchbase_odom_callback(
   const nav_msgs::OdometryConstPtr& fetchbase_odom_msg
) {
   ros::Time current_time = ros::Time::now();
   // Copy to make some tweaks
   nav_msgs::Odometry new_odom_msg(*fetchbase_odom_msg);
   new_odom_msg.header.stamp = current_time; // some lag, but avoids time sync issues
   new_odom_msg.header.frame_id = "base_footprint"; 
   // Republish 
   odom_pub->publish(new_odom_msg);

   // Set up header for tf
   // odom_trans.header.stamp = fetchbase_odom_msg->header.stamp; // times need to be well synched!
   odom_trans.header.stamp = current_time; // some lag, but avoids time sync issues
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_footprint"; 
 
   odom_trans.transform.translation.x = fetchbase_odom_msg->pose.pose.position.x;
   odom_trans.transform.translation.y = fetchbase_odom_msg->pose.pose.position.y;
   odom_trans.transform.translation.z = fetchbase_odom_msg->pose.pose.position.z;
   odom_trans.transform.rotation = fetchbase_odom_msg->pose.pose.orientation;
 
   odom_trans_init = true;
 
   // Send the transform out on the UP
   odom_broadcaster->sendTransform(odom_trans);
}

// Callback for *cmd_vel "Twist" subscriptions
void cmd_vel_callback(
   const geometry_msgs::TwistConstPtr& cmd_vel_msg
) {
   // Republish raw msg
   fetchbase_cmd_vel_pub->publish(cmd_vel_msg);
}

int main (
    int argc, 
    char** argv
) {
    ros::init(argc, argv, "base_bridge");  //node_name
    ros::NodeHandle nh;
    ROS_INFO_STREAM("base_bridge started");

    // Arrange to exit cleanly 
    register_sig_handler(); 

    // Set up all publishers

    // Create tf broadcaster for odom frame
    odom_broadcaster = new tf::TransformBroadcaster();

    // Publish odom "Odometry" topic 
    odom_pub = new ros::Publisher(nh.advertise<nav_msgs::Odometry>(
        "odom", 
        100          // queue size
    ));     

    // Publish fetchbase_cmd_vel "Twist" topic 
    fetchbase_cmd_vel_pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>(
        "fetchbase_cmd_vel", 
        100          // queue size
    ));     

    // Set up all subscribers
    //  => need to do this AFTER the publishers are set up, otherwise we will
    //     be trying to use null pointers in the callbacks...

    // Subscribe to fetchbase_odom "Odometry" topic 
    fetchbase_odom_sub = new ros::Subscriber(nh.subscribe<nav_msgs::Odometry>(
        "fetchbase_odom", 
        100,         // queue size  
        &fetchbase_odom_callback
    ));

    // Subscribe to *cmd_vel "Twist" topics 
    //   => these both use the same callback.  
    //   => we do not do any prioritization.
    cmd_vel_sub = new ros::Subscriber(nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 
        100,         // queue size  
        &cmd_vel_callback
    ));
    nav_cmd_vel_sub = new ros::Subscriber(nh.subscribe<geometry_msgs::Twist>(
        "nav_cmd_vel", 
        100,         // queue size  
        &cmd_vel_callback
    ));

    // TODO: should really make sure none of the above calls return errors 
                                                        
    // Spin and wait for stuff to come in
    ros::Rate r(50.0); 
    while (ros::ok() && running) {
        ros::spinOnce();
        // keep transform alive; necessary or not?
        if (odom_trans_init) {
            odom_trans.header.stamp = ros::Time::now();
            odom_broadcaster->sendTransform(odom_trans);
        }
        r.sleep();
    }

    // Cleanup: delete subscribers first, then publishers
    delete fetchbase_odom_sub; 
    fetchbase_odom_sub = 0; 
    delete cmd_vel_sub; 
    cmd_vel_sub = 0;
    delete nav_cmd_vel_sub; 
    nav_cmd_vel_sub = 0;

    delete fetchbase_cmd_vel_pub; 
    fetchbase_cmd_vel_pub = 0; 
    delete odom_pub; 
    odom_pub = 0; 
    delete odom_broadcaster;
    odom_broadcaster = 0;

    return 0;
}

