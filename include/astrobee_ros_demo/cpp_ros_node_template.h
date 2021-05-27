#ifndef CPP_ROS_NODE_TEMPLATE_H
#define CPP_ROS_NODE_TEMPLATE_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Wrench.h"
#include "std_srvs/SetBool.h"
#include "ff_msgs/FamCommand.h"
#include "ff_msgs/FlightMode.h"
#include "ff_msgs/SetFloat.h"
#include <ff_util/ff_flight.h>

#define TS_THRESHOLD 1.0

class SimpleControlExample
{
private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher control_pub_;
    ros::Publisher flight_mode_pub_;
    
    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    // Service Clients
    ros::ServiceClient onboard_ctl_;
    ros::ServiceClient pmc_timeout_;

    // Service Providers
    ros::ServiceServer start_service_;

    // Internal variables
    double starting_time_;
    double t_;
    double pose_time_;
    double twist_time_;
    bool start_node_;
    bool data_validity_;

    // Internal ROS variables
    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped twist_;
    geometry_msgs::Wrench control_input_;
    ros::Rate rate_;
    
    // Helpers for Services and Publishers/Subscribers inits
    void SetServices();
    void SetPublishersSubscribers();

    // Service callback
    bool StartServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    // Subscribers callbacks
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Extra function declarations
    void PublishControl();
    void PublishFlightMode();
    bool CheckDataValidity(double t);
    int Run();

public:
    SimpleControlExample(const ros::NodeHandle &nh);
    ~SimpleControlExample();

};

#endif