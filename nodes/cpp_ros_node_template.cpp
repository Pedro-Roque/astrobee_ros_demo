#include "astrobee_ros_demo/cpp_ros_node_template.h"
#include "geometry_msgs/Vector3.h"

SimpleControlExample::SimpleControlExample(const ros::NodeHandle &nh) : nh_(nh), start_node_(false), rate_(5)
{
    /**
     * @brief Simple example class initializer.
     *
     * @param nh ROS nodehandler for this node.
     */

    // Initialize class variables
    rate_ = ros::Rate(5);
    starting_time_ = 0.0;
    pose_time_ = 0.0;
    twist_time_ = 0.0;

    // Call helper functions for services and topics
    SetServices();
    SetPublishersSubscribers();

    // Change PMC timeout
    ff_msgs::SetFloat new_timeout;

    new_timeout.request.data = 1.5;
    pmc_timeout_.call(new_timeout);
    if (!new_timeout.response.success)
    {
        ROS_ERROR("Couldn't change PMC timeout.");
    }
    else
    {
        ROS_INFO("PMC Timeout Updated.");
    }

    Run();
}

SimpleControlExample::~SimpleControlExample()
{
}

/*---------------------------------------
        START of Callbacks Section
---------------------------------------*/

bool SimpleControlExample::StartServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    /**
     * @brief Callback for the node start service
     *
     * @param req service request
     *
     * @param res service response message
     *
     */

    bool state = req.data;
    ROS_INFO("Start Callback received request");

    if (state)
    {
        res.success = true;
        res.message = "Node started!";

        starting_time_ = ros::Time::now().toSec();

        // Disable onboard controller
        std_srvs::SetBool obc;
        obc.request.data = false;
        onboard_ctl_.call(obc);
        ROS_INFO_STREAM("OBC Response on Disabling: " << obc.response.success << " - " << obc.response.message);
        start_node_ = true;
    }
    else
    {
        res.success = true;
        res.message = "Node stopped!";

        // Enable onboard controller
        std_srvs::SetBool obc;
        obc.request.data = true;
        onboard_ctl_.call(obc);
        ROS_INFO_STREAM("OBC Response on Enabling: " << obc.response.success << " - " << obc.response.message);
        start_node_ = false;
    }

    return true;
}

void SimpleControlExample::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /**
     * @brief Callback for the pose subscriber.
     *
     * @param msg pose message
     *
     */
    pose_time_ = msg->header.stamp.toSec();
    pose_ = *msg;
}

void SimpleControlExample::TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    /**
     * @brief Callback for the twist subsriber, containing the estimated velocity.
     *
     * @param msg twist message
     *
     */
    twist_time_ = msg->header.stamp.toSec();
    twist_ = *msg;
}

/*---------------------------------------
        END of Callbacks Section
---------------------------------------*/

void SimpleControlExample::SetServices()
{
    /**
     * @brief Helper function to initialized required services.
     *
     */
    // Astrobee control disable and timeout change service
    onboard_ctl_ = nh_.serviceClient<std_srvs::SetBool>("onboard_ctl_enable_srv");
    pmc_timeout_ = nh_.serviceClient<ff_msgs::SetFloat>("pmc_timeout_srv");

    // Start Service
    start_service_ = nh_.advertiseService("start_srv", &SimpleControlExample::StartServiceCallback, this);

    // Wait for services to be advertised
    onboard_ctl_.waitForExistence();
    pmc_timeout_.waitForExistence();

    return;
}

void SimpleControlExample::SetPublishersSubscribers()
{
    /**
     * @brief Helper function to setup publishers and subscribers
     *
     */
    // Set Subscribers
    pose_sub_ = nh_.subscribe("pose_topic", 1, &SimpleControlExample::PoseCallback, this);
    twist_sub_ = nh_.subscribe("twist_topic", 1, &SimpleControlExample::TwistCallback, this);

    // Set Publishers
    control_pub_ = nh_.advertise<ff_msgs::FamCommand>("control_topic", 1);
    flight_mode_pub_ = nh_.advertise<ff_msgs::FlightMode>("flight_mode", 1);
}

bool SimpleControlExample::CheckDataValidity(double t)
{
    /**
     * @brief Function that checks the expiration of the last pose and velocity received
     *
     * @param t time since node was activated - can be used for further validitions by the user
     *
     */

    bool pos_val = false;
    bool vel_val = false;

    if (ros::Time::now().toSec() - pose_time_ < TS_THRESHOLD)
        pos_val = true;
    if (ros::Time::now().toSec() - twist_time_ < TS_THRESHOLD)
        vel_val = true;
    // TODO(@User): Add further conditions that are needed for control update

    if (!pos_val || !vel_val)
        ROS_WARN_STREAM("Skipping control. Validity flags:\n Pos:" << pos_val << " - Vel: " << vel_val);

    return pos_val && vel_val;
}

void SimpleControlExample::PublishControl()
{
    /**
     * @brief Helper function to publish the control setpoint to the Fam module.
     *
     */
    ff_msgs::FamCommand gnc_setpoint_;

    gnc_setpoint_.header.frame_id = "body";
    gnc_setpoint_.header.stamp = ros::Time::now();

    gnc_setpoint_.wrench = control_input_;

    gnc_setpoint_.status = 3;
    gnc_setpoint_.control_mode = 2;

    control_pub_.publish(gnc_setpoint_);

    return;
}

void SimpleControlExample::PublishFlightMode()
{
    /**
     * @brief Helper function to publish the flight mode message
     *
     */
    std::string flight_mode_name = "difficult"; // To have full access to Astrobee actuators range
    ff_msgs::FlightMode flight_mode_;
    ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name);
    flight_mode_.control_enabled = false;
    flight_mode_.control_enabled = false;

    flight_mode_.att_ki.x = 0.002;
    flight_mode_.att_ki.y = 0.002;
    flight_mode_.att_ki.z = 0.002;

    flight_mode_.att_kp.x = 4.0;
    flight_mode_.att_kp.y = 4.0;
    flight_mode_.att_kp.z = 4.0;

    flight_mode_.omega_kd.x = 3.2;
    flight_mode_.omega_kd.y = 3.2;
    flight_mode_.omega_kd.z = 3.2;

    flight_mode_.pos_kp.x = 0.6;
    flight_mode_.pos_kp.y = 0.6;
    flight_mode_.pos_kp.z = 0.6;

    flight_mode_.pos_ki.x = 0.0001;
    flight_mode_.pos_ki.y = 0.0001;
    flight_mode_.pos_ki.z = 0.0001;

    flight_mode_.vel_kd.x = 1.2;
    flight_mode_.vel_kd.y = 1.2;
    flight_mode_.vel_kd.z = 1.2;

    flight_mode_.speed = 3;

    flight_mode_.tolerance_pos = 0.2;
    flight_mode_.tolerance_vel = 0;
    flight_mode_.tolerance_att = 0.3490;
    flight_mode_.tolerance_omega = 0;
    flight_mode_.tolerance_time = 1.0;

    flight_mode_.hard_limit_accel = 0.0200;
    flight_mode_.hard_limit_omega = 0.5236;
    flight_mode_.hard_limit_alpha = 0.2500;
    flight_mode_.hard_limit_vel = 0.4000;
    flight_mode_pub_.publish(flight_mode_);
    return;
}

int SimpleControlExample::Run()
{
    /**
     * @brief Main loop for the simple control example class.
     *
     */

    double ctl_elapsed_;

    while (ros::ok())
    {
        // Check if node is active
        if (!start_node_)
        {
            ROS_INFO("Sleeping...");
            rate_.sleep();
            ros::spinOnce();
            continue;
        }

        // Check if data is valid
        t_ = ros::Time::now().toSec() - starting_time_;
        data_validity_ = CheckDataValidity(t_);
        if (!data_validity_)
        {
            rate_.sleep();
            ros::spinOnce();
            continue;
        }

        // Control Astrobee!
        ctl_elapsed_ = ros::Time::now().toSec();

        // Set zeroed control input - TODO(@User): replace with whatever controller you like!
        control_input_.force.x = 0.0;
        control_input_.force.y = 0.0;
        control_input_.force.z = 0.0;
        control_input_.torque.x = 0.0;
        control_input_.torque.y = 0.0;
        control_input_.torque.z = 0.0;

        // Get elapsed time
        ctl_elapsed_ = ros::Time::now().toSec() - ctl_elapsed_;
        ROS_INFO_STREAM("Elapsed time: " << ctl_elapsed_);

        // Create control and flight mode messages
        PublishControl();
        PublishFlightMode();

        // Sleep for remaining time...
        rate_.sleep();
        ros::spinOnce();
    }

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n("~");

    SimpleControlExample ctl(n);

    return 0;
}
