#!/usr/bin/env python
import numpy as np
from numpy.core.numeric import Inf
import rospy

from astrobee_ros_demo.util import *

import geometry_msgs.msg
import std_srvs.srv
import ff_msgs.msg
import ff_msgs.srv


class SimpleControlExample(object):
    """
    Class implementing a simple controller example that
    does absolutely nothing on the NASA Astrobees.
    """

    def __init__(self):
        """
        Initialize controller class
        """

        # Initialize basic parameters
        self.dt = 1
        self.rate = rospy.Rate(5)
        self.start = False
        self.state = np.zeros((13, 1))
        self.state[9] = 1
        self.t0 = 0.0
        self.twist = None
        self.pose = None

        # Data timestamps and validity threshold
        self.ts_threshold = 1.0
        self.pose_ts = 0.0
        self.twist_ts = 0.0

        # Set publishers and subscribers
        self.set_services()
        self.set_subscribers_publishers()

        # Change onboard timeout
        new_timeout = ff_msgs.srv.SetFloatRequest()
        new_timeout.data = 1.5
        ans = self.pmc_timeout(new_timeout)
        if not ans.success:
            rospy.logerr("Couldn't change PMC timeout.")
            exit()
        else:
            rospy.loginfo("Timeout updated.")

        self.run()

        pass

    # ---------------------------------
    # BEGIN: Callbacks Section
    # ---------------------------------
    def pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        """
        Pose callback to update the agent's position and attitude.

        :param msg: estimated pose, defaults to geometry_msgs.msg.PoseStamped()
        :type msg: geometry_msgs.msg.PoseStamped
        """

        self.pose_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.pose = np.array([[msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]]).T
        # Update state variable
        self.state[0:3] = self.pose[0:3]
        self.state[6:10] = self.pose[3:7]
        return

    def twist_sub_cb(self, msg=geometry_msgs.msg.TwistStamped()):
        """
        Twist callback to update the agent's lineear and angular velocities.

        :param msg: estimated velocities
        :type msg: geometry_msgs.msg.TwistStamped
        """

        self.twist_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.twist = np.array([[msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z,
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z]]).T
        # Update state variable
        self.state[3:6] = self.twist[0:3]
        self.state[10:13] = self.twist[3:6]
        return

    def start_srv_callback(self, req=std_srvs.srv.SetBoolRequest()):
        """
        Service to start the operation of the autonomous control.

        :param req: request state
        :type req: std_srvs.srv.SetBoolRequest
        :return: success at starting
        :rtype: std_srvs.srv.SetBoolResponse
        """
        state = req.data
        ans = std_srvs.srv.SetBoolResponse()
        if state:
            ans.success = True
            ans.message = "Node started!"

            self.t0 = rospy.get_time()
            # Disable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = False
            self.onboard_ctl(obc)
            self.start = True
        else:
            ans.success = True
            ans.message = "Node stopped!"
            # Enable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = True
            self.onboard_ctl(obc)
            self.start = False

        return ans

    # ---------------------------------
    # END: Callbacks Section
    # ---------------------------------

    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """

        # Subscribers
        self.pose_sub = rospy.Subscriber("~pose_topic",
                                         geometry_msgs.msg.PoseStamped,
                                         self.pose_sub_cb)
        self.twist_sub = rospy.Subscriber("~twist_topic",
                                          geometry_msgs.msg.TwistStamped,
                                          self.twist_sub_cb)

        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           ff_msgs.msg.FamCommand,
                                           queue_size=1, latch=True)

        self.flight_mode_pub = rospy.Publisher("~flight_mode",
                                               ff_msgs.msg.FlightMode,
                                               queue_size=1)

        pass

    def set_services(self):
        """
        Helper function to create all services.
        """

        # Astrobee control disable and timeout change service
        self.onboard_ctl = rospy.ServiceProxy("~onboard_ctl_enable_srv",
                                              std_srvs.srv.SetBool)
        self.pmc_timeout = rospy.ServiceProxy("~pmc_timeout_srv",
                                              ff_msgs.srv.SetFloat)

        # Start service
        self.start_service = rospy.Service("~start_srv", std_srvs.srv.SetBool,
                                           self.start_srv_callback)

        # Wait for services
        self.onboard_ctl.wait_for_service()
        self.pmc_timeout.wait_for_service()
        pass

    def check_data_validity(self):
        """
        Helper function to check the data validity.

        :return: True if data is valid, False otherwise
        :rtype: boolean
        """

        pos_val = False
        vel_val = False

        # Check state validity
        if rospy.get_time() - self.pose_ts < self.ts_threshold:
            pos_val = True
        if rospy.get_time() - self.twist_ts < self.ts_threshold:
            vel_val = True
        # TODO(@User): Add further conditions that are needed for control update

        if pos_val is False or vel_val is False:
            rospy.logwarn("Skipping control. Validity flags:\nPos: "
                          + str(pos_val) + "; Vel: " + str(vel_val))

        return pos_val and vel_val

    def create_control_message(self):
        """
        Helper function to create the control message to be published

        :return: control input to vehicle
        :rtype: ff_msgs.msg.FamCommand()
        """

        # Create message
        u = ff_msgs.msg.FamCommand()

        # Fill header
        u.header.frame_id = 'body'
        u.header.stamp = rospy.Time.now()

        # Fill force / torque messages
        u.wrench.force.x = self.u_traj[0]
        u.wrench.force.y = self.u_traj[1]
        u.wrench.force.z = self.u_traj[2]
        u.wrench.torque.x = self.u_traj[3]
        u.wrench.torque.y = self.u_traj[4]
        u.wrench.torque.z = self.u_traj[5]

        # Set control mode and status
        u.status = 3
        u.control_mode = 2

        return u

    def create_flight_mode_message(self):
        """
        Helper function to create the flight mode message.

        Here we use the flight-mode difficult to have full access to the
        actuation capabilities of the Astrobee.
        """

        fm = ff_msgs.msg.FlightMode()

        fm.name = "difficult"

        fm.collision_radius = 0.25
        fm.control_enabled = False

        fm.att_ki = Vec(0.002, 0.002, 0.002)
        fm.att_kp = Vec(4.0, 4.0, 4.0)
        fm.omega_kd = Vec(3.2, 3.2, 3.2)

        fm.pos_kp = Vec(.6, .6, .6)
        fm.pos_ki = Vec(0.0001, 0.0001, 0.0001)
        fm.vel_kd = Vec(1.2, 1.2, 1.2)

        fm.speed = 3

        fm.tolerance_pos = 0.2
        fm.tolerance_vel = 0
        fm.tolerance_att = 0.3490
        fm.tolerance_omega = 0
        fm.tolerance_time = 1.0

        fm.hard_limit_accel = 0.0200
        fm.hard_limit_omega = 0.5236
        fm.hard_limit_alpha = 0.2500
        fm.hard_limit_vel = 0.4000

        return fm

    def run(self):
        """
        Main operation loop.
        """

        while not rospy.is_shutdown():

            # Only do something when started
            if self.start is False:
                self.rate.sleep()
                rospy.loginfo("Sleeping...")
                continue

            # Use self.pose, self.twist to generate a control input
            t = rospy.get_time() - self.t0
            val = self.check_data_validity()
            if val is False:
                self.rate.sleep()
                continue

            # - Start of the control section
            #
            # Here the user should modify the variable u_traj that is posteriorly sent
            # to the robot. The variable u_traj is an array containing a 3D force
            # (on u_traj[0:3]) and 3D torque (on u_traj[3:]).
            tin = rospy.get_time()
            self.u_traj = np.zeros((6, ))  # TODO(@User): use your controller here
            tout = rospy.get_time() - tin
            rospy.loginfo("Time for control: " + str(tout))

            # Create control input message
            u = self.create_control_message()
            fm = self.create_flight_mode_message()

            # Publish control
            self.control_pub.publish(u)
            self.flight_mode_pub.publish(fm)
            self.rate.sleep()
    pass


if __name__ == "__main__":
    rospy.init_node("node_template")
    dmpc = SimpleControlExample()
    rospy.spin()
    pass
