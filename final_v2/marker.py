#! /usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg    import JointState
import rclpy
from rclpy.node         import Node
from numpy import random
from final_v2.KinematicChain    import KinematicChain
import numpy as np
from hw5code.TransformHelpers  import *
from std_msgs.msg       import Bool

class MinimalPublisher(Node):
    v = [0,0,0,0]
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'markers_array', 10)
        self.once_not_done = False
        self.once_not_done_2 = True

        self.starttime = self.get_clock().now()
        self.servotime = self.starttime
        timer_period = 0.001  # seconds
        self.timer     = self.create_timer(timer_period, self.timer_callback)

        self.chain = KinematicChain(Node('marker_chain'), 'world', 'tip', self.jointnames())
        self.q = np.radians(np.array([0]*self.chain.dofs).reshape((-1,1)))
        self.q[0,0] = np.pi/2
        # yz case
        # self.q[1,0] = np.pi/2

        self.subscription_1 = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_rcvd,
            10)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Bool, 
            'phase',
            self.set_phase, 
            10)
        self.phase = False

        self.marker_arr = MarkerArray()
        self.marker_len = 2
        self.v = [0]*self.marker_len
        for i in range(self.marker_len):
            mark = Marker()
            mark.id = i
            mark.ns = 'name%i'%i
            mark.action = Marker.ADD
            mark.type = Marker.MESH_RESOURCE
            mark.mesh_resource = "package://final_v1/meshes/Rock.stl"
            # mark.type = 2
            mark.color.a = 1.
            mark.color.r = .7
            mark.color.g = .5
            mark.color.b = .1

            mark.header.frame_id = "/world"
            mark.scale.x = 0.1
            mark.scale.y = 0.1
            mark.scale.z = 0.1
            
            x, y = self.random_startpt()
            mark.pose.position.x = x
            mark.pose.position.y = y
            mark.pose.position.z = 4.+random.normal(0,1)
            mark.pose.orientation.x = 0.
            mark.pose.orientation.y = 0.
            mark.pose.orientation.z = 0.
            mark.pose.orientation.w = 1.
            self.marker_arr.markers.append(mark)
            
    
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []
        
        # xy case
        for ind in range(1,42):
            j.append('joint%i'%ind)
        
        # yz case
        # for ind in range(1,21):
        #     j.append('joint%ia'%ind)
        #     j.append('joint%ib'%ind)
        # j.append('finalspin')

        return j

    def joint_rcvd(self,msg):
        self.q = msg.position

    def set_phase(self, msg):
        self.phase = msg.data

    def timer_callback(self):
        dt = 0.001

        if self.phase:
            for index in range(self.marker_len):
                self.marker_arr.markers[index].color.a = 1.

                self.v[index]+=9.8*dt
                self.marker_arr.markers[index].pose.position.z -= self.v[index]*dt
                if self.marker_arr.markers[index].pose.position.z < -3.:
                    self.marker_arr.markers[index].pose.position.z = 4.+random.normal(0,1)
                    x, y = self.random_startpt()
                    self.marker_arr.markers[index].pose.position.x = x
                    self.marker_arr.markers[index].pose.position.y = y
                    self.v[index] = 0

            self.publisher_.publish(self.marker_arr)
            self.once_not_done = True

        elif self.once_not_done:
            self.once_not_done = False
            for index in range(self.marker_len):
                self.marker_arr.markers[index].color.a = 0.
                self.marker_arr.markers[index].pose.position.z = -4.
            self.publisher_.publish(self.marker_arr)



        # # Grab the current time.
        # now = self.get_clock().now()
        # t   = (now - self.starttime).nanoseconds * 1e-9
        # dt  = (now - self.servotime).nanoseconds * 1e-9
        # self.servotime = now

        # # To avoid the time jitter introduced by an inconsistent timer,
        # # just enforce a constant time step and ignore the above.
        # self.t += self.dt
        # t  = self.t
        # dt = self.dt

        # if t%25>4:


        #     if self.once_not_done_2:
        #         for index in range(self.marker_len):
        #             self.marker_arr.markers[index].action = Marker.ADD
        #         self.once_not_done_2 = False
        #     else:
        #         pass
            
        #     for index in range(self.marker_len):

        #         self.v[index]+=9.8*dt
        #         self.marker_arr.markers[index].pose.position.z -= self.v[index]*dt
        #         if self.marker_arr.markers[index].pose.position.z < -3.:
        #             self.marker_arr.markers[index].pose.position.z = 4.+random.normal(0,1)
        #             x, y = self.random_startpt()
        #             self.marker_arr.markers[index].pose.position.x = x
        #             self.marker_arr.markers[index].pose.position.y = y
        #             self.v[index] = 0

        #     self.publisher_.publish(self.marker_arr)
        #     self.once_not_done =True

        # else:
        #     if self.once_not_done:
        #         for index in range(self.marker_len):
        #             self.marker_arr.markers[index].action = Marker.DELETE
        #         self.once_not_done = False
        #         self.once_not_done_2 = True
        #     else:
        #         pass



    
    def random_startpt(self):
        self.chain.setjoints(self.q)
        T = self.chain.data.T
        i = random.randint(10,len(T)-1)
        Pi = p_from_T(T[i])
        Pf = p_from_T(T[i+1])
        ratio = random.rand()
        P = Pi*ratio+Pf*(1-ratio)
        # print('random link', i)
        # return P[0][0],P[1][0]#+random.randint(-2,2)
        p = P[:2]
        p[0][0] += np.random.normal(0,0.2)
        p[1][0] += np.random.normal(0,0.2)
        if np.linalg.norm(p)<0.3:
            p *= 0.3/np.linalg.norm(p)
        return p[0][0],p[1][0]




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

