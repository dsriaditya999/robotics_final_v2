'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should re-use the trajectory construction from HW4 P4.  It
   also has proper handling to shut down when the trajectory runs out
   of segments.  And provides the time step (dt) to the evaluation, in
   preparation for the coming inverse kinematics.

   Node: /generator Publish: /joint_states sensor_msgs/JointState

'''

import rclpy
import numpy as np

from asyncio            import Future
from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from std_msgs.msg import Int32MultiArray

from hw3code.Segments   import Hold, Stay, GotoCubic, SplineCubic
# from hw4code.hw4p3      import fkin, Jac

from final_v2.GeneratorNode     import GeneratorNode
from final_v2.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *
from std_msgs.msg       import Bool
from geometry_msgs.msg       import Pose

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self,node):
        # Pick the target.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        self.q = np.radians(np.array([0]*41).reshape((-1,1)))
        self.q[0,0] = np.pi/2
        self.q_nom = np.radians(np.array([0]*41).reshape((-1,1)))
        self.q_nom[0,0] = np.pi/2

        self.publisher_ = node.create_publisher(Int32MultiArray, 'collision', 10)
        # [# of success, # of total]
        self.publisher_2 = node.create_publisher(Int32MultiArray, 'touch_target', 10)
        self.target_touched = 0
        self.target_total = 0

        self.publisher_3 = node.create_publisher(Pose, 'set_random_target', 10)

        self.chain.setjoints(self.q)
        #self.segments = [Hold(self.q, 1.0),GotoCubic(q1, q2, 1.0),GotoCubic(q2, q1, 1.0)]
        self.t0 = 0
        self.cyclic = True
        self.goal_list = [[-1.35,1,0],[-1.35,-1,0]]

        self.lam = 10
        self.collision_count = 0
        self.colliding = False

        self.segments = []
        self.t0 = 0
        self.ta = 0
        self.err = np.zeros((6,1))

        self.subscription_3 = node.create_subscription(
            Bool,
            'reach',
            self.reach_control_rcvd,
            10
        )

        self.pos = self.chain.ptip()
        self.quat = quat_from_R(self.chain.Rtip())
        self.node = node
        self.reach_status = True


    def reach_control_rcvd(self,msg):

        self.reach_status = msg.data

        if msg.data and len(self.segments)==0:
            #self.node.get_logger().info('getting target?')
            
            #self.node.get_logger().info(str(msg.data))
            self._set_target(self.pos, self.quat)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []
        for ind in range(1,42):
            j.append('joint%i'%ind)
        return j

    def set_target(self,pos,quat):
        #self.node.get_logger().info(str(pos.x))
        # self.pos = [pos.x,pos.y,pos.z]
        # self.quat = [quat.w,quat.x,quat.y,quat.z]
        pass

    def _set_target(self, pos, quat):
        # print('target (%f,%f,%f)'%(x,y,z))
        self.t0 = self.ta
        xspline = GotoCubic( self.chain.ptip(), np.array([[pos[0]-0.02],[pos[1]+0.02],[pos[2]-0.05]]),6,space='Tip')
        self.target_total += 1

        R0 = self.chain.Rtip()
        Rf = R_from_quat(np.array(quat)-np.array([  0.0,0.0,-0.216,  0.976]))
        self.Rf = Rf
        self.xf = np.array([[pos[0]],[pos[1]],[pos[2]]])
        R0f = Rf.T @ R0
        u = np.array([
            [R0f[2,1]-R0f[1,2]],
            [R0f[0,2]-R0f[2,0]],
            [R0f[1,0]-R0f[0,1]]
        ])
        un = np.linalg.norm(u)
        if un<1e-5:
            w,v = np.linalg.eig( R0f )
            u = v[:,2].reshape((3,1))
            u = np.real(u)
        else:
            u/=un
        alpha = np.arccos((np.trace(R0f)-1)/2)
        # R0 @ R(e,alpha) = Rf -> RfT @ R0 = R(e,-alpha)
        if not np.allclose(Rote(u,-alpha),R0f):
            alpha = -alpha

        rspline = GotoCubic(0,alpha,6)
        self.segments.append([xspline,rspline,u,R0])

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        self.publisher_3.publish(pose)

    def step(self, dt):
        x0 = self.chain.ptip()
        R0 = self.chain.Rtip()
        xf = np.array(self.pos).reshape((3,1))
        Rf = R_from_quat(np.array(self.quat))

        xdot = ep(xf, x0)
        # xdn = np.linalg.norm(xdot)
        # # if 1 * dt< xdn:
        # #     xdot/=xdn
        # # else:
        # #     xdot /= dt*100
        
        # R0f = Rf.T @ R0
        # u = np.array([
        #     [R0f[2,1]-R0f[1,2]],
        #     [R0f[0,2]-R0f[2,0]],
        #     [R0f[1,0]-R0f[0,1]]
        # ])
        # un = np.linalg.norm(u)
        # if un<1e-5:
        #     w,v = np.linalg.eig( R0f )
        #     u = v[:,2].reshape((3,1))
        #     u = np.real(u)
        # else:
        #     u/=un
        # alpha = np.arccos((np.trace(R0f)-1)/2)
        # # R0 @ R(e,alpha) = Rf -> RfT @ R0 = R(e,-alpha)
        # if not np.allclose(Rote(u,-alpha),R0f):
        #     alpha = -alpha
        
        eR_R = eR(Rf,R0)
        # if 1*dt < np.abs(alpha):
        #     alpha /= alpha
        #     eR_R /= np.linalg.norm(eR_R)
        # else:
        #     alpha /= dt*100
        # wd = R0 @ u * alpha

        return xdot, eR_R

    def set_goal(self, pos):
        self.goal_list = []
        for p in pos:
            self.goal_list.append([p.x,p.y,p.z])

        #print(self.goal_list)
    def check_touch(self):
        x0 = self.chain.ptip()
        R0 = self.chain.Rtip()
        xf, Rf = self.xf, self.Rf
        condition = [np.allclose(R0, Rf, atol=.1), np.allclose(x0, xf, atol=.1), self.ta>self.t0+6]
        if (condition[0] and condition[1]) or condition[2]:
            if condition[0] and condition[1]:
                self.target_touched += 1
                self.node.get_logger().info('touch target')
            else:
                self.node.get_logger().info('times up')
            msg = Int32MultiArray()
            msg.data = [self.target_touched, self.target_total]
            self.publisher_2.publish(msg)
            seg = self.segments.pop(0)
            u,v,w = np.random.uniform(0,1,3)
            quat = [
                np.sqrt(1-u)*np.sin(2*np.pi*v),
                np.sqrt(1-u)*np.cos(2*np.pi*v),
                np.sqrt(u)*np.sin(2*np.pi*w),
                np.sqrt(u)*np.cos(2*np.pi*w),
            ]
            self._set_target(
                [np.random.uniform(-1,1),
                np.random.uniform(-1,1),
                np.random.uniform(-1,1)],
                quat
            )
    
    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):
        self.ta = tabsolute

        qdot = np.zeros((41,1))
        J_all = np.empty((0,41))
        eRR = np.empty((0,1))

        collision_count = 0
        collision_threshold = 0.072
        collision_arr = Int32MultiArray()

        for goal in self.goal_list:
            pd = np.array(goal).reshape((3,1))

            p_links = []
            for dof in range(self.chain.dofs):
                p_links.append(p_from_T(self.chain.data.T[dof]).reshape((3,1)))

            # Calulating the #dof of tip
            p_links = np.array(p_links).squeeze()
            p_links = p_links.T
            tip_dof = np.argmin(np.linalg.norm(pd-p_links,axis=0))

            tip_dis = np.min(np.linalg.norm(pd-p_links,axis=0))
            if tip_dis<collision_threshold:# and tip_dof>=10:
                collision_count +=1
                collision_arr.data.append(1)
            else:
                collision_arr.data.append(0)

            pd_tip = p_from_T(self.chain.data.T[tip_dof]).reshape((3,1))

            if pd[2][0]<pd_tip[2][0]:
                continue

            eR1 = pd[:2,:]
            eR2 = pd_tip[:2,:]
            eR12 = ep(eR2,eR1)
            eRn = np.linalg.norm(eR12)
            eR12 /= eRn**2
            eRR = np.append(eRR, 0.5*eR12, axis=0)
            J_all = np.append(J_all,self.chain.Jv_tip(tip_dof)[:2,:],axis=0)

        if collision_count>0 and not self.colliding: 
            print('collision detected: #',collision_count)
            self.collision_count += collision_count
            self.colliding = True
        if collision_count>0 and self.colliding:
            self.colliding = False
        collision_arr.data.insert(0,self.collision_count)
        self.publisher_.publish(collision_arr)
            
        Jinv = np.linalg.pinv(J_all,rcond=0.01)

        # if len(self.segments)>0 and self.segments[0][0].completed(tabsolute - self.t0):
        #     self.check_touch()
        #     seg = self.segments.pop(0)
        #     self.t0 += seg[0].duration()
        #     print('target hit')

        if self.reach_status:

            if len(self.segments)==0:
                qdot3 = (self.q_nom - self.q)
                
                J2 = np.vstack((self.chain.Jv(),self.chain.Jw()))
                Jinv2 = np.linalg.pinv(J2,0.01)
                (xdot, wd) = self.step(dt)
                xd2 = np.vstack((xdot,wd))
                qdot2 = Jinv2 @ xd2 + (np.eye(J2.shape[1])- Jinv2 @ J2) @ qdot3

                Jinv = np.linalg.pinv(J_all,0.01)
                qdot = Jinv @ eRR + (np.eye(J_all.shape[1])- Jinv @ J_all)@ (qdot2)
                
                self._set_target(
                    [np.random.uniform(-1,1),
                    np.random.uniform(-1,1),
                    np.random.uniform(-1,1)],
                    [1.,0.,0.,0.]
                )
                q = self.q + qdot*dt
                self.q = q
                self.chain.setjoints(self.q)
            else:
                qdot3 = (self.q_nom - self.q)
                
                J2 = np.vstack((self.chain.Jv(),self.chain.Jw()))
                Jinv2 = np.linalg.pinv(J2,0.1)
                (xd, xdot) = self.segments[0][0].evaluate(tabsolute - self.t0)
                (theta_d, wd) = self.segments[0][1].evaluate(tabsolute - self.t0)
                eh = self.segments[0][2]
                R0 = self.segments[0][3]
                Rd = R0 @ Rote(eh,theta_d)
                wd = R0 @ eh * wd
                xd2 = np.vstack((xdot,wd))

                # two primary task
                # eRR = np.append(eRR, (xd2 + 10*self.err), axis=0)
                # J_all = np.append(J_all,J2,axis=0)
                # Jinv = np.linalg.pinv(J_all,0.01)
                # qdot = Jinv @ (eRR) + (np.eye(J_all.shape[1])- Jinv @ J_all)@ (qdot3)

                # avoid -> traj
                qdot2 = Jinv2 @ (xd2 + 10*self.err) + (np.eye(J2.shape[1])- Jinv2 @ J2) @ qdot3
                Jinv = np.linalg.pinv(J_all,0.01)
                qdot = Jinv @ eRR + (np.eye(J_all.shape[1])- Jinv @ J_all)@ (qdot2)

                q = self.q + qdot*dt
                self.q = q
                self.chain.setjoints(self.q)
            
                self.err = np.vstack((ep(xd,self.chain.ptip()), eR(Rd,self.chain.Rtip())))
                self.check_touch()

        else:

            qdot = Jinv @ eRR + (np.eye(J_all.shape[1])- Jinv @ J_all) @ (self.lam*(self.q_nom - self.q))

            q = self.q + qdot*dt
            self.q = q
            self.chain.setjoints(self.q)

            

        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
