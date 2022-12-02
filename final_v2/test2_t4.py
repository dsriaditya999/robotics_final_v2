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

        #q1 = np.radians(np.array([0]*39).reshape((-1,1)))
        #q2 = np.radians(np.array([10]*39).reshape((-1,1)))
        self.publisher_ = node.create_publisher(Int32MultiArray, 'collision', 10)
        # [# of success, # of total]
        self.publisher_2 = node.create_publisher(Int32MultiArray, 'touch_target', 10)
        self.target_touched = 0
        self.target_total = 0

        self.publisher_3 = node.create_publisher(Pose, 'set_random_target', 10)
        self.publisher_4 = node.create_publisher(Bool, 'phase', 10)

        self.publisher_5 = node.create_publisher(Int32MultiArray, '/self_collision', 10)
        self.self_collision = Int32MultiArray()
        self.self_collision.data = [0]

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

        self.pos = [-3.,0.,0.]
        self.quat = [1.,0.,0.,0.]
        self.node = node
        self.reach_status = True
        self.setting = True


    def setup(self):
        self.q = np.radians(np.array([0]*41).reshape((-1,1)))
        self.Node = np.random.randint(20,40)
        self.startpoint = np.random.randint(1,41-self.Node)
        xyzgoal = self.findgoal(self.Node, self.startpoint)
        self.pgoal = xyzgoal.flatten('F').reshape((self.Node *3,1))
        self.self_collision.data = [0]
        self.setting = False


    def reach_control_rcvd(self,msg):

        self.reach_status = msg.data

        if msg.data and len(self.segments)==0:
            #self.node.get_logger().info('getting target?')
            
            #self.node.get_logger().info(str(msg.data))
            # self._set_target(self.pos, self.quat)
            pass

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []
        for ind in range(1,42):
            j.append('joint%i'%ind)
        return j

    def findgoal(self, Node=30, startpoint = 5):
        a = 1/3
        b = 5/3
        N = 1000
        t = np.linspace(np.pi*a,np.pi*b,N)

        x = np.cos(t)+2*np.cos(2*t)
        y = np.sin(t)-2*np.sin(2*t)
        z =-np.sin(3*t)
        xyz = np.vstack((x,y,z))

        xyz = Rotx(np.random.uniform(0,2*np.pi))@Rotz(np.random.uniform(0,2*np.pi))@xyz

        dxyz = xyz[:,1:]-xyz[:,:N-1]
        dr = np.sqrt(np.sum(dxyz**2,axis=0))
        ds = np.cumsum(dr)

        scale = ds[N-2]/Node/0.072
        xyz/=scale

        ds_N = ds[N-2]/Node
        indexs = []
        index = 0
        for i in range(1,Node):
            for j in range(index,N-1):
                if ds[j]>=ds_N*i:
                    index = j
                    indexs.append(index)
                    break
        indexs.append(N-1)
        base = 0.072*startpoint*dxyz[:,0]/np.linalg.norm(dxyz[:,0])
        xyz_goal = xyz[:,np.array(indexs)]-np.repeat(xyz[:,0]-base,Node,axis=0).reshape((3,Node))
        return xyz_goal


    def tie_knot(self):

        q0 = self.q
        q = {}
        q[0] = q0

        for j in range(100):
            x_tips = np.empty((0,1))
            J_tips = np.empty((0,41))

            start_point = self.startpoint
            for i in range(start_point,start_point + self.Node):
                x_tips = np.append(x_tips, p_from_T(self.chain.data.T[i]), axis=0)
                J_tips = np.append(J_tips, self.chain.Jv_tip(i), axis=0)

            Jinv = np.linalg.pinv(J_tips, 0.01)
            # qdot = Jinv @ (self.pgoal - x_tips)
            q[j+1] = q[j] + 0.1*Jinv @ (self.pgoal - x_tips)

            self.chain.setjoints(q[j+1])


        return q[100]
    
    def solve_knot(self,eRR_c,J_inv_col, J_col):

        qdot3 = self.q_nom - self.q
        qdot3[qdot3>np.pi] -= 2*np.pi
        qdot3[qdot3<-np.pi] += 2*np.pi

        qdot2 = J_inv_col @ eRR_c + (np.eye(J_col.shape[1]) - J_inv_col @ J_col) @ qdot3

        p_joints = np.empty((3,0))
        for dof in range(self.chain.dofs):
            p_joints = np.append(p_joints, p_from_T(self.chain.data.T[dof]), axis=1)
        
        self_collision = 0
        J_all = np.empty((0,41))
        x_repulsive = np.empty((0,1))
        for i in range(p_joints.shape[1]-1):
            dP = p_joints[:,i+1]-p_joints[:,i]
            dP_norm = np.linalg.norm(dP)
            for j in range(i+2, p_joints.shape[1]):
                dPj = p_joints[:,j]-p_joints[:,i]
                proj = np.dot(dP,dPj)/dP_norm
                perpend = dPj-dP/dP_norm*proj
                if proj>0 and proj<dP_norm and np.linalg.norm(perpend)<0.2:
                    if np.linalg.norm(perpend)<0.072:
                        self_collision += 1
                    xj_rep = p_joints[:,j]-0.5*(p_joints[:,i+1]+p_joints[:,i])
                    xj_rep /= np.linalg.norm(perpend)**2
                    xj_rep *= 0.05
                    Jj = self.chain.Jv_tip(j)
                    Jj[:,:i+1] = 0
                    x_repulsive = np.append(x_repulsive, xj_rep.reshape((3,1)), axis=0)
                    J_all = np.append(J_all, Jj, axis=0)
        
        self.self_collision.data = [self_collision]
        Jinv = np.linalg.pinv(J_all, 0.01)
        qdot = Jinv @ x_repulsive + (np.eye(J_all.shape[1])- Jinv @ J_all) @ qdot2
                
        return qdot


    def set_target(self,pos,quat):
        #self.node.get_logger().info(str(pos.x))
        # self.pos = [pos.x,pos.y,pos.z]
        # self.quat = [quat.w,quat.x,quat.y,quat.z]
        pass

    def _set_target(self, pos=None, quat=None):
        if pos==None:
            pos = [np.random.uniform(-1,1),
                    np.random.uniform(-1,1),
                    np.random.uniform(-1,1)]
        if quat==None:
            u,v,w = np.random.uniform(0,1,3)
            quat = [
                np.sqrt(1-u)*np.sin(2*np.pi*v),
                np.sqrt(1-u)*np.cos(2*np.pi*v),
                np.sqrt(u)*np.sin(2*np.pi*w),
                np.sqrt(u)*np.cos(2*np.pi*w),
            ]
        # print('target (%f,%f,%f)'%(x,y,z))
        self.t0 = self.ta
        xspline = GotoCubic( self.chain.ptip(), np.array([[pos[0]],[pos[1]],[pos[2]]]),6,space='Tip')
        self.target_total += 1

        R0 = self.chain.Rtip()
        Rf = R_from_quat(np.array(quat))
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
        eR_R = eR(Rf,R0)
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
        condition = [np.allclose(R0, Rf, atol=.05), np.allclose(x0, xf, atol=.05), self.ta>=self.t0+11]
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
            self._set_target()
    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):
        
        self.ta = tabsolute
        phase = Bool()

        if self.ta%25<=4:
            if self.setting:
                self.setup()
                q = self.tie_knot()
            else:
                q = self.q
            qdot = np.zeros((41,1))
            self.q = q
            self.chain.setjoints(self.q)

            phase.data = False
            self.segments = []
            

        # Case-1 : You are in a knot intially and you have to come out while avoiding obstacles

        # Primary Task - Untangle, Secondary Task - Avoid Collisions, Tertiary Task - Pull to nominal

        else:
            
            phase.data = True

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



            if 4<(self.ta%25)<=14:

                self.setting = True
                qdot = self.solve_knot(eRR,Jinv,J_all)
                q = self.q + qdot*dt
                self.q = q
                self.chain.setjoints(self.q)

        # Case-1 : You are hopefully out the knot, you have to now touch the target while avoiding obstacles

        # Primary Task - Untangle, Secondary Task - Avoid Collisions, Tertiary Task - Pull to nominal

            else:
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
                        
                        self._set_target(quat=[1.,0.,0.,0.])
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

        self.publisher_4.publish(phase)                
        self.publisher_5.publish(self.self_collision)
        
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
