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
import math

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
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self,node):
        # Pick the target.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        self.q = np.radians(np.array([0]*41).reshape((-1,1)))
        self.qdot = np.radians(np.array([0]*41).reshape((-1,1)))
        
        self.q_nom = np.radians(np.array([0]*41).reshape((-1,1)))
        # self.q_nom[0,0] = np.pi/2

        self.setup()


    def setup(self):
        self.q = np.radians(np.array([0]*41).reshape((-1,1)))
        self.Node = np.random.randint(20,40)
        self.startpoint = np.random.randint(1,41-self.Node)
        xyzgoal = self.findgoal(self.Node, self.startpoint)
        self.pgoal = xyzgoal.flatten('F').reshape((self.Node *3,1))
        self.setting = False

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


    def tie_knot(self, step=1.):
        x_tips = np.empty((0,1))
        J_tips = np.empty((0,41))
        g_tips = np.empty((0,1))
        base = self.pgoal[:3]*1

        step = step*1.5
        if step>1: step=1

        start_point = round( self.startpoint * step + (self.chain.dofs-1) * (1-step) )
        base *= (start_point-self.startpoint)/self.startpoint
        for i in range(start_point, min( start_point + self.Node, self.chain.dofs)):
            x_tips = np.append(x_tips, p_from_T(self.chain.data.T[i]), axis=0)
            J_tips = np.append(J_tips, self.chain.Jv_tip(i), axis=0)
            j = i-start_point
            g_tips = np.append(g_tips, (self.pgoal[3*j:3*j+3]+base).reshape((3,1)), axis=0)
        
        Jinv = np.linalg.pinv(J_tips, 0.01)
        qdot3 = self.q_nom - self.q
        qdot = Jinv @ (g_tips - x_tips) + (np.eye(J_tips.shape[1])- Jinv @ J_tips) @ qdot3
        return qdot
        x_tips = np.empty((0,1))
        J_tips = np.empty((0,41))

        start_point = self.startpoint
        for i in range(start_point, start_point + self.Node):
            x_tips = np.append(x_tips, p_from_T(self.chain.data.T[i]), axis=0)
            J_tips = np.append(J_tips, self.chain.Jv_tip(i), axis=0)

        Jinv = np.linalg.pinv(J_tips, 0.01)
        qdot = Jinv @ (self.pgoal - x_tips)
        return qdot
    
    def solve_knot(self):
        qdot3 = self.q_nom - self.q
        qdot3[qdot3>np.pi] -= 2*np.pi
        qdot3[qdot3<-np.pi] += 2*np.pi

        p_joints = np.empty((3,0))
        for dof in range(self.chain.dofs):
            p_joints = np.append(p_joints, p_from_T(self.chain.data.T[dof]), axis=1)
        
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
                    # xj_rep = 0.05* perpend/np.linalg.norm(perpend)**2
                    # if proj<dP_norm/2:
                    #     # xj_rep += -0.1*dP/dP_norm
                    # else:
                    #     xj_rep += 0.1*dP/dP_norm
                    xj_rep = p_joints[:,j]-0.5*(p_joints[:,i+1]+p_joints[:,i])
                    xj_rep /= np.linalg.norm(perpend)**2
                    xj_rep *= 0.05
                    Jj = self.chain.Jv_tip(j)
                    Jj[:,:i+1] = 0
                    x_repulsive = np.append(x_repulsive, xj_rep.reshape((3,1)), axis=0)
                    J_all = np.append(J_all, Jj, axis=0)
        
        Jinv = np.linalg.pinv(J_all, 0.01)
        qdot = Jinv @ x_repulsive + (np.eye(J_all.shape[1])- Jinv @ J_all) @ qdot3
                
        return qdot


    def evaluate(self, tabsolute, dt):
        T = 30
        duty = 20
        if tabsolute%T<duty:
            if self.setting:
                self.setup()
            qdot = self.tie_knot((tabsolute%T)/duty)
        else:
            self.setting = True
            qdot = self.solve_knot()
        
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
