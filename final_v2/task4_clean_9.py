
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
from hw5code.TransformHelpers   import *
from std_msgs.msg               import Bool
from geometry_msgs.msg          import Pose
from final_v2.tasks_tool        import *
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self,node):
        # Pick the target.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        dofs = self.chain.dofs
        self.q = np.radians(np.array([0]*dofs).reshape((-1,1)))
        self.q[0,0] = np.pi/2
        self.q_nom = np.radians(np.array([0]*dofs).reshape((-1,1)))
        self.q_nom[0,0] = np.pi/2

        # yz case
        # self.q[1,0] = np.pi/2
        # self.q_nom[1,0] = np.pi/2

        self.chain.setjoints(self.q)
        
        self.setup_pub_and_sub(node)
        
        self.goal_list = [[-1.35,1,0],[-1.35,-1,0]]

        self.lam = 10
        

        self.segments = []
        self.t0 = 0
        self.ta = 0
        self.err = np.zeros((6,1))

        self.pos = [-3.,0.,0.]
        self.quat = [1.,0.,0.,0.]
        self.node = node
        self.reach_status = True
        self.setting = True

        self.self_colli = 0
        self.end = False

    def setup_pub_and_sub(self,node):
        # [# of total collision, [collision for each marker]]
        self.publisher_ = node.create_publisher(Int32MultiArray, 'collision', 10)
        self.collision_count = 0
        self.colliding = False

        # [# of success, # of total]
        self.publisher_2 = node.create_publisher(Int32MultiArray, 'touch_target', 10)
        self.target_touched = 0
        self.target_total = 0

        self.publisher_3 = node.create_publisher(Pose, 'set_random_target', 10)

        self.publisher_4 = node.create_publisher(Bool, 'phase', 10)

        # [# of current collision]
        self.publisher_5 = node.create_publisher(Int32MultiArray, '/self_collision', 10)
        self.self_collision = Int32MultiArray()
        self.self_collision.data = [0]

        self.subscription_3 = node.create_subscription(
            Bool,
            'reach',
            self.reach_control_rcvd,
            10
        )


    def setup(self):
        self.q = np.radians(np.array([0]*self.chain.dofs).reshape((-1,1)))
        self.Node = np.random.randint(7,8)
        self.startpoint = np.random.randint(1,9-self.Node)
        xyzgoal = self.findgoal(self.Node, self.startpoint)
        self.pgoal = xyzgoal.flatten('F').reshape((self.Node *3,1))
        self.setting = False
        self.setting_target = True


    def reach_control_rcvd(self,msg):
        self.reach_status = msg.data
        if msg.data and len(self.segments)==0:
            # self._set_target(self.pos, self.quat)
            pass

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []

        # xy case
        for ind in range(1,10):
            j.append('joint%i'%ind)

        # yz case
        # for ind in range(1,5):
        #     j.append('joint%ia'%ind)
        #     j.append('joint%ib'%ind)
        # j.append('finalspin')

        return j

    def findgoal(self, Node=7, startpoint = 1):
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
            J_tips = np.empty((0,self.chain.dofs))

            start_point = self.startpoint
            for i in range(start_point,start_point + self.Node):
                x_tips = np.append(x_tips, p_from_T(self.chain.data.T[i]), axis=0)
                J_tips = np.append(J_tips, self.chain.Jv_tip(i), axis=0)

            Jinv = np.linalg.pinv(J_tips, 0.01)
            # qdot = Jinv @ (self.pgoal - x_tips)
            q[j+1] = q[j] + 0.1*Jinv @ (self.pgoal - x_tips)

            self.chain.setjoints(q[j+1])


        return q[100]

    def _set_target(self, pos=None, quat=None):
        l = 9/41
        if pos==None:
            pos = [np.random.uniform(-l,l),
                    np.random.uniform(-l,l),
                    np.random.uniform(-l,l)]
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
        Rf = R_from_quat(np.array(quat))#-np.array([0.976, 0.0,0.0,-0.216]))
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
            index = np.argmin(np.abs(w-1.))
            u = v[:,index].reshape((3,1))
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
        p_offset = np.array([+.02,-.02,-.05]).reshape((3,1))
        p_offset = Rf @ R_from_quat(np.array([-0.976, 0., 0., +0.216])) @ p_offset
        pose.position.x = pos[0] + p_offset[0][0]
        pose.position.y = pos[1] + p_offset[1][0]
        pose.position.z = pos[2] + p_offset[2][0]
        quat = quat_from_R(
            Rf @ R_from_quat(np.array([-0.976, 0., 0., +0.216]))
        )
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        self.publisher_3.publish(pose)

    def set_goal(self, pos):
        self.goal_list = []
        for p in pos:
            self.goal_list.append([p.x,p.y,p.z])

    def check_touch(self):
        x0 = self.chain.ptip()
        R0 = self.chain.Rtip()
        xf, Rf = self.xf, self.Rf
        condition = [np.allclose(R0, Rf, atol=.005), np.allclose(x0, xf, atol=.005), self.ta>=self.t0+8]
        if (condition[0] and condition[1]) or condition[2]:
            if condition[0] and condition[1]:
                self.target_touched += 1
                self.node.get_logger().info('touch target')
            else:
                self.node.get_logger().info('times up')
            msg = Int32MultiArray()
            msg.data = [self.target_touched, self.target_total]
            self.publisher_2.publish(msg)
            if len(self.segments)>0: seg = self.segments.pop(0)
            self._set_target()

    # Evaluate at the given time.
    def evaluate(self, ta, dt):
        self.ta = ta
        if ta>100:
            if not self.end:
                self.end = True
                self.node.get_logger().info('#############################################')
                self.node.get_logger().info('obj coliision #')
                self.node.get_logger().info(str(self.collision_count))
                self.node.get_logger().info('total target #')
                self.node.get_logger().info(str(self.target_total))
                self.node.get_logger().info('touched target #')
                self.node.get_logger().info(str(self.target_touched))
                self.node.get_logger().info('self collision #')
                self.node.get_logger().info(str(self.self_colli))
                self.node.get_logger().info('#############################################')
            phase = Bool()
            phase.data = False
            self.publisher_4.publish(phase)
            return (self.q.flatten().tolist(), np.zeros((self.chain.dofs,1)).flatten().tolist())
        T = 25
        phase1 = 1
        phase2 = 5
        self_collision = None
        collision_count = None

        if ta%T<=phase1:
            if self.setting:
                self.setup()
                q = self.tie_knot()
            else:
                q = self.q
            qdot = np.zeros((self.chain.dofs,1))
            self.q = q
            self.chain.setjoints(self.q)
            self.segments = []
            # _, _, _, self_collision = repulsive(self.chain)

        elif ta%T<=phase2:
            qdot1, J1, Jinv1, collision_count, collision_arr = avoid_objects(self.goal_list, self.chain)
            qdot2, J2, Jinv2, self_collision = repulsive(self.chain)
            # qdot3, J3, Jinv3 = target_pinv(self.chain, self.Rf, self.xf)
            qdot3, J3 = nominal(self.q, self.q_nom)

            qdot = qdot1 + nullspace(J1,Jinv1) @ (qdot2 + nullspace(J2,Jinv2) @ (qdot3))# + nullspace(J3,Jinv3) @ qdot4))
            q = self.q + qdot*dt
            self.q = q
            self.chain.setjoints(self.q)

            self.err = np.zeros((6,1))

        elif self.reach_status:
            if self.setting_target:
                self.setting_target = False
                self._set_target()
                self.setting = True

            qdot1, J1, Jinv1, collision_count, collision_arr = avoid_objects(self.goal_list, self.chain)

            spline_attract = True
            if True: #self.ta>self.t0+6 and spline_attract:
                qdot2, J2, Jinv2 = target_pinv(self.chain, self.Rf, self.xf)
                qdot2 *= 5
            else:
                qdot2, J2, Jinv2, xd, Rd = target_spline(self.chain, self.segments[0],ta-self.t0,self.err)
            _, _, _, self_collision = repulsive(self.chain)
            qdot3, J3 = nominal(self.q, self.q_nom)

            qdot = qdot1 + nullspace(J1,Jinv1) @ (qdot2 + nullspace(J2,Jinv2) @ (qdot3))# + nullspace(J3,Jinv3) @ qdot4))
            q = self.q + qdot*dt
            self.q = q
            self.chain.setjoints(self.q)
            if False: #not spline_attract or self.ta<=self.t0+6:
                self.err = np.vstack((ep(xd,self.chain.ptip()), eR(Rd,self.chain.Rtip())))
            self.check_touch()
        
        else:
            qdot1, J1, Jinv1, collision_count, collision_arr = avoid_objects(self.goal_list, self.chain)
            qdot2, _ = nominal(self.q, self.q_nom)

            qdot = qdot1 + nullspace(J1,Jinv1) @ qdot2
            q = self.q + qdot*dt
            self.q = q
            self.chain.setjoints(self.q)

        # pub 1
        if collision_count!=None:
            c_arr = Int32MultiArray()
            if collision_count>0 and not self.colliding: 
                print('collision detected: #',collision_count)
                self.collision_count += collision_count
                self.colliding = True
            if collision_count==0 and self.colliding:
                self.colliding = False
            collision_arr.insert(0, self.collision_count)
            c_arr.data = collision_arr
            self.publisher_.publish(c_arr)
        # pub 4
        phase = Bool()
        phase.data = ta%T>phase1
        self.publisher_4.publish(phase)
        # pub 5
        if self_collision!=None: 
            self.self_colli += self_collision
            self.self_collision.data = [self_collision]
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
