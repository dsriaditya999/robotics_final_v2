import numpy as np
from numpy import pi, cos, sin, sqrt
from hw5code.TransformHelpers  import *

def nullspace(J,Jinv):
    return np.eye(J.shape[1])- Jinv @ J

def nominal(q,q_nom):
    qdot = q_nom - q
    filter1 = qdot>pi
    filter2 = qdot<-pi
    qdot[filter1] -= ((qdot[filter1]-pi)//(2*pi) +1) * 2*pi
    qdot[filter2] += ((-pi-qdot[filter2])//(2*pi) +1) * 2*pi
    J = np.empty((0,q.shape[0]))
    return qdot, J

def avoid_objects(goal_list, chain, gamma=0.01):
    qdot = np.zeros((chain.dofs,1))
    J = np.empty((0,chain.dofs))
    eRR = np.empty((0,1))
    collision_count = 0
    collision_threshold = 0.072
    collision_arr = []
    for goal in goal_list:
        pd = np.array(goal).reshape((3,1))

        p_links = np.empty((3,0))
        dofs = chain.dofs
        for dof in range(dofs):
            p_links = np.append(p_links, p_from_T(chain.data.T[dof]), axis=1)
        
        tip_dof = np.argmin(np.linalg.norm(pd-p_links,axis=0))

        tip_dis = np.min(np.linalg.norm(pd-p_links,axis=0))
        if tip_dis<collision_threshold:
            collision_count +=1
            collision_arr.append(1)
        else:
            collision_arr.append(0)

        pd_tip = p_from_T(chain.data.T[tip_dof]).reshape((3,1))

        if pd[2][0]<pd_tip[2][0]:
            continue

        eR1 = pd[:2,:]
        eR2 = pd_tip[:2,:]
        eR12 = ep(eR2,eR1)
        eRn = np.linalg.norm(eR12)#ep(pd,pd_tip))
        eRn_large = np.linalg.norm(ep(pd,pd_tip))
        eR12 /= eRn*eRn_large
        eRR = np.append(eRR, eR12, axis=0)
        J = np.append(J,chain.Jv_tip(tip_dof)[:2,:],axis=0)
    
    Jinv = np.linalg.pinv(J, gamma)
    qdot = Jinv @ eRR #*0.5

    return qdot, J, Jinv, collision_count, collision_arr

def repulsive(chain, gamma=0.01):
    repulsive_range = 0.2
    collision_threshold = 0.072

    p_joints = np.empty((3,0))
    dofs = chain.dofs
    for dof in range(dofs):
        p_joints = np.append(p_joints, p_from_T(chain.data.T[dof]), axis=1)
    
    self_collision = 0
    J = np.empty((0,dofs))
    x_repulsive = np.empty((0,1))
    for i in range(p_joints.shape[1]-1):
        dP = p_joints[:,i+1]-p_joints[:,i]
        dP_norm = np.linalg.norm(dP)
        for j in range(i+2, p_joints.shape[1]):
            dPj = p_joints[:,j]-p_joints[:,i]
            proj = np.dot(dP,dPj)/dP_norm
            perpend = dPj-dP/dP_norm*proj
            if proj>0 and proj<dP_norm and np.linalg.norm(perpend)<repulsive_range:
                if np.linalg.norm(perpend)<collision_threshold: self_collision += 1
                xj_rep = p_joints[:,j]-0.5*(p_joints[:,i+1]+p_joints[:,i])
                xj_rep /= np.linalg.norm(perpend)**2
                Jj = chain.Jv_tip(j)
                Jj[:,:i+1] = 0
                x_repulsive = np.append(x_repulsive, xj_rep.reshape((3,1)), axis=0)
                J = np.append(J, Jj, axis=0)

    Jinv = np.linalg.pinv(J, gamma)
    qdot = Jinv@x_repulsive *0.05
    return qdot, J, Jinv, self_collision

def target_spline(chain, segment, t_relative, pre_err, gamma=0.01, lam=10):
    J = np.vstack((chain.Jv(),chain.Jw()))
    
    (xd, xdot) = segment[0].evaluate(t_relative)
    (theta_d, wd) = segment[1].evaluate(t_relative)
    eh = segment[2]
    R0 = segment[3]
    Rd = R0 @ Rote(eh,theta_d)
    wd = R0 @ eh * wd
    pd = np.vstack((xdot,wd))

    Jinv = np.linalg.pinv(J,gamma)
    qdot = Jinv @ (pd + lam*pre_err)
    return qdot, J, Jinv, xd, Rd

def target_pinv(chain, Rd, xd, gamma=0.01):
    x0 = chain.ptip()
    R0 = chain.Rtip()
    xdot = ep(xd, x0)
    Rdot = eR(Rd,R0)

    J = np.vstack((chain.Jv(),chain.Jw()))
    pd = np.vstack((xdot,Rdot))
    Jinv = np.linalg.pinv(J,gamma)
    qdot = Jinv @ pd *10
    return qdot, J, Jinv
