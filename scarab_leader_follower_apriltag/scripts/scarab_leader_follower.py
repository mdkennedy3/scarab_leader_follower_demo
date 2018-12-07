#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Vector3 as geom_vector

from scarab_leader_follower_apriltag.msg import velocities

import tf

#Geommsgs
# This expresses velocity in free space broken into its linear and angular parts.
#Vector3  linear
#Vector3  angular
#use list to fill this param
class tf_pose_object(object):
    def __init__(self):
        self.trans = []
        self.rot = []


class Control_follower_scarab(object):
    def __init__(self, scarab_pub_44,scarab_pub_46,Kp):

        # self.scarab_name = '/scarab44'
        self.camera_frame44 = '/camera_link'
        self.camera_frame46 = '/camera46_link'
        self.vmax = 0.6
        self.wmax = 0.6
        self.linear_vect = geom_vector()
        self.angular_vect = geom_vector()
        #intialize:
        self.linear_vect.x = 0
        self.linear_vect.y = 0
        self.linear_vect.z = 0
        self.angular_vect.x = 0
        self.angular_vect.y = 0
        self.angular_vect.z = 0
        self.scarab_twist_msg = velocities()
        self.scarab_twist = Twist()
        # self.scarab_msg_pub = scarab_msg_pub
        self.scarab_pub_44 = scarab_pub_44
        self.scarab_pub_46 = scarab_pub_46
        # self.seq_iter = 1
        tag_locations = ['back_tag','right_tag','left_tag']
        # self.tag_list_first_follower = ['/t3_kin','/t2_kin','/t11_kin'] #where 3 is in back, 2 is on right, 11 is on left
        # self.tag_list_second_follower = ['/t100_kin','/t101_kin','/t102_kin'] #where 100 is in back, 101 is on right, 102 is on left
        self.tag_list_first_follower = dict(zip(tag_locations,['/t3_kin','/t2_kin','/t11_kin']))  #where 3 is in back, 2 is on right, 11 is on left
        self.tag_list_second_follower = dict(zip(tag_locations,['/t100_kin','/t101_kin','/t102_kin'])) #where 100 is in back, 101 is on right, 102 is on left

        self.Kp = np.matrix([[Kp[0],0],[0,Kp[1]]])
        self.TF_T3_T2 = []
        self.TF_T3_T11 = []

        self.TF_T100_T101 = []
        self.TF_T100_T102 = []

        self.tf_listener = tf.TransformListener()

        self.reverse_delta = 0.01  #threshold to reverse controller for backdriving system

        #Robotus
        self.scarab_control_lead_dist = 0.8#0.15 #distance in meters of the controllable point 'P'
        self.follow_distance = 0.0 #units of meters

    def write_msgs(self,msg_pub,control_vel):

        self.linear_vect.x = np.min([control_vel[0], self.vmax])
        self.angular_vect.z = np.min([control_vel[1], self.wmax])

        self.scarab_twist.linear = self.linear_vect
        self.scarab_twist.angular = self.angular_vect
        msg_pub.publish(self.scarab_twist)

    def rot_to_quat(self,R):
        R = np.matrix(R)
        q0 = 0.5* np.sqrt(R[0,0] + R[1,1] + R[2,2] + 1 )
        q1 = np.divide((R[1,2] - R[2,1]),(4*q0))
        q2 = np.divide((R[2,0] - R[0,2]),(4*q0))
        q3 = np.divide((R[0,1] - R[1,0]),(4*q0))
        q = np.matrix([[q1],[q2],[q3],[q0]])
        return q
    def quat_to_rot(self,q):
        #This rotational form is the 'transformation' version, imagine rotating one reference frame into the other, this describes the location of the transformation axis: e.g. for 90deg about v =[1,1,1], vector would be 
        # i to j, but transform would take i to k (z axis would sit on old x axis).
        if isinstance(q,list) or isinstance(q,tuple):
            q0 = q[3]
            q1 = q[0]
            q2 = q[1]
            q3 = q[2]
        else:
            q0 = q.item(3)
            q1 = q.item(0)
            q2 = q.item(1)
            q3 = q.item(2)

        R11 = 2*(q0**2 + q1**2) - 1
        R21 = 2*(q1*q2 - q0*q3)
        R31 = 2*(q1*q3 + q0*q2)

        R12 = 2*(q1*q2 + q0*q3)
        R22 = 2*(q0**2 + q2**2) - 1
        R32 = 2*(q2*q3 - q0*q1)
        
        R13 = 2*(q1*q3 - q0*q2)
        R23 = 2*(q2*q3 +q0*q1)
        R33 = 2*(q0**2 + q3**2) - 1
        
        Rmat = np.matrix([[R11, R12, R13],
                        [R21, R22 ,R23],
                        [R31, R32, R33]])
        '''NOTE: this returns exactly what quat is, hence R1 -> q -> R2  then R1 == R2, this was tested '''
        #Note that this is frame rotation {frame}^{R}_{base}  [gimmic: frame has neg in front (R21 is neg)]
        return Rmat
    def TF_btw_frames(self,start_tf,end_tf):
        #this script finds the transform of start_tf in end_tf frame or is the point transform from end to start tf (in end frame)
        transform_obj = tf_pose_object()
        try:
            t = self.tf_listener.getLatestCommonTime(start_tf,end_tf)
            (transform_obj.trans, transform_obj.rot) = self.tf_listener.lookupTransform(end_tf,start_tf,t)
        except:
            #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            transform_obj.trans = []
            transform_obj.rot = []
        return transform_obj
    def tf_to_mat(self,tf_obj):
        if len(tf_obj.trans) != 0:
            obj_mat = np.matrix([tf_obj.trans[0],tf_obj.trans[1],tf_obj.trans[2],tf_obj.rot[0],tf_obj.rot[1],tf_obj.rot[2],tf_obj.rot[3]]).T
        else:
            #or make sure this is not entered if info not available
            obj_mat = np.matrix([[],[],[],[],[],[],[]])
        return obj_mat
    def transform_from_pose(self,tf_vect):
        #takes in vector: [x,y,z,q1,q2,q3,qw]_frame1_to_frame2 and returns 4x4 transform of TF_frame1_to_frame2
        pos_vect = tf_vect[:3]
        quat = tf_vect[3:]
        '''Very sure of below, but issues may arise here if you get turned around '''
        R_F1_to_F2 = self.quat_to_rot(quat).T #function returned the opposite, R_f2_f1, so transpose gives same frame transformation
        Transform = np.matrix([[R_F1_to_F2[0,0], R_F1_to_F2[0,1], R_F1_to_F2[0,2], pos_vect[0]],
                               [R_F1_to_F2[1,0], R_F1_to_F2[1,1], R_F1_to_F2[1,2], pos_vect[1]],
                               [R_F1_to_F2[2,0], R_F1_to_F2[2,1], R_F1_to_F2[2,2], pos_vect[2]],
                               [0,0,0,1]])
        return Transform
    def pose_from_transform(self, TF_obj):
        R = TF_obj[0:3,0:3]
        # print('inside callback')
        # print('R:', R)

        v = TF_obj[0:3,3]
        # print('v:', v)
        quat = self.rot_to_quat(R) #q1,q2,q3,q0
        # print('quat: ', quat)
        tfp = np.matrix(np.vstack([v,quat]))
        # print('pose from callback', tf_pose)
        return tfp
    def get_inter_tag_TF(self,tag_list):
        TF_Tmain_Ti_test = self.TF_btw_frames(tag_list['back_tag'],tag_list['right_tag']) #TF_T3_T11_test or TF_T100_101
        TF_Tmain_Tj_test = self.TF_btw_frames(tag_list['back_tag'],tag_list['left_tag']) #TF_T3_T2_test or TF_T100_102
        TF_Tmain_Ti = []
        TF_Tmain_Tj = []
        if len(TF_Tmain_Ti_test.trans) != 0:
            TF_Tmain_Ti = TF_Tmain_Ti_test
        if len(TF_Tmain_Tj_test.trans) != 0:
            TF_Tmain_Tj = TF_Tmain_Tj_test

        return [TF_Tmain_Ti, TF_Tmain_Tj]   #[TF_T3_T11, TF_T3_T2] or [TF_T100_101, TF_T100_102]


    def main_controller(self,scarab_pub,tag_list_follower, camera_frame, TF_Tback_Tright, TF_Tback_Tleft):
        #find the pose of the leader in local frame
        TF_Tback_to_follow = self.TF_btw_frames(tag_list_follower['back_tag'],camera_frame) #ends in camera frame
        TF_Tright_to_follow = self.TF_btw_frames(tag_list_follower['right_tag'],camera_frame) #ends in camera frame
        TF_Tleft_to_follow = self.TF_btw_frames(tag_list_follower['left_tag'],camera_frame) #ends in camera frame


        [TF_Tback_Tright,TF_Tback_Tleft] = self.get_inter_tag_TF(tag_list_follower)

        des_pose = np.matrix([self.scarab_control_lead_dist,0]).T #initialize as 0 err
        if len(TF_Tback_to_follow.trans) != 0:
            # print('using Tback which is main')
            #use back tag first
            R_Tback_to_follow = self.quat_to_rot(TF_Tback_to_follow.rot)
            # z_vect = R_Tback_to_follow[:,2]
            z_vect = R_Tback_to_follow[2,:].T

            des_pose[0] = TF_Tback_to_follow.trans[0] + self.follow_distance*z_vect[0] #x
            des_pose[1] = TF_Tback_to_follow.trans[1] + self.follow_distance*z_vect[1]#y

        elif len(TF_Tright_to_follow.trans) != 0:
            #use right tag second
            # print(TF_Tright_to_follow.trans)
            # print('using Tright')
            if not isinstance(TF_Tback_Tright,list):
                #intertag TF is known: 
                TF_Tback_Tright_vect = self.tf_to_mat(TF_Tback_Tright)
                TF_Tback_Tright_mat = self.transform_from_pose(TF_Tback_Tright_vect)
                TF_Tright_to_follow_vect = self.tf_to_mat(TF_Tright_to_follow)
                TF_Tright_to_follow_mat = self.transform_from_pose(TF_Tright_to_follow_vect)
                TF_Tback_to_follow_surr_mat = TF_Tright_to_follow_mat*TF_Tback_Tright_mat #surragate TF
                TF_Tback_to_follow_surr_pose = self.pose_from_transform(TF_Tback_to_follow_surr_mat)
                
                R_Tback_to_follow =  self.quat_to_rot(TF_Tback_to_follow_surr_pose[3:])
                # z_vect = R_Tback_to_follow[:,2]
                z_vect = R_Tback_to_follow[2,:].T


                des_pose[0] = TF_Tback_to_follow_surr_pose[0] + self.follow_distance*z_vect[0] #x
                des_pose[1] = TF_Tback_to_follow_surr_pose[1] + self.follow_distance*z_vect[1]#y


            else:
                #Tback was never visible with Tright
                R_Tright_to_follow = self.quat_to_rot(TF_Tright_to_follow.rot)
                # neg_x_vect = -R_Tright_to_follow[:,1]
                neg_x_vect = -R_Tright_to_follow[0,:]

                des_pose[0] = TF_Tright_to_follow.trans[0] + self.follow_distance*neg_x_vect[0] #x
                des_pose[1] = TF_Tright_to_follow.trans[1] + self.follow_distance*neg_x_vect[1]#y

        elif len(TF_Tleft_to_follow.trans) != 0:
            #use left tag last

            if not isinstance(TF_Tback_Tleft,list):
                #intertag TF is known: 
                TF_Tback_Tleft_vect = self.tf_to_mat(TF_Tback_Tleft)
                TF_Tback_Tleft_mat = self.transform_from_pose(TF_Tback_Tleft_vect)
                TF_Tleft_to_follow_vect = self.tf_to_mat(TF_Tleft_to_follow)
                TF_Tleft_to_follow_mat = self.transform_from_pose(TF_Tleft_to_follow_vect)
                TF_Tback_to_follow_surr_mat = TF_Tleft_to_follow_mat*TF_Tback_Tleft_mat #surragate TF
                TF_Tback_to_follow_surr_pose = self.pose_from_transform(TF_Tback_to_follow_surr_mat)
                
                R_Tback_to_follow =  self.quat_to_rot(TF_Tback_to_follow_surr_pose[3:])
                # z_vect = R_Tback_to_follow[:,2]
                z_vect = R_Tback_to_follow[2,:].T

                des_pose[0] = TF_Tback_to_follow_surr_pose[0] + self.follow_distance*z_vect[0] #x
                des_pose[1] = TF_Tback_to_follow_surr_pose[1] + self.follow_distance*z_vect[1]#y


            else:
                #Tback was never visible with Tright
                R_Tleft_to_follow = self.quat_to_rot(TF_Tleft_to_follow.rot)
                x_vect = R_Tleft_to_follow[0,:]
                des_pose[0] = TF_Tleft_to_follow.trans[0] + self.follow_distance*x_vect[0] #x
                des_pose[1] = TF_Tleft_to_follow.trans[1] + self.follow_distance*x_vect[1]#y

        else:
            #default stop if no tag is visible
            des_pose[0] = self.scarab_control_lead_dist #local origin or just same position
            des_pose[1] = 0.0 #local origin or just same position

        #find the control input in linear and angular velocity, error:


        pos_err = self.Kp*(des_pose - np.matrix([self.scarab_control_lead_dist,0]).T)

        if pos_err[0] < -self.reverse_delta:
            control_vel = np.linalg.pinv(np.matrix([[1,0],[0,-self.scarab_control_lead_dist]]))*(-pos_err) #controlling on point p
        else:
            control_vel = np.linalg.pinv(np.matrix([[1,0],[0,self.scarab_control_lead_dist]]))*pos_err #controlling on point p



        #if this were in vicon frame, then subscribe to teh pose topics and place all poses desired and actual in world frame, then get angle in world frame: 
        # self.control_vel = np.linalg.pinv(np.matrix([[np.cos(self.rob_theta),-np.sin(self.rob_theta)*self.scarab_control_lead_dist],[np.sin(self.rob_theta), np.cos(self.rob_theta)*self.scarab_control_lead_dist]]))*self.pos_err_44

        #Command these velocities
        self.write_msgs(scarab_pub, control_vel)
        return [TF_Tback_Tright,TF_Tback_Tleft]



    def run_controller(self,robot):
        if robot in 'scarab44':
            #run controller for robot 44:
            [self.TF_T3_T11,self.TF_T3_T2] = self.main_controller(self.scarab_pub_44,self.tag_list_first_follower, self.camera_frame44, self.TF_T3_T11,self.TF_T3_T2)

        if robot in 'scarab46':
            #run controller for robot 46:
            [self.TF_T100_T101,self.TF_T100_T102] = self.main_controller(self.scarab_pub_46,self.tag_list_second_follower, self.camera_frame46,self.TF_T100_T101,self.TF_T100_T102)



def main():
    rospy.init_node('leader_follower')

    #make msgs for cmd vel publisher and pose listener
    #Make Topics Publishers
    # scarab_pub_44 = rospy.Publisher('/cmd_vel44',velocities,queue_size = 1)
    scarab_pub_44 = rospy.Publisher('/cmd_vel44',Twist,queue_size = 1) #first follower
    scarab_pub_46 = rospy.Publisher('/cmd_vel46',Twist,queue_size = 1) #second follower, follows 44

    Kp = [0.3,0.3] #[v,w]

    #main function
    control_obj = Control_follower_scarab(scarab_pub_44,scarab_pub_46,Kp)
    print('running...')
    while not rospy.is_shutdown():
        # control_obj.main_controller_44()
        # control_obj.main_controller_46()
        control_obj.run_controller('scarab44')
        control_obj.run_controller('scarab46')

    print('program stopped.')

    #need to use: 
    #from cmd_vel to cmd_vel44




if __name__ == '__main__':
    main()
