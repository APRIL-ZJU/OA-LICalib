#!/usr/bin/env python

import rospy
from oa_licalib.msg import imu_array
from oa_licalib.msg import pose_array
import nav_msgs.msg
import geometry_msgs.msg
import message_filters

import numpy as np
from transforms3d.euler import quat2euler
import liso_plots as plots

            
class ResultViewer() :
    def __init__(self):
        self.draw_pose = False
        self.draw_imu = False
        self.draw_spline_traj = False
        self.pose_raw = []
        self.pose_est = []
        self.imu_raw = []
        self.imu_est = []
        self.spline_traj = []

    def path_msg_2_numpy_array(self, path_msg):
        path_data = []
        for poseStamped in path_msg.poses:
            t = poseStamped.header.stamp.to_sec()
            p = poseStamped.pose.position 
            q = poseStamped.pose.orientation
            quat = [q.w, q.x, q.y, q.z]
            euler = quat2euler(quat)
            euler = (180.0 / np.pi) * np.array(euler)
            
            path_data.append([t, p.x, p.y, p.z, euler[0], euler[1], euler[2]])
        path_data = np.array(path_data)
        return path_data
    
    def pose_msg_2_numpy_array(self, msg):
        pose_data = []
        for (t, p, q) in zip(msg.timestamps, msg.positions, msg.orientations):
            quat = [q.w, q.x, q.y, q.z]
            euler = quat2euler(quat)
            euler = (180.0 / np.pi) * np.array(euler)
            pose_data.append([t, p.x, p.y, p.z, euler[0], euler[1], euler[2]])

        pose_data = np.array(pose_data)
        return pose_data

    def imu_msg_2_numpy_array(self, msg, outscrean = False):
        imu_data = []
        for (t, gyro, accel) in zip(msg.timestamps, msg.angular_velocities, msg.linear_accelerations):
            imu_data.append([t, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z])
            if outscrean:
                print gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z

        imu_data = np.array(imu_data)
        return imu_data
       
    def TrajectoryCallback(self, msg):
        self.spline_traj = self.path_msg_2_numpy_array(msg)
        print '[TrajectoryCallback]', len(self.spline_traj) 
        if len(self.spline_traj) > 0:
            self.draw_spline_traj = True
        
    def PoseCallback(self, est_msg, raw_msg):
        self.pose_raw = self.pose_msg_2_numpy_array(raw_msg)
        self.pose_est = self.pose_msg_2_numpy_array(est_msg)
        print '[PoseCallback]      ', len(self.pose_raw), len(self.pose_est)
        if len(self.pose_raw) > 0 and len(self.pose_est) > 0:
            self.draw_pose = True

    def IMUCallback(self, est_msg, raw_msg):
        self.imu_raw = self.imu_msg_2_numpy_array(raw_msg)
        self.imu_est = self.imu_msg_2_numpy_array(est_msg)
        print '[IMUCallback]       ', len(self.imu_raw), len(self.imu_est)
        if len(self.imu_raw) > 0 and len(self.imu_est) > 0:
            self.draw_imu = True
            
    def DrawFigure(self):
        draw_pose_ = self.draw_pose
        draw_imu_ = self.draw_imu;
        draw_spline_traj_ = self.draw_spline_traj;
        
        # Clear status
        self.draw_pose = False
        self.draw_imu = False
        self.draw_spline_traj = False
        
        plots.DrawFigure(draw_pose_, draw_imu_, draw_spline_traj_,
                         self.pose_raw, self.pose_est, 
                         self.imu_raw, self.imu_est, 
                         self.spline_traj, 
                         "")
        

if __name__ == '__main__':
    rospy.init_node('plot_node', anonymous=True)
    viewer = ResultViewer()

    rospy.Subscriber('/lidar_trajectory', nav_msgs.msg.Path, viewer.TrajectoryCallback)
    
    pose_est_sub = message_filters.Subscriber('/path_est', pose_array)
    pose_raw_sub = message_filters.Subscriber('/path_raw', pose_array)
    ts_pose = message_filters.TimeSynchronizer([pose_est_sub, pose_raw_sub], 1)
    ts_pose.registerCallback(viewer.PoseCallback)

    imu_est_sub = message_filters.Subscriber('/imu_est_array', imu_array)
    imu_raw_sub = message_filters.Subscriber('/imu_raw_array', imu_array)
    ts_imu = message_filters.TimeSynchronizer([imu_est_sub, imu_raw_sub], 1)
    ts_imu.registerCallback(viewer.IMUCallback)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        viewer.DrawFigure()
        rate.sleep()
