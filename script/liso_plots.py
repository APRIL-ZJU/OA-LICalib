# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 09:50:18 2019

@author: ha
"""

import numpy as np
import pylab as pl
from mpl_toolkits.mplot3d import Axes3D 

import PlotCollection
from matplotlib.backends.backend_pdf import PdfPages

fig_num = 2020

def plotVectorOverTime(times, values, figname="", ylabel="", color='r-', label="", fno=1, clearFigure=True, lw=1):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    if figname is not "":    
        f.suptitle(figname)
    for r in range(0, 3):
        pl.subplot(3, 1, r + 1)
        pl.plot(times, values[:, r], color, lw=lw, label=label)
        pl.grid(True)
        if ylabel is not "":    
            pl.xlabel("time (s)")
            pl.ylabel(ylabel)
        if label is not "":
            pl.legend()

# error ($m/s^2$) ($rad/s$)
def plotErrorPerAxis(times, est_data, gt_data, figname="", ylabel="", fno=1):
    errors = est_data - gt_data
   
    f = pl.figure(fno)
    f.suptitle(figname)
    
    for i in xrange(3):
        pl.subplot(3, 1, i+1)
        pl.plot(times, errors[:,i])
        pl.xlabel("time (s)")
        pl.ylabel(ylabel)
        pl.grid(True)
#        sigma = 0
#        pl.plot(np.array([0., errors.shape[0]]), sigma * 3.* np.ones(2), 'r--')
#        pl.plot(np.array([0., errors.shape[0]]), -sigma * 3.* np.ones(2), 'r--')
        #pl.xlim([0., errors.shape[1]])

            
# plots angular velocity of the body fixed spline versus all imu measurements
def plotAngularVelocities(times, est_data, gt_data, fno=1):
    # plot the imu measurements
    plotVectorOverTime(times, gt_data,
                       figname="Comparison of predicted and measured angular velocities (body frame)",
                       ylabel="ang. velocity ($rad/s$)",
                       color='r-',
                       label="imu measurememt",
                       fno=fno, clearFigure=False)

    # plot the predicted measurements
    plotVectorOverTime(times, est_data,
                       figname="",
                       ylabel="",
                       color='b',
                       label="spline",
                       fno=fno, clearFigure=False)

        
def plotAccelerations(times, est_data, gt_data, fno=1):
    # plot the imu measurements
    plotVectorOverTime(times, gt_data,
                       figname="Comparison of predicted and measured specific force",
                       ylabel="specific force ($m/s^2$)",
                       color='r-',
                       label="imu measurememt",
                       fno=fno, clearFigure=False)

    # plot the predicted measurements
    plotVectorOverTime(times, est_data,
                       figname="",
                       ylabel="",
                       color='b',
                       label="spline",
                       fno=fno, clearFigure=False)
                    

def plotPositionFigure(times, est_data, gt_data, fno=1) :

    # plot the measurements
    plotVectorOverTime(times, gt_data,
                       figname="Comparison of predicted and measured position",
                       ylabel="position ($m$)",
                       color='r-',
                       label="measurememt",
                       fno=fno, clearFigure=False)
    
    # plot the predicted measurements
    plotVectorOverTime(times, est_data,
                       figname="",
                       ylabel="",
                       color='b',
                       label="spline",
                       fno=fno, clearFigure=False) 


def plotOrientationFigure(times, est_data, gt_data, fno=1) :

    # plot the measurements
    plotVectorOverTime(times, gt_data,
                       figname="Comparison of predicted and measured euler",
                       ylabel="euler ($degree$)",
                       color='r-',
                       label="measurememt",
                       fno=fno, clearFigure=False)
    
    # plot the predicted measurements
    plotVectorOverTime(times, est_data,
                       figname="",
                       ylabel="",
                       color='b',
                       label="spline",
                       fno=fno, clearFigure=False) 
    
def plot3DTrajectoryIn2D(traj, fno=1):
    fig = pl.figure(fno)
    fig.suptitle('traj in 2D')
    
    x, y, z= traj[:,0], traj[:,1], traj[:,2]  
    
    pl.subplot(3, 1, 1)
    pl.plot(x, y)
    pl.xlabel("X $(m)$")
    pl.ylabel("Y $(m)$") 
    pl.axis("equal")
    pl.grid(True)
    
    
    pl.subplot(3, 1, 2)
    pl.plot(x, z)
    pl.xlabel("X $(m)$")
    pl.ylabel("Z $(m)$") 
    pl.axis("equal")
    pl.grid(True)
    
    
    pl.subplot(3, 1, 3)
    pl.plot(y, z)
    pl.xlabel("Y $(m)$")
    pl.ylabel("Z $(m)$") 
    pl.axis("equal")
    pl.grid(True)
    


    
def plot3DTrajectory(traj, fno=1):
    fig = pl.figure(fno)
    ax = fig.gca(projection='3d')
    
    min_m = np.min(np.min(traj, axis=0))
    max_m = np.max(np.max(traj, axis=0))
    tick_lim = [min_m, max_m]
    
    x, y, z= traj[:,0], traj[:,1], traj[:,2]
    
    ax.plot(x, y, z, label='spline')
    ax.set_xlim3d(tick_lim)
    ax.set_ylim3d(tick_lim)
    ax.set_zlim3d(tick_lim)
    ax.set_xlabel("X ($m$)")
    ax.set_ylabel("Y ($m$)")
    ax.set_zlabel("Z ($m$)") 
    ax.legend() 

    
def DrawFigure(draw_pose=False, draw_imu=False, draw_traj=False, 
               pose_raw=[], pose_est=[], 
               imu_raw=[], imu_est=[],  
               traj=[], 
               filename = ''):
    
    has_sth_draw = draw_pose or draw_imu or draw_traj
    if not has_sth_draw :
        return
    
    figs = list()
    plotter = PlotCollection.PlotCollection("Calibration report")
    
    global fig_num
    #fig_num = 2020

    # for vicon data
    if draw_pose:
        print "DrawPoseFigure"
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotPositionFigure(pose_raw[:,0], pose_est[:,1:4], pose_raw[:,1:4], 
                           fno=f.number) 
        plotter.add_figure("pose: position", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotErrorPerAxis(pose_raw[:,0], pose_est[:,1:4], pose_raw[:,1:4], 
                         figname="position error", 
                         ylabel="error ($m$)", 
                         fno=f.number) 
        plotter.add_figure("pose: position error", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotOrientationFigure(pose_raw[:,0], pose_est[:,4:7], pose_raw[:,4:7], 
                              fno=f.number) 
        plotter.add_figure("pose: orientation", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotErrorPerAxis(pose_raw[:,0], pose_est[:,4:7], pose_raw[:,4:7], 
                         figname="orientation error", 
                         ylabel="error ($degree$)", 
                         fno=f.number) 
        plotter.add_figure("pose: orientation error", f)
        figs.append(f)
        
    if draw_imu:
        print "DrawIMUFigure"
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotAngularVelocities(imu_raw[:,0], imu_est[:,1:4], imu_raw[:,1:4],
                              fno=f.number)
        plotter.add_figure("imu: angular velocities", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotErrorPerAxis(imu_raw[:,0], imu_est[:,1:4], imu_raw[:,1:4], 
                         figname="gyro error", 
                         ylabel="error ($rad/s$)", 
                         fno=f.number) 
        plotter.add_figure("imu: angular velocity error", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotAccelerations(imu_raw[:,0], imu_est[:,4:7], imu_raw[:,4:7],
                          fno=f.number)
        plotter.add_figure("imu: accelerations", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotErrorPerAxis(imu_raw[:,0], imu_est[:,4:7], imu_raw[:,4:7], 
                         figname="accelerations error", 
                         ylabel="error ($m/s^2$)", 
                         fno=f.number) 
        plotter.add_figure("imu: accelerations error", f)
        figs.append(f)
        
    if draw_traj:
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotVectorOverTime(traj[:,0], traj[:,1:4],
                           figname="spline position trajectory",
                           ylabel="position ($m$)",
                           color='b',
                           label="spline_traj",
                           fno=f.number, clearFigure=False)
        plotter.add_figure("spline traj position", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plot3DTrajectoryIn2D(traj[:,1:4], fno=f.number)
        plotter.add_figure("spline 2D traj", f)
        figs.append(f)
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plot3DTrajectory(traj[:,1:4], fno=f.number)
        plotter.add_figure("spline 3D traj", f)
        figs.append(f)
        
        
        fig_num += 1
        f = pl.figure(fig_num)
        f.clf()
        plotVectorOverTime(traj[:,0], traj[:,4:7],
                           figname="spline orientation trajectory",
                           ylabel="euler ($degree$)",
                           color='b',
                           label="spline_traj",
                           fno=f.number, clearFigure=False)
        plotter.add_figure("spline traj euler", f)
        figs.append(f)
        
    #write to pdf
    if filename != "":
        pdf=PdfPages(filename)
        for fig in figs:
            pdf.savefig(fig)
        pdf.close()

    # showOnScreen:
    plotter.show()
