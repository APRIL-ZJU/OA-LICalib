#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 19:49:36 2021

@author: ha
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def load_lidar_intrinsic_data():
    filenames = []
    filenames.append('../data/lidar_intrinsic_gt.txt')
    filenames.append('../data/bag/simu_bag/lidar-13.txt')
 
    df = pd.DataFrame()
    lidar_gt = pd.DataFrame()
    for filename in filenames:
        print "Load lidar intrinsic data from ", filename
        
        data = pd.read_csv(filename, ',')
        dsr_col = np.abs(np.array(range(16)) - 15)
        data['dsr'] = dsr_col
        
        data['vert_degree'] = data['vert_degree'] - np.round(data['vert_degree'])
        data['dist_scale'] = (data['dist_scale'] - 1) * 100
    
        if (filename == filenames[0]):
            lidar_gt = data
        else:
            df = df.append(data)
        
    lidar_list = [[],[],[],[],[],[]]
    for dsr in range(16):
        for col_y, idx in zip(df.columns, range(6)):
            data_col = df.loc[df['dsr'] == dsr][col_y].to_list()
            lidar_list[idx].append(data_col)
    
    for idx in range(6):
        lidar_list[idx] = np.array(lidar_list[idx]).T
    
    print lidar_gt.columns
    return lidar_list, lidar_gt
 
if __name__ == "__main__":
    est_list, gt_df =  load_lidar_intrinsic_data()
    
    gt_df['dsr'] = gt_df['dsr'] + 1
    
    ylabels = ['$s\  (\%)$', r'$\delta \rho \ (mm)$', '$V\  (mm)$', '$H\  (mm)$', 
           '$\delta \phi \ (deg)$', '$\delta \Theta \ (deg)$']
    
    fig, axs = plt.subplots(nrows=2, ncols=3, sharex=True, sharey=False, figsize=(18, 6))
    plt.subplots_adjust(wspace=0.3, hspace=None)
    for idx, y_label, col_y in zip(range(6), ylabels, gt_df.columns):
        ax = axs[idx/3][idx%3]
        
        ax.boxplot(est_list[idx])
        ax.plot(gt_df['dsr'],  gt_df[col_y],  'b.', label='groundtruth')
        
        
        ax.set_xlabel('beam', fontsize=14)
        ax.set_ylabel(y_label, fontsize=14)

    # add x-tick labels
    x_ticks_now = [y + 1 for y in range(16)]
    x_ticks_after = [y for y in range(16)]
    plt.setp(axs, xticks=x_ticks_now, xticklabels=x_ticks_after)
    #fig.autofmt_xdate(ha='center')

    save_file_name = './lidar_intrinsic.pdf'
    plt.savefig(save_file_name, bbox_inches='tight')
    plt.show()