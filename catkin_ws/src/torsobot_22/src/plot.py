#! /usr/bin/env python3
"""
Created on Wed May 18 14:03:57 2022

@author: Yuxiao
@editor: JungWoo
"""
import argparse
import pandas as pd
import matplotlib.pyplot as plt


if __name__ == "__main__":  # noqa: C901
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_path", help=".csv file path", type=str, required=True)
    args = parser.parse_args()

    data_path = args.data_path
    raw_data = pd.read_csv(data_path)
    data = pd.DataFrame(raw_data)
    data["Time"] = data["Time"] - data["Time"][0]
    
    fig, ax = plt.subplots(dpi=120)
    
    data.plot(kind='line',y='imu_angle',x='Time', color='red', ax=ax)
    



    
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lean Angle[deg]")
    
    # ax.set_ylim(0,30)
    
    
    ax2=ax.twinx()
    data.plot(kind='line',y='Wheel_speed',x='Time', color='blue', ax=ax2)
    ax2.set_ylabel("Speed[rad/s]")
    

    plt.show()