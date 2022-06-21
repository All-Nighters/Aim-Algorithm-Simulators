import Initial_angle
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time

def fit(a0, V0, x, y, Vx, launcher_height, target_slope, err, y_err, v_step, a_step):
    g = 9.8
    count = 0
    a = a0
    V = V0
    func = lambda a : ((V*np.sin(a)*x) / (V*np.cos(a) + Vx)) - ((g * x**2) / (2 * (V * np.cos(a)+Vx)**2)) + launcher_height
    slope = lambda a : ( V*np.sin(a) - ( (g*x) / (V*np.cos(a)+Vx) ) ) / (V * np.cos(a)+Vx)

    while (slope(a) < target_slope - err or slope(a) > target_slope + err):
        
        while (func(a) > y - y_err):
            count += 1
            V -= v_step
            if (slope(a) > target_slope - err and slope(a) < target_slope + err):
                break
        while (func(a) < y + y_err):
            count += 1
            a += a_step
            if (a > (pi/2)):
                print("Angle: ", a)  
                print("Velocity: ", V)  
                print("Count: ", count)
                return a0, V0
            if (slope(a) > target_slope - err and slope(a) < target_slope + err):
                break

    print("Angle: ", a)  
    print("Velocity: ", V)  
    print("Count: ", count)
    return a, V 
    

if __name__ == "__main__":
    g = 9.8

    V = 10.99 # eject velocity
    Vx = 2 # launcher velocity

    upper = 70 # upperbound
    lower = 0 # lowerbound
    step = 0.01

    # goal position
    x = 2
    y = 0.76835
    xPos = x
    yPos = y
    y_error = 0.01

    # desired slope
    target_slope = -0.5
    slope_error = 0.4

    # angle sim settings
    launcher_height = 0.4572


    func = lambda a : ((V*np.sin(a)*x) / (V*np.cos(a) + Vx)) - ((g * x**2) / (2 * (V * np.cos(a)+Vx)**2)) + launcher_height - y
    slope = lambda a : ( V*np.sin(a) - ( (g*x) / (V*np.cos(a)+Vx) ) ) / (V * np.cos(a)+Vx)
    start = time.time()
    a0 = Initial_angle.solve(func, lower=lower, upper=upper, step=0.01, epoch=20)
   

    if (a0 >= 0):

        a0, V = fit(a0, V, x, y, Vx, launcher_height, target_slope, slope_error, y_error, 0.01, 0.1)
        duration = time.time() - start
        print("Duration: ", duration)
        t = np.arange(0,1,0.01)
        x = (V * np.cos(a0) + Vx) * t
        y = V * np.sin(a0) * t - 0.5 * g * (t**2) + launcher_height
        plt.ylim(0, 5)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlabel("Distance (m)")
        plt.ylabel("Height (m)")
        plt.title("Optimized Trajectory")
        plt.plot(x,y)
        plt.plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
        plt.show()
    else:
        print("Out of range")