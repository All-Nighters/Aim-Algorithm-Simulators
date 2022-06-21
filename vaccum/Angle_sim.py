import Initial_angle
import Optimize_angle
import matplotlib.pyplot as plt
import numpy as np
from math import *


def sim(V=10.99, Vx=0, x=2):
    # fundamental constants
    g = 9.8

    # V = 10.99 # eject velocity
    # Vx = 0 # launcher velocity

    launcher_height = 0.4572

    # angle solving bounds
    upper = 70
    lower = 0
    step = 0.01

    # goal position
    # x = 2
    y = 0.76835

    # acceptable range of height error
    y_error = 0.01

    xPos = x
    yPos = y

    target_slope = -0.5 # desired slope
    slope_error = 0.1 # acceptable range of slope error


    Range = np.arange(lower, upper, step)
    t = np.arange(0, 1, 0.01)
    angle = Range * pi/180.0

    # for graphing use
    f = ((V*np.sin(angle)*x) / (V*np.cos(angle) + Vx)) - ((g * x**2) / (2 * (V * np.cos(angle)+Vx)**2)) + launcher_height - y

    # for calculation use
    func = lambda a : ((V*np.sin(a)*x) / (V*np.cos(a) + Vx)) - ((g * x**2) / (2 * (V * np.cos(a)+Vx)**2)) + launcher_height - y

    a = Initial_angle.solve(func, upper, lower, step, 20)

    if (a >= 0):
        x = (V * np.cos(a) + Vx) * t
        y = V * np.sin(a) * t - 0.5 * g * (t**2) + launcher_height

        fig, ax = plt.subplots(2)
        fig.suptitle(f"Solution: {a*180/pi} degrees, Velocity: {V} m/s", fontsize=10)
        aPrev = a
        Vprev = V
        
        ax[0].plot(Range,f)
        ax[1].plot(x,y, label="Initial trajectory")
        ax[1].set_ylim([0,1.5])
        ax[0].title.set_text("Solution Curve (find the smalles zero)")
        ax[1].title.set_text("Trajectory")
        ax[0].set_xlabel('angle (degrees)')
        ax[0].set_ylabel('value')
        ax[1].set_xlabel('Distance (m)')
        ax[1].set_ylabel('Height (m)')
        # plt.gca().set_aspect('equal', adjustable='box')
        ax[1].plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")


        x = xPos
        y = yPos

        a0, V = Optimize_angle.fit(a, V, x, y, Vx, launcher_height, target_slope, slope_error, y_error, 0.01, 0.1)
        t = np.arange(0,1,0.01)
        x = (V * np.cos(a0) + Vx) * t
        y = V * np.sin(a0) * t - 0.5 * g * (t**2) + launcher_height
        plt.ylim(0, 5)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.suptitle(f"Angle: {a0 * 180 / pi} degrees, Velocity: {V} m/s", fontsize=10)
        ax[1].plot(x,y,label="Optimized trajectory")
        ax[1].legend()
        fig.tight_layout()
        plt.savefig(fname='optimized')
        plt.show()
        return True

    else:
        print("Out of range")
        return False

if __name__ == "__main__":
    V = 10.99 # eject velocity
    Vx = 0 # launcher velocity
    x = 2 # goal distance
    sim(V,Vx,x)
