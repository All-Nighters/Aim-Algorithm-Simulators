import numpy as np
from math import *
import matplotlib.pyplot as plt

def solve(f, upper, lower, step, epoch):
    u = upper
    l = lower
    s = step

    pt = lower

    for i in range(epoch):
        a = pt * pi/180.0
        # print(pt, l, u, f(a))
        
        if (l > u):
            break
        elif (f(a) > 0):
            tmp = pt
            pt -= (u-l) / 2
            u = tmp - s
        elif (f(a) < 0):
            tmp = pt
            pt += (u-l) / 2
            l = tmp + s
        else:
            break

    if (abs(f(a)) > 0.01):
        print(f"Out of range: {f(a)}")
        return -1
    return a


if __name__ == "__main__":
    g = 9.8

    V = 10.99 # eject velocity
    Vx = 0 # launcher velocity

    upper = 70 # upperbound
    lower = 0 # lowerbound
    step = 0.01

    # angle settings
    Range = np.arange(lower, upper, step)
    launcher_height = 0.4572
    angle = Range * pi/180.0

    # goal position
    x = 1.86
    y = 0.76835
    xPos = x
    yPos = y 

    # time
    t = np.arange(0, 1, 0.01)

    # for graphing use
    f = ((V*np.sin(angle)*x) / (V*np.cos(angle) + Vx)) - ((g * x**2) / (2 * (V * np.cos(angle)+Vx)**2)) + launcher_height - y

    # for calculation use
    func = lambda a : ((V*np.sin(a)*x) / (V*np.cos(a) + Vx)) - ((g * x**2) / (2 * (V * np.cos(a)+Vx)**2)) + launcher_height - y

    a = solve(func, upper, lower, step, 20)
    if (a >= 0):
        print(a, "Radians")
        print(a*180/pi, "Degrees")


        x = (V * np.cos(a) + Vx) * t
        y = V * np.sin(a) * t - 0.5 * g * (t**2) + launcher_height



        fig, ax = plt.subplots(2)
        fig.suptitle(f"Solution: {a*180/pi} degrees")
        ax[0].plot(Range,f)
        ax[1].plot(x,y)
        ax[1].set_ylim([0,1.5])
        ax[0].title.set_text("Solution Curve (find the smalles zero)")
        ax[1].title.set_text("Trajectory")
        ax[0].set_xlabel('angle (degrees)')
        ax[0].set_ylabel('value')
        ax[1].set_xlabel('distance (m)')
        ax[1].set_ylabel('height (m)')
        fig.tight_layout()
        # plt.gca().set_aspect('equal', adjustable='box')
        ax[1].plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
        plt.show()