import Initial_angle
import Optimize_angle
import VChange_linear
import VChange_binary
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time


if __name__ == "__main__":


    xPos = 4 # distance to the goal
    yPos = 0.76835 # height of the goal
    launcher_height = 0.3 # height of the launcher
    v = 10.8 # eject velocity
    Vh = 0 # launcher horizontal velocity
    m = 0.06 # mass
    g = 9.81 # gravity
    p = 1.225 # air density
    Av = 0.015393804 # y direction surface area
    Ah = 0.0028 # x direction surface area
    Cv = 0.47 # y direction drag coefficient
    Ch = 0.47 # x direction drag coefficient
    m = 2.8

    upper = 30 # angle upperbound
    lower = 0 # angle lowerbound
    step = 0.01
    vMin = 7.2
    vMax = 10.8

    y_error = 0.01

    # desired slope
    target_slope = -0.3
    slope_error = 0.05

    if launcher_height > yPos: # low goal aiming
        u = v
        l = 1
        s = step

        v = l
        V0 = v

        for i in range(20):
            f = Initial_angle.trajectory(0, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height)
            print(v, f, l, u)
            
            if (l > u):
                v = V0
                break
            elif (f > 0):
                tmp = v
                V0 = v
                v -= (u-l) / 2
                u = tmp - s
            elif (f < 0):
                tmp = v
                V0 = v
                v += (u-l) / 2
                l = tmp + s
            else:
                v = V0
                break

        if (abs(f) > 0.01):
            print(f"Can't further increase speed")

            a = Initial_angle.solve(upper, lower, step, 20, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height)
            print(f"Angle: {a * 180 / pi}, Velocity: {v}")
            if (a >= 0):
                Xinit = np.arange(0, 10, 0.1)
                Yinit = []
                for x in Xinit:
                    Yinit.append(Initial_angle.trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

                plt.title("Trajectory")
                plt.xlabel("Distance (m)")
                plt.ylabel("Height (m)")
                plt.ylim(0, 6)
                plt.gca().set_aspect('equal', adjustable='box')
                plt.plot(Xinit, Yinit, color="mediumblue", label="Fastest trajectory")
                plt.plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
                # plt.savefig(fname="optimized")
                plt.show()
                
            else:
                print(f"Out of range: {f}")

        else:
            print(f"Velocity: {v}")
            Xinit = np.arange(0, 10, 0.1)
            Yinit = []
            for x in Xinit:
                Yinit.append(Initial_angle.trajectory(0, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

            plt.title("Trajectory")
            plt.xlabel("Distance (m)")
            plt.ylabel("Height (m)")
            plt.ylim(0, 6)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.plot(Xinit, Yinit, color="mediumblue", label="Fastest trajectory")
            plt.plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
            plt.show()
        











    else: # high goal aiming
        plt.rcParams['figure.figsize'] = [14, 7]
        fig, axs = plt.subplots(1, 3)


        """
        Algorithm 1: guess and optimize
        """
        timer = time.time()
        a = Initial_angle.solve(upper, lower, step, 20, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height)
        aInit = a

        if (a >= 0):
            
            timer = time.time()
            a, v = Optimize_angle.fit(a, upper, vMin, xPos, yPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height, target_slope, slope_error, y_error, 0.01, 0.001)
            print(f"Algorithm 1 elapsed time: {time.time() - timer} s")
            print(f"Algorithm 1 error: {abs(Initial_angle.trajectory(a, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height) - yPos)}")


            # graphing
            Xinit = np.arange(0, 10, 0.1)
            Yinit = []
            for x in Xinit:
                Yinit.append(Initial_angle.trajectory(aInit, x, vMax, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

            axs[0].plot(Xinit, Yinit, color="mediumblue", label="Fastest trajectory")

            X = np.arange(0, 10, 0.1)
            Y = []
            for x in X:
                Y.append(Initial_angle.trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

            axs[0].plot(X, Y, color="lime", label="Optimized trajectory")

            axs[0].plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
            
            axs[0].set_title("Algorithm 1 Computed Trajectory")
            axs[0].set_xlabel("Distance (m)")
            axs[0].set_ylabel("Height (m)")
            axs[0].set_aspect('equal', 'box')
            axs[0].set(xlim=(-1, 10), ylim=(0, 6))
            axs[0].legend()
            # plt.gca().set_aspect('equal', adjustable='box')
            # plt.show()
        







        """
        Algorithm 2: velocity changer (linear)
        """
        timer = time.time()
        v, a = VChange_linear.aim(target_slope, slope_error, vMin, vMax, upper, xPos, yPos, launcher_height, Vh, Av, Ah, Cv, Ch, m, g, p)
        print(f"Algorithm 2 (linear) elapsed time: {time.time() - timer} s")
        print(f"Algorithm 2 error: {abs(Initial_angle.trajectory(a, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height) - yPos)}")


        # graphing
        X = np.arange(0, 10, 0.1)
        Y = []
        for x in X:
            Y.append(Initial_angle.trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

        axs[1].set_title("Algorithm 2 (linear) Computed Trajectory")
        axs[1].set_xlabel("Distance (m)")
        axs[1].set_ylabel("Height (m)")
        axs[1].plot(X, Y, color="mediumblue")
        axs[1].plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
        axs[1].set_aspect('equal', 'box')
        axs[1].set(xlim=(-1, 10), ylim=(0, 6))
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.ylim(0, 6)
        # plt.show()







        """
        Algorithm 3: velocity changer (binary)
        """
        timer = time.time()
        v, a = VChange_binary.aim(target_slope, slope_error, vMin, vMax, upper, xPos, yPos, launcher_height, Vh, Av, Ah, Cv, Ch, m, g, p)
        print(f"Algorithm 2 (binary) elapsed time: {time.time() - timer} s")
        print(f"Algorithm 2 error: {abs(Initial_angle.trajectory(a, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height) - yPos)}")


        # graphing
        X = np.arange(0, 10, 0.1)
        Y = []
        for x in X:
            Y.append(Initial_angle.trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

        axs[2].set_title("Algorithm 2 (binary) Computed Trajectory")
        axs[2].set_xlabel("Distance (m)")
        axs[2].set_ylabel("Height (m)")
        axs[2].plot(X, Y, color="mediumblue")
        axs[2].plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
        axs[2].set_aspect('equal', 'box')
        axs[2].set(xlim=(-1, 10), ylim=(0, 6))
        
        # plt.gca().set_aspect('equal', adjustable='box')
        fig.tight_layout()
        plt.show()