import Initial_angle
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time

def trajectory(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch, launcher_height):

    n = a
    Av = Av_temp * cos(n) + Ah_temp * sin(n)
    Ah = Ah_temp * cos(n) + Av_temp * sin(n)
    # Cv = Cv_temp
    # Ch = Ch_temp

    Kv = 0.5 * p * Av * Cv
    Kh = 0.5 * p * Ah * Ch
    t = (m*(e**(Kh*x/m)-1)) / (Kh*(v*cos(n)+Vh))

    b1 = sqrt(m/(g*Kv)) * atan(v*sin(n)*sqrt(Kv/(m*g)))

    y = 0

    if (t > b1):
        y = -m/Kv * log(cosh(t*sqrt(g*Kv/m) - atan(v*sin(n)*sqrt(Kv / (m*g))))) + m / (2*Kv) * log(Kv*v**2*sin(n)**2 / (m*g) + 1) + launcher_height
    else:
        y = m/Kv * log(cos(t*sqrt(g*Kv/m) - atan(v*sin(n)*sqrt(Kv / (m*g))))) + m / (2*Kv) * log(Kv*v**2*sin(n)**2 / (m*g) + 1) + launcher_height
    return y

def slope(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch):


    n = a

    Av = Av_temp * cos(n) + Ah_temp * sin(n)
    Ah = Ah_temp * cos(n) + Av_temp * sin(n)

    Kv = 0.5 * p * Av * Cv
    Kh = 0.5 * p * Ah * Ch
    t = (m*(e**(Kh*x/m)-1)) / (Kh*(v*cos(n)+Vh))

    b1 = sqrt(m/(g*Kv)) * atan(v*sin(n)*sqrt(Kv/(m*g)))

    dtdx = e**(Kh*x/m) / (cos(n)*v+Vh)

    if (t > b1):
        dydt = - (sqrt(g*Kv/m) * m * sinh(sqrt(g*Kv/m) * t - atan(sqrt(Kv / (m*g)) * sin(n) * v))) / (Kv * cosh(sqrt(g*Kv/m) * t - atan(sqrt(Kv / (m*g)) * sin(n) * v)))
        
    else:
        dydt = - (sqrt(g*Kv/m) * m * sin(sqrt(g*Kv/m) * t - atan(sqrt(Kv / (m*g)) * sin(n) * v))) / (Kv * cos(sqrt(g*Kv/m) * t - atan(sqrt(Kv / (m*g)) * sin(n) * v)))
        
    return dydt * dtdx

def fit(a0, aMax, vMin, x, y, V0, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch, launcher_height, target_slope, err, y_err, v_step, a_step):
    count = 0
    a = a0
    v = V0
    func = trajectory(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch, launcher_height)
    s = slope(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch)

    while ((s < target_slope - err or s > target_slope + err)):
        
        while (func > y - y_err):
            count += 1  
            v -= v_step
            func = trajectory(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch, launcher_height)
            s = slope(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch)
            # print("Decrease speed")   
            # print("Angle: ", a)  
            # print("Velocity: ", v)
            # print(f"y: {func}, {y - y_err}")    
            # print("Count: ", count)
            # print() 
            if (v < vMin):
                # print("Out!")   
                # print("Angle: ", a)  
                # print("Velocity: ", v)
                # print(f"Slope: {s}")    
                # print("Count: ", count)
                # print()   
                return a0, V0
            
            if (s > target_slope - err and s < target_slope + err and abs(func - y) < y_err):
                a0 = a
                V0 = v
                break
        while (func < y + y_err):
            count += 1  
            a += a_step
            func = trajectory(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch, launcher_height)
            s = slope(a, x, v, Vh, m, g, p, Av_temp, Ah_temp, Cv, Ch)
            # print("Increase angle")   
            # print("Angle: ", a)  
            # print("Velocity: ", v)
            # print(f"y: {func}, {y + y_err}")   
            # print("Count: ", count)
            # print()   
            if (a > (aMax*pi/180)):
                # print("Out!")   
                # print("Angle: ", a)  
                # print("Velocity: ", v)
                # print(f"Slope: {s}")    
                # print("Count: ", count)
                # print()   
                return a0, V0
            if (s > target_slope - err and s < target_slope + err and abs(func - y) < y_err):
                a0 = a
                V0 = v
                break
        a0 = a
        V0 = v

    # print("Angle: ", a)  
    # print("Velocity: ", v)  
    # print(f"Slope: {s}")    
    # print("Count: ", count)
    # print(y_err)
    # print(func - y)
    return a, v 


if __name__ == "__main__":
    xPos = 2 # distance to the goal
    yPos = 0.76835 # height of the goal
    launcher_height = 0.3 # height of the launcher
    v = 10.8 # eject velocity
    Vh = 2 # launcher horizontal velocity
    m = 2.8 # mass
    g = 9.81 # gravity
    p = 1.225 # air density
    Av = 0.25 # y direction surface area
    Ah = 0.1 # x direction surface area
    Cv = 0.47 # y direction drag coefficient
    Ch = 0.47 # x direction drag coefficient
    m = 2.8

    upper = 57 # upperbound
    lower = 0 # lowerbound
    step = 0.01

    y_error = 0.01

    # desired slope
    target_slope = -0.5
    slope_error = 0.1

    a = Initial_angle.solve(upper, lower, step, 20, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height)
    print(a*180/pi)

    if (a >= 0):
        a, v = fit(a, upper, xPos, yPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height, target_slope, slope_error, y_error, 0.01, 0.01)
        x = np.arange(0,10,0.1)
        y = []

        for i in x:
            y.append(trajectory(a, i, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

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
