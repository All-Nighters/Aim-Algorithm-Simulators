from math import *
import numpy as np
import matplotlib.pyplot as plt
import time

"""
Precondition: 
input angle in radian, 
distance x in meters,
eject velocity in m/s,
horizontal velocity in m/s,
projectile mass in kg,
gravity constant,
Horizontal cross section area

Postcondition: height y at x meters
"""
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


def solve(upper, lower, step, epoch, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, y, launcher_height):
    u = upper
    l = lower
    s = step

    pt = lower

    for i in range(epoch):
        a = pt * pi/180.0
        f = trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height) - (y)
        
        if (l > u):
            break
        elif (f > 0):
            tmp = pt
            pt -= (u-l) / 2
            u = tmp - s
        elif (f < 0):
            tmp = pt
            pt += (u-l) / 2
            l = tmp + s
        else:
            break

    if (abs(f) > 0.01):
        return -1
    return a


def aim(target_slope, slope_err, vMin, vMax, aMax, xPos, yPos, launcher_height, Vh, Av, Ah, Cv, Ch, m, g, p):

    vTestArr = np.flip(np.arange(vMin, vMax, 0.1))
    vOut = 0
    aOut = 0
    count = 0
    for v in vTestArr:
        count += 1;
        a = solve(aMax, 0, 0.01, 20, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height)
        if a != -1:
            s = slope(a, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch)

            if (s > target_slope - slope_err and s < target_slope + slope_err):
                vOut = v
                aOut = a
                break
            elif (s == -1):
                break

            vOut = v
            aOut = a
        else:
            break

    return vOut, aOut



if __name__ == "__main__":
    a = 30 # launch angle
    xPos = 5 # distance to the goal
    yPos = 0.76835 # height of the goal
    launcher_height = 0.3 # height of the launcher
    v = 10.8 # eject velocity
    vMin = 7.2 # minimum eject velocity
    Vh = 2 # launcher horizontal velocity
    m = 2.8 # mass
    g = 9.81 # gravity
    p = 1.225 # air density
    Av = 0.25 # y direction surface area
    Ah = 0.1 # x direction surface area
    Cv = 0.47 # y direction drag coefficient
    Ch = 0.47 # x direction drag coefficient

    start = time.time()
    v, a = aim(-0.5, 0.1, vMin, v, a, xPos, yPos, launcher_height, Vh, Av, Ah, Cv, Ch, m, g, p)
    end = time.time()
    print(f"Velocity: {v} m/s Angle: {a * 180 / pi} degrees")
    print(f"Elapsed time: {end-start} s")
    print(f"Error: {abs(yPos-trajectory(a, xPos, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))} m")

    Xinit = np.arange(0, 10, 0.1)
    Yinit = []
    for x in Xinit:
        Yinit.append(trajectory(a, x, v, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height))

    plt.title("Trajectory")
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.plot(Xinit, Yinit, color="mediumblue")
    plt.plot(xPos, yPos, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="cyan")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.ylim(0, 6)
    plt.show()