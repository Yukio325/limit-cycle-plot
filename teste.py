import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc
from matplotlib import animation
import math
from matplotlib.transforms import Affine2D

class Point():
    def __init__(self, x, y, theta=None):
        self.x = x
        self.y = y
        self.theta = 0

class Ball():
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

    def update(self):
        if self.x + .0427/2 > 1.5 or self.x - .0427/2 < 0:
            self.vx *= -1
        if self.y + .0427/2 > 1.3 or self.y - .0427/2 < 0:
            self.vy *= -1

        self.x += self.vx
        self.y += self.vy

fig = plt.figure(figsize=(10, 7.8))

vel = 0.05

t = Point(1.5, .65) # target
t = Point(1.2, 1.2)
q = Point(1.2, .65) # obstacle
r_v = .075*(1+np.sqrt(2))/2 # avoidance radius
#r = Point(1.0125, .5875, -45) # robot
r = Point(.2, .2, 0)

# Plotting Field
ax = plt.gca()
ax.add_patch(Rectangle((0, 0), 1.5, 1.3, fill=None, alpha=1))
ax.add_patch(Rectangle((0, 0), .75, 1.3, fill=None, alpha=1))
ax.add_patch(Rectangle((0, .45), -.1, .4, fill=None, alpha=1))
ax.add_patch(Rectangle((1.5, .45), .1, .4, fill=None, alpha=1))
ax.add_patch(Rectangle((0, .3), .15, .7, fill=None, alpha=1))
ax.add_patch(Rectangle((1.5, .3), -.15, .7, fill=None, alpha=1))
ax.add_patch(Circle((.75, .65), .2, fill=None, alpha=1))
ax.add_patch(Arc((.15, .65), .1, 0.2, angle=0, theta1=-90, theta2=90, fill=None, alpha=1))
ax.add_patch(Arc((1.5-.15, .65), .1, 0.2, angle=0, theta1=90, theta2=-90, fill=None, alpha=1))

# ax.add_patch(Circle((1.3, .9), (.0427/2), fill=True, color="orange", alpha=1))
#ax.add_patch(Rectangle((q.x-(.075/2), q.y-(.075/2)), .075, .075, fill=True, color="yellow", alpha=1))
#ax.add_patch(Circle((q.x, q.y), r_v, fill=None, color="black", alpha=1))
tra = Affine2D().rotate_deg_around(r.x, r.y, r.theta) + ax.transData
ax.add_patch(Rectangle((r.x-(.075/2), r.y-(.075/2)), .075, .075, fill=True, color="blue", alpha=1, transform=tra))

x = np.linspace(r.x, 1.2, 10)

a = t.y - r.y
b = r.x - t.x
c = t.x*r.y - r.x*t.y

y = (-a*x - c)/b

d = (a*q.x + b*q.y + c)/np.sqrt(a**2 + b**2)

path_x = []
path_y = []

for i in range(200):
    dt = 0.01
    
    x = np.linspace(r.x, 1.2, 10)

    a = t.y - r.y
    b = r.x - t.x
    c = t.x*r.y - r.x*t.y

    y = (-a*x - c)/b

    k = 1 + (a**2/b**2)
    l = -2*0.75 + (2*a*c)/b**2 + (2*a*0.65)/b
    m = 0.75**2 + 0.65**2 - 0.2**2 + c**2/b**2 + (2*c*0.65)/b
    dscr = l**2 - 4*k*m

    if dscr <= 0:
        color="green"
    else:
        color="white"

    dx = r.x - 0.75
    dy = r.y - 0.65

    p = round(5*1/math.sqrt(dx**2 + dy**2))

    # print(f"{p=}")
    re = 0.2

    ddx = (d/abs(d))*dy + dx*(re**2 - dx**2 - dy**2)*25
    ddy = -(d/abs(d))*dx + dy*(re**2 - dx**2 - dy**2)*25

    theta_d = math.atan2(ddy, ddx)

    #print(theta_d)

    path_x.append(r.x)
    path_y.append(r.y)
    # plt.plot(x, y, color="grey", linewidth=1)

    r.x = r.x + dt*ddx
    r.y = r.y + dt*ddy

    plt.plot(x, y, color=color, linewidth=1)

plt.plot(path_x, path_y)

# patch = Circle((1, 1), .0427/2, fill=True, color="orange", alpha=1)

# ball = Ball(1, 1, 0.01, 0.01)

# def init():
#     patch.center = (ball.x, ball.y)
#     ax.add_patch(patch)
#     return patch,

# def animate(i):
#     bx, by = patch.center
#     ball.update()
#     patch.center = (ball.x, ball.y)
#     return patch,

# anim = animation.FuncAnimation(fig, animate, 
#                                init_func=init, 
#                                frames=500, 
#                                interval=20,
#                                blit=True)

# Setting x, y boundary limits
plt.xlim(-0.15, 1.65)
plt.ylim(-0.05, 1.35)
  
# Show plot with grid
plt.show()
