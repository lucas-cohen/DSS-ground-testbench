import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from control.default_dynamics import circle_around_point

fig, ax = plt.subplots()

xdata1, ydata1 = [], []
xdata2, ydata2 = [], []

ln1,   = plt.plot([], [], 'k-', alpha=0.5)
mark1, = plt.plot([], [], 'ro', label="Robot 1")
vec1  = plt.quiver([],[],[],[], width=5e-3)

ln2,   = plt.plot([], [], 'k-', alpha=0.5)
mark2, = plt.plot([], [], 'bo', label="Robot 2")
vec2  = plt.quiver([],[],[],[], width=5e-3)

vecRel = plt.quiver([],[],[],[], width=5e-3)
lnRel,   = plt.plot([], [], 'k:', alpha=0.5)

time_disp = plt.text(-1.45,-1.45,"", fontsize=11) 

presist_on_repeat = False
f1, f2 = 3,7
R1, R2 = 0.7, 0.5

gcd = np.gcd(f1,f2)

def init():
    ax.set_xlim(-1.5,1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.legend()
    ax.set_aspect('equal')
    return ln1,mark1,ln2,mark2,vec1,vec2,vecRel,lnRel,time_disp

def update(frame):
    global xdata1, ydata1
    global xdata2, ydata2
    
    # reset on repeat:
    if frame == 0.0 and not presist_on_repeat:
        xdata1, ydata1 = [], []
        xdata2, ydata2 = [], []

    # Motion particle 1
    pos1x, pos1y, dir1x, dir1y = circle_around_point([0,0], [0,0], frame, R1, (2*np.pi)/f1)
    
    xdata1.append(pos1x)
    ydata1.append(pos1y)
    ln1.set_data(xdata1, ydata1)
    mark1.set_data(pos1x, pos1y)
    
    vec1.set_offsets([pos1x, pos1y])
    vec1.set_UVC(dir1x, dir1y)
    
    # Motion particle 2
    pos2x, pos2y, dir2x, dir2y = circle_around_point([pos1x, pos1y], [dir1x, dir1y], frame, R2, (2*np.pi)/f2)

    
    xdata2.append(pos2x)
    ydata2.append(pos2y)

    ln2.set_data(xdata2, ydata2)
    mark2.set_data(pos2x,pos2y)
    
    vec2.set_offsets([pos2x, pos2y])
    vec2.set_UVC(dir2x, dir2y)
    
    lnRel.set_data([pos1x, pos2x], [pos1y, pos2y])
    
    relx, rely = pos1x - pos2x, pos1y - pos2y
    vecRel.set_offsets([pos2x, pos2y])
    vecRel.set_UVC(relx, rely)
    
    
    time_disp.set_text(f"t = {round(frame,2)}")
    

    return ln1,mark1,ln2,mark2,vec1,vec2,vecRel,lnRel,time_disp

ani = FuncAnimation(fig, update, frames=np.linspace(0, gcd*2*np.pi, gcd*256),
                    init_func=init, blit=True, interval=25, repeat_delay=0)
plt.show()