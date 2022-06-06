import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()

xdata1, ydata1 = [], []
xdata2, ydata2 = [], []

ln1,   = plt.plot([], [], 'k-', alpha=0.5)
mark1, =  plt.plot([], [], 'ro')

ln2,   = plt.plot([], [], 'k-', alpha=0.5)
mark2, =  plt.plot([], [], 'bo')

presist_on_repeat = True
f1, f2 = 1,5

gcd = np.gcd(f1,f2)
print(gcd)

def init():
    ax.set_xlim(-1.5,1.5)
    ax.set_ylim(-1.5, 1.5)
    return ln1,mark1,ln2,mark2,

def update(frame):
    global xdata1, ydata1
    global xdata2, ydata2
    
    # reset on repeat:
    if frame == 0.0 and not presist_on_repeat:
        xdata1, ydata1 = [], []
        xdata2, ydata2 = [], []

    # Motion particle 1
    pos1x, pos1y = np.cos(frame), np.sin(frame)
    
    xdata1.append(pos1x)
    ydata1.append(pos1y)
    ln1.set_data(xdata1, ydata1)
    mark1.set_data(pos1x, pos1y)
    
    
    # Motion particle 2
    pos2x, pos2y = pos1x + 0.25*np.cos(4*frame), pos1y + 0.25*np.sin(4*frame)
    
    xdata2.append(pos2x)
    ydata2.append(pos2y)

    ln2.set_data(xdata2, ydata2)
    mark2.set_data(pos2x,pos2y)
    
    return ln1,mark1,ln2,mark2

ani = FuncAnimation(fig, update, frames=np.linspace(0, gcd*2*np.pi, 128),
                    init_func=init, blit=True, interval=10, repeat_delay=0)
plt.show()