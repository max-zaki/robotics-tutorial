import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def forward_kinematics(theta1, theta2, L1, L2):
    # compute (x, y) of the elbow and the end-effector
    elbow_x = L1 * np.cos(theta1)
    elbow_y = L1 * np.sin(theta1)
    ee_x = elbow_x + L2 * np.cos(theta1 + theta2)
    ee_y = elbow_y + L2 * np.sin(theta1 + theta2)

    # return both points
    return ((elbow_x, elbow_y), (ee_x, ee_y))

def plot_arm(ax, theta1, theta2, L1=1.0, L2=1.0):
    # get the joint positions from forward kinematics
    (elbow_x, elbow_y), (ee_x, ee_y) = forward_kinematics(theta1, theta2, L1, L2)

    # plot: origin -> elbow -> tip as a connected line
    # mark each joint as a dot
    x = [0, elbow_x, ee_x]
    y = [0, elbow_y, ee_y]
    ax.plot(x, y, '.-')

    # set equal axis scaling
    ax.axis('equal')
    

if __name__ == '__main__':
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)

    ax_t1 = plt.axes([0.2, 0.1, 0.6, 0.03])
    ax_t2 = plt.axes([0.2, 0.05, 0.6, 0.03])
    slider_t1 = Slider(ax_t1, 'θ1', -np.pi, np.pi, valinit=np.pi/4)
    slider_t2 = Slider(ax_t2, 'θ2', -np.pi, np.pi, valinit=np.pi/4)

    def update(val):
        ax.cla()
        # call plot_arm here, passing slider values and ax
        # hint: plot_arm needs to draw into ax, not a new figure
        plot_arm(ax, slider_t1.val, slider_t2.val)
        fig.canvas.draw_idle()
    
    slider_t1.on_changed(update)
    slider_t2.on_changed(update)
    
    plt.show()

