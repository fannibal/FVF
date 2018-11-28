# -*- coding: utf-8 -*-

from roblib import *

def f(x, u):
    """Evolution function : return xdot = f(x,u)"""
    theta = (x.flatten())[2]
    xdot = np.array([[cos(theta)], [sin(theta)], [1*u]]) #TODO
    return xdot

def draw_field(xmin,xmax,ymin,ymax):
    """Draw a (VX, VY) vector field between xmin, xmax and ymin, ymax."""
    Mx    = arange(xmin,xmax,1)
    My    = arange(ymin,ymax,1)
    X1,X2 = meshgrid(Mx,My)
    a ,b = 1,-X2
    VX    = a #TODO
    VY    = b+0*X1 #TODO
    R = sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def draw_target(xmin, xmax):
    """Draw a red line that represents the goal between xmin, xmax and ymin, ymax."""
    s = arange(xmin, xmax, 0.01)
    w = array([s, 0*s])
    plot2D(w, "red", 3)
    return

def control(x):
    """return u, the command"""
    #TODO 
    xf = x.flatten()
    a ,b = 1,-xf[1]
    psi = arctan2(b,a)
    theta = xf[2]
    theta = arctan2(sin(theta),cos(theta))
    e = sawtooth(psi-theta)
    u = 1*e
    return u


# Initial state
x = array([[0], [-3], [1]])
u = 0

dt = 0.05
T = 15

xmin, xmax, ymin, ymax = 0, 20, -5, 5
ax = init_figure(xmin, xmax, ymin, ymax)


for t in arange(0, T, dt):
    clear(ax)
    draw_tank(x,'darkblue',0.3)
    draw_field(xmin,xmax,ymin,ymax)
    draw_target(xmin,xmax)
    u = control(x)
    x = x+dt*f(x,u) # Euler integration
pause(1)
