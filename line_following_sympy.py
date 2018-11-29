from sympy import *
from sympy.diffgeom import *
from sympy.matrices import matrix_multiply_elementwise as mmul_elementwise
import numpy as np
import matplotlib.pyplot as plt
from roblib import sawtooth

def f(X,u):
    """Evolution function : return xdot = f(X,u)"""
    x, y, theta, theta_t, x_t, y_t = X
    Lt = 1.5
    return Matrix([cos(theta), sin(theta), 1*u, Lt*sin(theta-theta_t),cos(theta),sin(theta)]) #TODO

                  
def g(X):
    """Observation function : return y = g(X)"""
    x, y, theta, theta_t, x_t, y_t = X
    return theta_t #TODO

def draw_field(xmin,xmax,ymin,ymax):
    """Draw a (VX, VY) vector field between xmin, xmax and ymin, ymax."""
    Mx    = np.arange(xmin,xmax,1)
    My    = np.arange(ymin,ymax,1)
    X1,X2 = np.meshgrid(Mx,My)
    a = X2
    b = -(0.01*X1**2 - 1)*X2 - X1
    VX    = a #TODO
    VY    = b #TODO
    R = np.sqrt(VX**2+VY**2)
    ax.quiver(Mx,My,VX/R,VY/R)
    return

def draw_target(xmin,xmax):
    """Draw a red line that represents the goal between xmin, xmax and ymin, ymax."""
    s = np.arange(xmin,xmax,0.01)
    w = np.array([s,np.sin(s)])
    ax.plot(w[0, :], w[1, :], "red", linewidth = 3)
    return

def move_motif(M,x,y,theta):
    """Move and rotate a motif M wrt (x, y, theta). Return the new motif at the right place."""
    M1 = ones(1, M.shape[1])
    M2 = M.row_insert(M.shape[0], M1)
    R = Matrix([[cos(theta),-sin(theta),x], [sin(theta),cos(theta),y]])
    return(R * M2)

def draw_tank(x,col='darkblue',r=1):
    """Draw a tank motif at the place (x[0], x[1]) rotated by x[2] in color col and with the scale r."""
    M = r*Matrix([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M = move_motif(M, x[0], x[1], x[2])
    M = M.evalf().tolist()
    ax.plot(M[0], M[1], col, linewidth = 2)

def control(X):
    """return u, the command"""
    err = f_e(*X)
    derr = f_de(*X)
    d2err = f_d2e(*X)
    #TODO
    diff_eq = err + 2*derr + d2err #TODO
    command = solve(diff_eq, u)[0] # We solve the differential equation in order to find u, the command.
    return command


M = Manifold('M', 6) # 3 state variables
P = Patch('P', M)
coord = CoordSystem("coord", P, ["x", "y", "theta","theta_t", "x_t", "y_t"])
x, y, theta, theta_t, x_t, y_t = coord.coord_functions()
X = Matrix([x, y, theta,theta_t, x_t, y_t]) # state vector X
u = Symbol('u') # command u
f_vf = sum(mmul_elementwise(f(X,u), Matrix(coord.base_vectors()))) # vector field of the function f

# measure
y_m = g(X)

# (a, b) is the vector field we want to follow
#TODO
a = y
b = -(0.01*x**2 - 1)*y - x
# So b_x is the desired angle
b_x = atan2(b, a)

# Our error is e
e = y_m - b_x
de = LieDerivative(f_vf, e) # Derivative of the error is a Lie derivative
d2e = LieDerivative(f_vf, de)
#TODO

# We create the functions that gives the error and its derivatives wrt the state
f_e = lambdify((x, y, theta, theta_t, x_t, y_t), e)
f_de = lambdify((x, y, theta, theta_t, x_t, y_t), de)
f_d2e = lambdify((x, y, theta, theta_t, x_t, y_t), d2e)
#TODO

# Initial state
Lt = 1.5
state = [1,-3,.5*np.pi,.25*np.pi, 0, 0]
state[4] = state[0]-Lt*cos(state[3])
state[5] = state[1]-Lt*sin(state[3])
dt = 0.05
T = 15

# We create the function that gives the state at t+dt
euler = lambdify((x, y, theta, theta_t, x_t, y_t, u), X+dt*f(X,u))

# init_figure
xmin, xmax, ymin, ymax = -15, 15, -15, 15
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
ax.xmin=xmin
ax.xmax=xmax
ax.ymin=ymin
ax.ymax=ymax

for t in np.arange(0,T,dt):
    # drawing
    plt.pause(.0001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)
    draw_field(xmin,xmax,ymin,ymax)
    #draw_target(-xmin,xmax)
    draw_tank(state,'darkblue',0.3)
    draw_tank([state[4],state[5],state[3]],'red',0.25)

    # state evolution
    command = control(state)
    state = euler(*state, command) # Euler integration
    state = [float(v) for v in state.flatten()]
    state[2] = sawtooth(state[2])
    state[3] = sawtooth(state[3])
    state[4] = state[0]-Lt*np.cos(state[3])
    state[5] = state[1]-Lt*np.sin(state[3])
plt.pause(1)
