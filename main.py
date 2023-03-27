import vpython as vp
import numpy as math


# parameter
floor_depth = 0.1
robot_arm1_length = 2
robot_arm2_length = 2
original_point = vp.vec(0, 0, 0)
#initial coordinate
x = robot_arm2_length
z = robot_arm1_length
y = 0
x_n = 1
y_n = 1.6
z_n = 2

floor = vp.box(pos=vp.vec(0, floor_depth/2, 0), size=vp.vec(5, floor_depth, 5), color=vp.color.black)
joint01 = vp.sphere(pos=original_point, radius=0.2, color=vp.color.blue)
#joint type is sphere
joint02 = vp.sphere(pos=vp.vec(0, robot_arm1_length, 0), radius=0.2, color=vp.color.blue)
#joint type is ratation
joint03 = vp.sphere(pos=vp.vec(x, z, y), radius=0.16, color=vp.color.blue)

robot_arm1 = vp.cylinder(pos=original_point, radius=0.08, axis=vp.vec(0, 1, 0), length=robot_arm1_length,
                         color=vp.color.cyan)
robot_arm2 = vp.cylinder(pos=vp.vec(0, robot_arm1_length, 0), radius=0.08, axis=vp.vec(1, 0, 0),
                         length=robot_arm2_length, color=vp.color.cyan)
x_slider = vp.slider(min=-5, max=5, step=0.1, value=x, bind=lambda: move())
y_slider = vp.slider(min=-5, max=5, step=0.1, value=y, bind=lambda: move())
z_slider = vp.slider(min=-5, max=5, step=0.1, value=z, bind=lambda: move())
psi_0 = 0
psi_n = 0
phi_0 = 0
phi_n = 0
theta_n = 0

def theta_cal():
    global x, y, z, x_n, y_n, z_n, theta_n
    cos_theta = x/math.sqrt(x**2+y**2)
    sin_theta = y/math.sqrt(x**2+y**2)
    cos_theta1 = x_n/math.sqrt(x_n**2+y_n**2)
    sin_theta1 = y_n/math.sqrt(x_n**2+y_n**2)
    theta_n = math.arccos(cos_theta1)

def psi_cal():
    global psi_0, psi_n, x, y, z, x_n, y_n, z_n
    cos_phi = (robot_arm1_length**2+robot_arm2_length**2-(x**2+y**2+z**2))/(2*robot_arm1_length*robot_arm2_length)
    psi_0 = math.arccos(cos_phi)
    cos_phi1 = (robot_arm1_length**2+robot_arm2_length**2-(x_n**2+y_n**2+z_n**2))/(2*robot_arm1_length*robot_arm2_length)
    psi_n = math.arccos(cos_phi1)


def phi_cal():
    global psi_0, psi_n, robot_arm2_length, x, y, z, x_n, y_n, z_n, phi_0, phi_n
    sin_phi = z/math.sqrt(x**2+y**2+z**2)
    phi = math.arcsin(sin_phi)
    sin_phi1 = z_n / math.sqrt(x_n**2+y_n**2+z_n**2)
    phi1 = math.arcsin(sin_phi1)
    sin_phi2 = robot_arm2_length*math.sin(psi_0)/math.sqrt(x**2+y**2+z**2)
    phi2 = math.arcsin(sin_phi2)
    sin_phi3 = robot_arm2_length * math.sin(psi_n) / math.sqrt(x_n**2+y_n**2+z_n**2)
    phi3 = math.arcsin(sin_phi3)
    phi_0 = phi + phi2
    phi_n = phi1 + phi3


def move():
    global theta_n, phi_n, psi_n, robot_arm1_length, robot_arm1, joint02, robot_arm2_length, robot_arm2, joint03, x_n, y_n, z_n
    x_n = x_slider.value
    y_n = y_slider.value
    z_n = z_slider.value
    theta_cal()
    psi_cal()
    phi_cal()
    axis1 = vp.vec(robot_arm1_length*math.cos(phi_n)*math.cos(theta_n),
                  robot_arm1_length * math.sin(phi_n),
                  robot_arm1_length*math.cos(phi_n)*math.sin(theta_n))
    robot_arm1.axis = axis1
    joint02.pos = axis1
    robot_arm2.pos = axis1
    axis2 = vp.vec(robot_arm2_length*math.sin(psi_n+phi_n-math.pi/4)*math.cos(theta_n),
                   robot_arm2_length*math.cos(psi_n+phi_n-math.pi/4),
                   robot_arm2_length*math.sin(psi_n+phi_n-math.pi/4)*math.sin(theta_n))
    robot_arm2.axis = axis2
    joint03.pos = vp.vec(x_n, z_n, y_n)


while True:
    vp.rate(60)