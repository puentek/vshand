import os
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
import numpy as np
import math 
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# load urdf model 
planeId = p.loadURDF("urdf_assem4_18.urdf", useFixedBase = True)
# force parameter 
idff = p.addUserDebugParameter("Test force",-2.96, 2.96,0) 
# add table environment 
tableUid= p.loadURDF(os.path.join("table/table.urdf"),basePosition=[0.5,0,-0.75])
#add tray 
trayUid = p.loadURDF(os.path.join("tray/traybox.urdf"),basePosition=[0.65,0,0])
p.setGravity(0,0,-10)
#banana:
flags = p.URDF_USE_INERTIA_FROM_FILE
obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbBanana', "model.urdf"),[0.21, -0.10, 0.1], flags=flags)
obj_id2 = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbPear', "model.urdf"),[0.65,0,0], flags=flags)
obj_id3 = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbTennisBall', "model.urdf"),[0.65,0,0], flags=flags)
obj_id4 = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbMediumClamp', "model.urdf"),[0.65,0,0], flags=flags)
obj_id5 = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbPowerDrill', "model.urdf"),[0.65,0,0], flags=flags)
p.resetDebugVisualizerCamera(cameraDistance=1.5,cameraYaw=0,cameraPitch=-40,cameraTargetPosition=[0.50,-0.35,0.2])

state_durations = [1,1,1,1]
control_dt= 1/240
p.setTimeStep= control_dt
state_t = 0
current_state=0

while True:
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    # apply force to last link:
    
    if current_state ==0:
        p.setJointMotorControl2(planeId, 0, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(planeId, 1, 
                        p.POSITION_CONTROL,math.pi/4.)
        p.setJointMotorControl2(planeId, 2, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(planeId, 3, 
                        p.POSITION_CONTROL,-math.pi/2.)
        p.setJointMotorControl2(planeId, 4, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(planeId, 5, 
                        p.POSITION_CONTROL,3*math.pi/4)
        
        # reset joint state:
        p.resetJointState = (planeId, 1,-0.5,0)

        jointFrictionForces = 0
        for joint in range(p.getNumJoints(planeId)):
            p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=5.2, force = 2.5)
            # pend = [0,1.1,0.1]
            # Fend = [p.readUserDebugParameter(idff),0,0]
            # p.applyExternalForce(planeId,1,Fend, pend, p.WORLD_FRAME)
            # p.addUserDebugLine(pend,np.array(pend)+100*np.array(Fend), lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=2)
            # p.stepSimulation()
            # time.sleep(1*1e-3)
    
    if current_state == 1:
        p.setJointMotorControl2(planeId, 1, 
                        p.POSITION_CONTROL,math.pi/4.+.15)
        p.setJointMotorControl2(planeId, 3, 
                        p.POSITION_CONTROL,-math.pi/2.+.15)
        jointFrictionForces = 0
        for joint in range(p.getNumJoints(planeId)):
            p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=5.2, force = 2.5)
        
        # pend = [0,1.1,0.1]
        # Fend = [p.readUserDebugParameter(idff),0,0]
        # p.applyExternalForce(planeId,1,Fend, pend, p.WORLD_FRAME)
        # p.addUserDebugLine(pend,np.array(pend)+100*np.array(Fend), lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=2)
        # p.stepSimulation()
        # time.sleep(1*1e-3)

    if current_state == 2:
        p.setJointMotorControl2(planeId, 1, 
                        p.POSITION_CONTROL,math.pi/4.-1)
        p.setJointMotorControl2(planeId, 3, 
                        p.POSITION_CONTROL,-math.pi/2.-1)
        jointFrictionForces = 0
        for joint in range(p.getNumJoints(planeId)):
            p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=5.2, force = 2.96)

        # pend = [0,1.1,0.1]
        # Fend = [p.readUserDebugParameter(idff),0,0]
        # p.applyExternalForce(planeId,1,Fend, pend, p.WORLD_FRAME)
        # p.addUserDebugLine(pend,np.array(pend)+100*np.array(Fend), lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=2)
        # p.stepSimulation()
        # time.sleep(1*1e-3)
    
    if state_t >state_durations[current_state]:
        current_state += 1
        if current_state >= len(state_durations):
            current_state = 0
        state_t = 0
        jointFrictionForces = 0
        for joint in range(p.getNumJoints(planeId)):
            p.setJointMotorControl2(planeId,joint,p.VELOCITY_CONTROL, targetVelocity=0, force = 0)

    p.stepSimulation()


# pend = [0,1.1,0.1]
# Fend = [p.readUserDebugParameter(idff),0,0]
# p.applyExternalForce(planeId,1,Fend, pend, p.WORLD_FRAME)
# p.addUserDebugLine(pend,np.array(pend)+100*np.array(Fend), lineColorRGB=[1,0,0],lifeTime=0.1,lineWidth=2)
# p.stepSimulation()
# time.sleep(SIM_TIMESTEP*1e-3)