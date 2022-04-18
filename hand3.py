import os
import pybullet as p
import time
import pybullet_data
from pybullet_object_models import ycb_objects
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
urdfFlags = p.URDF_USE_SELF_COLLISION

# useFixedBase = True
planeId = p.loadURDF("urdf_assem4_1.urdf", useFixedBase = True)
# quadruped = p.loadURDF("laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase= False)



startPos = [0,0,10]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# add object in the simulator 
# boxId = p.loadURDF("cube.urdf", [0,1,0],useFixedBase= True)
# boxId = p.loadURDF("urdf_screencast.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
flags = p.URDF_USE_INERTIA_FROM_FILE
obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(),'YcbBanana', "model.urdf"),[-0.23, -0.23, 0.2], flags=flags)
p.setGravity(0,0,-9.8)

for i in range (10000):

    p.stepSimulation()
    p.setRealTimeSimulation(1)
    # makes simulation real time 
    time.sleep(1./240.)


def set_torque(self, torque):
    p.setJointMotorControl(bodyIndex = self.bodies [self.bodyIndex], jointIndex= self.jointIndex, controlMode = p.TORQUE_CONTROL, 
    force = torque )
    #, posotionGain = 0.1, velocityGain = 0.1)
p.disconnect()



