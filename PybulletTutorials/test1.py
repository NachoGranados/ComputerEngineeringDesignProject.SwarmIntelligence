import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

# load assets
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase = True)
obj_of_focus = targid

jointid = 4
jlower = p.getJointInfo(targid, jointid)[8]
jupper = p.getJointInfo(targid, jointid)[9]

"""
for i in range(p.getNumJoints(targid)):
    print(p.getJointInfo(targid, i))
"""

maxForce = 500

for step in range(100000):

    joint_tow_targ = np.random.uniform(jlower, jupper)
    joint_four_targ = np.random.uniform(jlower, jupper)

    #p.setJointMotorControlArray(targid, [2 , 4], p.POSITION_CONTROL,
    #                            targetPositions = [joint_tow_targ, joint_four_targ])

    p.setJointMotorControl2(bodyUniqueId = targid, jointIndex = 4, controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 5.0, force = maxForce)
    
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 0, cameraPitch = -40,
                                 cameraTargetPosition = focus_position)
    p.stepSimulation()
    time.sleep(1./240.)


















