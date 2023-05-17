import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
#p.setGravity(0,0,-9.8)
p.setGravity(0,0,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
planeId = p.loadURDF("plane.urdf")

robot1StartPos = [0, 0, 1]
robot1StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot1Id = p.loadURDF("block.urdf", robot1StartPos, robot1StartOrientation)


"""
robot2StartPos = [0, 2, 0]
robot2StartOrientation = p.getQuaternionFromEuler([0, 0, -1])
robot2Id = p.loadURDF("husky/husky.urdf", robot2StartPos, robot2StartOrientation)

robot3StartPos = [2, 0, 0]
robot3StartOrientation = p.getQuaternionFromEuler([0, 0, 1])
robot3Id = p.loadURDF("husky/husky.urdf", robot3StartPos, robot3StartOrientation)

robot4StartPos = [0, -2, 0]
robot4StartOrientation = p.getQuaternionFromEuler([0, 0, -1])
robot4Id = p.loadURDF("husky/husky.urdf", robot4StartPos, robot4StartOrientation)

robot5StartPos = [-2, 0, 0]
robot5StartOrientation = p.getQuaternionFromEuler([0, 0, 1])
robot5Id = p.loadURDF("husky/husky.urdf", robot5StartPos, robot5StartOrientation)


robotsIdList = [robot1Id, robot2Id, robot4Id, robot4Id]
"""

#x = 0.0
#y = 0.0

for i in range(10000):

        p.stepSimulation()
        time.sleep(1./240.)

p.disconnect()
