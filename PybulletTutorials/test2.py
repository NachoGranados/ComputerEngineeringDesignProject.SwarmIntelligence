import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")

robot1StartPos = [2, 2, 1]
robot1StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot1Id = p.loadURDF("husky/husky.urdf", robot1StartPos, robot1StartOrientation)

robot2StartPos = [0, 0, 1]
robot2StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot2Id = p.loadURDF("husky/husky.urdf", robot2StartPos, robot2StartOrientation)

#boxId = p.loadURDF("franka_panda/panda.urdf", cubeStartPos, cubeStartOrientation)
#boxId = p.loadURDF("racecar/racecar.urdf", cubeStartPos, cubeStartOrientation)
#boxId = p.loadURDF("quadruped/spirit40.urdf", cubeStartPos, cubeStartOrientation)
#boxId = p.loadURDF("bicycle/bike.urdf", cubeStartPos, cubeStartOrientation)
#boxId = p.loadURDF("husky/husky.urdf", cubeStartPos, cubeStartOrientation)

x = 0.0

for i in range(10000):

    if x < 6.92:

        x = x + 0.01

    else:

        x = 0        
        #p.removeBody(robot1Id)
        #p.removeBody(robot2Id)
        #robot1Id = p.loadURDF("husky/husky.urdf", robot1StartPos, robot1StartOrientation)
        #robot2Id = p.loadURDF("husky/husky.urdf", robot2StartPos, robot2StartOrientation)

    #p.setGravity(-x, -x, 0, robot1Id)
    #p.setGravity(-x, -x, 0)
    p.stepSimulation()
    time.sleep(1./240.)

    """
    p.setJointMotorControl2(bodyUniqueId = robot2Id, jointIndex = 2, controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 5.0, force = 500)
    
    p.setJointMotorControl2(bodyUniqueId = robot2Id, jointIndex = 3, controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 5.0, force = 500)
    
    p.setJointMotorControl2(bodyUniqueId = robot2Id, jointIndex = 4, controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 5.0, force = 500)

    p.setJointMotorControl2(bodyUniqueId = robot2Id, jointIndex = 5, controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 5.0, force = 500)
    """
    
    p.setJointMotorControlArray(bodyUniqueId = robot2Id, jointIndices = [2, 3, 4, 5], controlMode = p.VELOCITY_CONTROL,
                                targetVelocities = [5.0, 5.0, 5.0, 5.0], forces = [500.0, 500.0, 500.0, 500.0])




#cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#print(cubePos, cubeOrn)

p.disconnect()











