import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#p.setRealTimeSimulation(1)
#p.createCollisionShape(p.GEOM_PLANE)
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
#p.createMultiBody(0, 0)

mass = 1
color = [0, 1, 1, 1]

# Reference ----------------------------------------------------------------------------------------------------

dimensions = [0.5, 0.5, 3]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = [4, 1, 1, 1])

basePosition = [0, 0, 3]
baseOrientation = [0, 0, 0, 1]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Vertical ----------------------------------------------------------------------------------------------------

baseOrientation = [0, 1, 0, 1]

# Length 1 --------------------------------------------------

dimensions = [2, 1.25, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [10.25, 19, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [20, 11, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 4.25 --------------------------------------------------

dimensions = [2, 4.25, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [8.25, 29, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [20.25, 29, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 4.50 --------------------------------------------------

dimensions = [2, 4.50, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [36.25, 20.25, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 6 --------------------------------------------------

dimensions = [2, 6.25, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [20.25, -4, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 9 --------------------------------------------------

dimensions = [2, 9.25, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [32.25, 11, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 10 --------------------------------------------------

dimensions = [2, 10.25, 0.25]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [2, 0, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [0, 20, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [14, 0, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Horizontal ----------------------------------------------------------------------------------------------------

baseOrientation = [0, 0, 1, 1]

# Length 2 --------------------------------------------------

dimensions = [0.25, 2.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [10.25, 25, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [34.25, 16, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 3 --------------------------------------------------

dimensions = [0.25, 3.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [3, 10, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [3, 15, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [17, 10, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 4 --------------------------------------------------

dimensions = [0.25, 4.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [4, 30, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [14.25, 18, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 5 --------------------------------------------------

dimensions = [0.25, 5.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [5.25, 20, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 6 --------------------------------------------------

dimensions = [0.25, 6.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [26.25, 2, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

basePosition = [14.25, 33, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 8 --------------------------------------------------

dimensions = [0.25, 8.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [28.25, 25, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)

# Length 9 --------------------------------------------------

dimensions = [0.25, 9.25, 2]

boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = dimensions)

boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX,
                                  halfExtents = dimensions,
                                  rgbaColor = color)

basePosition = [11, -10, 2]

boxId = p.createMultiBody(mass,
                          boxCollisionId,
                          boxVisualId,
                          basePosition,
                          baseOrientation)




#while(1000):
for i in range(10000):
      
  #keys = p.getKeyboardEvents()
  #print(keys)

  time.sleep(0.01)

p.disconnect()
