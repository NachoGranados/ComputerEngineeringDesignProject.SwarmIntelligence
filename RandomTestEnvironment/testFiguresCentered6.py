import pybullet as p
import time
import pybullet_data
from constants import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, GRAVITY)

def createWall(dimensions, color, basePosition, baseOrientation):
      
      boxCollisionId = p.createCollisionShape(shapeType = p.GEOM_BOX, halfExtents = dimensions)

      boxVisualId = p.createVisualShape(shapeType = p.GEOM_BOX, halfExtents = dimensions, rgbaColor = color)

      p.createMultiBody(MASS, boxCollisionId, boxVisualId, basePosition, baseOrientation)

# Plane ----------------------------------------------------------------------------------------------------

createWall(PLANE_DIMENSIONS, PLANE_COLOR, PLANE_POSITION, PLANE_ORIENTATION)

# Vertical ----------------------------------------------------------------------------------------------------

# Length 2 --------------------------------------------------

dimensions = [2, 2.25, 0.25]

basePosition = [-9.75, 6, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

# Length 4 --------------------------------------------------

dimensions = [2, 4.25, 0.25]

basePosition = [-11.75, 18, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

basePosition = [1, 18, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

basePosition = [-14, -18, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

basePosition = [12, -9, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

# Length 7 --------------------------------------------------

dimensions = [2, 7.25, 0.25]

basePosition = [4, -15, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

basePosition = [13, 15, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

# Length 9 --------------------------------------------------

dimensions = [2, 9.25, 0.25]

basePosition = [-6, -12.50, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

# Length 22 --------------------------------------------------

dimensions = [2, 22.25, 0.25]

basePosition = [-22, 0, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

basePosition = [22, 0, 2]

createWall(dimensions, WALL_COLOR, basePosition, VERTICAL_ORIENTATION)

# Horizontal ----------------------------------------------------------------------------------------------------

# Length 1 --------------------------------------------------

dimensions = [0.25, 1.25, 2]

basePosition = [-13, -14, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 2 --------------------------------------------------

dimensions = [0.25, 2.25, 2]

basePosition = [-9.75, 14, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

basePosition = [-4, -3, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 3 --------------------------------------------------

dimensions = [0.25, 3.25, 2]

basePosition = [10, 8, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 4 --------------------------------------------------

dimensions = [0.25, 4.25, 2]

basePosition = [-5.75, 4, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

basePosition = [-18, 0, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

basePosition = [-18, -7, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 5 --------------------------------------------------

dimensions = [0.25, 5.25, 2]

basePosition = [17, -13, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 6 --------------------------------------------------

dimensions = [0.25, 6.25, 2]

basePosition = [-15.75, 8, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

basePosition = [16, 1, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

# Length 22 --------------------------------------------------

dimensions = [0.25, 22.25, 2]

basePosition = [0, 22, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

basePosition = [0, -22, 2]

createWall(dimensions, WALL_COLOR, basePosition, HORIZONTAL_ORIENTATION)

#while(1000):
for i in range(10000):
      
  #keys = p.getKeyboardEvents()
  #print(keys)

  time.sleep(0.01)

p.disconnect()
