import mujoco
import mujoco_viewer
import numpy as np
import time
import random


model = mujoco.MjModel.from_xml_path("creature.xml")
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

#Get array of actuators
actuators = model.nu

# print(data.ctrl)

# print(actuators)

# for i in range(400):
#     mujoco.mj_step(model, data)

max_dist = 0

# print(data.xpos)

for i in range(3000):
    if viewer.is_alive:
        # Currently clock-based movement
        if i % 300 == 0:
            data.ctrl[:actuators] = [5]
            
        elif i % 300 == 150:
            data.ctrl[:actuators] = [-5]
        
        mujoco.mj_step(model, data)

        viewer.render()
    else:
        break


viewer.close()