import mujoco
import mujoco_viewer
import time

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

data.qvel[0] = -4
#data.qvel[1] = 4
data.qpos[0] = 1

# simulate and render
for _ in range(10000):
    print("pos")
    print(data.qpos)
    print("vel")
    print(data.qvel)


    if viewer.is_alive:
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break



# close
viewer.close()