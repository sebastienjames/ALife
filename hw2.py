import mujoco
import mujoco_viewer
import numpy as np
import time

PI = np.pi

class Node:
    def __init__(self, bodies, joints, children, name):
        self.bodies = bodies
        self.joints = joints
        self.children = children
        self.name = name

    def __str__(self) -> str:
        string = """<mujoco>
              <worldbody>
              <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 -1 0"/>
              <geom type="plane" size="10 10 0.1" rgba="0 0 0.9 1"/>"""

        body, j = self.traverse()

        string += body

        string += "</worldbody>\n<actuator>\n"

        for e in j:
            string += e[1]
        
        string += "</actuator>\n</mujoco>"

        return string



    
    def traverse(self):
        string = ""
        
        string += self.get_body()

        j = self.get_joints()

        for e in j:
            string += e[0]

        
        for child in self.children:
            s, k = child.traverse()
            j += k
            string += s

        string += "</body>\n"

        

        return (string, j)
        
    
    def get_body(self):
        string = ""
        for body in self.bodies:
            string += body.traverse(self.name)
        return string
    
    def get_joints(self):
        j = []

        for name_ext, joint in enumerate(self.joints):
            j.append(joint.traverse(self.name + "v" + str(name_ext)))

        return j

class Body:
    def __init__(self, pos, size, rbga):
        self.pos = pos
        self.size = size
        self.rgba = rbga
        self.kind = "box"


    def traverse(self, name):
        string = "<body name = \"" + name + "\" pos = \"" + " ".join(map(str, self.pos)) + "\">\n"
        string += "<geom type = \"" + self.kind + "\" size = \"" + " ".join(map(str, self.size)) + "\" rgba = \"" + " ".join(map(str, self.rgba)) + "\"/>"
        return string

class Joint:
    def __init__(self, kind, axis=[0, 1, 0], pos=[0, 0, 0], gear=1, ctrlrange=[-1, 1]) -> None:
        self.kind = kind
        self.axis = axis
        self.pos = pos
        self.gear = gear
        self.ctrlrange = list(map(lambda x: x * self.gear, ctrlrange))
    
    def traverse(self, name):
        joint = "<joint name = \"j" + name + "\" type = \"" + self.kind + "\" axis = \"" + " ".join(map(str, self.axis)) + "\" pos = \"" + " ".join(map(str, self.pos)) + "\"/>\n"
        motor = "<motor name = \"m" + name + "\" joint = \"j" + name + "\" gear = \"" + str(self.gear) + "\" ctrllimited = \"true\" ctrlrange = \"" + " ".join(map(str, self.ctrlrange)) + "\"/>\n"
        return (joint, motor)



figure = Node(
    [Body(
        [0, 0, 0.3],
        [0.1, 0.1, 0.1],
        [1, 0, 0, 1]
    )],
    [
        Joint(
            "free"
        )
    ],
    [Node(
        [
            Body(
                [0, 0.3, 0.2],
                [0.1, 0.2, 0.1],
                [0, 1, 0, 1]
            )
        ],
        [
            Joint(
                "hinge",
                [1, 0, 0],
                [0, -0.2, -0.1],
                10
            )
        ],
        [],
        "foot"
    ),
    Node(
        [
            Body(
                [0, -0.3, 0.2],
                [0.1, 0.2, 0.1],
                [0, 1, 0, 1]
            )
        ],
        [
            Joint(
                "hinge",
                [1, 0, 0],
                [0, 0.2, -0.1],
                10
            ),
            Joint(
                "hinge",
                [0, 0, 1],
                [0, 0.2, -0.1],
                10
            )
        ],
        [],
        "foot2"
    )],
    "head"
)



xml = str(figure)

print(xml)

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

#Get array of actuators
actuators = model.nu

# print(data.ctrl)

# print(actuators)

# for i in range(400):
#     mujoco.mj_step(model, data)

time.sleep(1)

for i in range(10000):
    if viewer.is_alive:

        if i % 300 == 0:
            data.ctrl[:actuators] = [5]
            print(data.ctrl)
        elif i % 300 == 150:
            data.ctrl[:actuators] = [-5]
        print(data.ctrl)
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break


viewer.close()