import mujoco
import mujoco_viewer
import numpy as np
import time
import random

PI = np.pi
MAX_SEGMENT_LENGTH = 0.3

DIRECTIONS = [
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0]
]

def add_elements(a, b):
    for e in b:
        a += e
    

"""

LIMITS:

RECURSIONS NOT USED

Node can only have one recursive pointer
Node.recursions are unused (for now)

Clock-based movement

Can overlap and cross into itself

"""

# class Node:
#     def __init__(self, data, children, recursions):
#         self.data = data
#         self.children = children
#         self.recursions = recursions
    
#     def add_child(self, other, times = -1):
#         # -1 if infinite (only for non-recursive nodes)
#         # otherwise times is for the number of times this child should be recursed on
#         if other == self:
#             if times == -1:
#                 raise RecursionError
#             else:
#                 child = self.copy()
#                 for x in range(len(child.children)):
#                     if child.children[x] == self:
#                         print("ME")
#                         child.children[x] == child
#                         child.recursions[x] -= 1 # Check

#                 if times > 1:
#                     child.add_child(child, times - 1)

#                 self.children.append(child)
#                 self.recursions.append(times)
#         else:
#             self.children.append(other)
#             self.recursions.append(times)
            

#     def debug(self):
#         print("-----------------")
#         print(id(self))
#         print()
#         for pointer, limit in zip(self.children, self.recursions):
#             print(id(pointer), limit)
#         print("-----------------")


#     def copy(self):
#         return Node(self.data, self.children[:], self.recursions[:])

#     def expand(self, tab):
#         print(tab * "  " + self.data) # Visted
#         for i in range(len(self.children)):
#             if self.recursions[i] != 0:
#                 self.recursions[i] -= 1
#                 self.children[i].expand(tab + 1)



# root = Node("ROOT", [], [])
# child = Node("CHILD", [], [])
# child2 = Node("CHILD2", [], [])
# root.add_child(child)
# root.add_child(root, 2)
# root.add_child(root, 2)


# root.expand(0)

class Node:
    def __init__(self, bodies, joints, children, name, recursions):
        self.bodies = bodies
        self.joints = joints
        self.children = children
        self.name = name
        self.recursions = recursions

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
    
    def add_child(self, other, times = -1):
        # -1 if infinite (only for non-recursive nodes)
        # otherwise times is for the number of times this child should be recursed on
        if other == self:
            if times == -1:
                raise RecursionError
            else:
                child = self.copy()
                print("ME")
                # Necessary for multiple of the recursive nodes
                # for x in range(len(child.children)):
                #     if child.children[x] == self:
                #         child.children[x] == child
                #         child.recursions[x] -= 1 # Check

                if times > 1:
                    child.add_child(child, times - 1)

                self.children.append(child)
                self.recursions.append(times)
        else:
            self.children.append(other)
            self.recursions.append(times)
    
    def copy(self):
        newbodies = self.bodies[:]
        newjoints = []

        for joint in self.joints:
            newjoint = joint.copy()
            if newjoint:
                newjoints.append(newjoints)
                

        return Node(newbodies, newjoints, self.children[:], self.name + "_new", self.recursions[:])

    def expand(self, tab):
        print(tab * "  " + self.data) # Visted
        for i in range(len(self.children)):
            if self.recursions[i] != 0:
                self.recursions[i] -= 1
                self.children[i].expand(tab + 1)
    



    ##################################
    def create_node(self, template):
        # template: -1 is self, other positive is the position in children
        if template == -1:

            self.children.append(Node(self.bodies, self.joints, self.children, self.name + "_new"))

        elif template >= 0:
            Node()
    

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
    
    def copy(self, offset):
        newpos = list(self.pos)
        for i in range(2):
            newpos[i] += offset[i]

        return Body(newpos, self.size, self.rgba)

class Joint:
    def __init__(self, kind, axis=[0, 1, 0], pos=[0, 0, 0], gear=1, ctrlrange=[-1, 1]) -> None:
        self.kind = kind
        self.axis = axis
        self.pos = pos
        self.gear = gear
        self.ctrlrange = list(map(lambda x: x * self.gear, ctrlrange))

    def copy(self, offset=[0, 0, 0]):
        if self.kind != "free":
            newpos = self.pos[:]
            for i in range(3):
                newpos[i] += offset[i]

            oldctrl = list(map(lambda x: x / self.gear, self.ctrlrange))
            
            return Joint(self.kind, self.axis, newpos, self.gear, oldctrl)
    
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
        "foot",
        []
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
        "foot2",[]
    )],
    "head",
    []
)

body1 = Body([0, 0, 0.1], [0.1, 0.1, 0.1], [1, 0, 0, 1])
body2 = Body([0, 0, 0.2], [0.1, 0.1, 0.1], [0, 1, 0, 1])

test = Node([body1], [Joint("free")], [], "test", [])

# test.add_child(Node([body2], [], [], "sec", []))

test.add_child(test, 1)



def render(obj):

    xml = str(obj)

    # print(xml)

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # create the viewer object
    viewer = mujoco_viewer.MujocoViewer(model, data)

    #Get array of actuators
    actuators = model.nu

    # print(data.ctrl)

    # print(actuators)

    for i in range(400):
        mujoco.mj_step(model, data)

    time.sleep(1)

    for i in range(10000):
        if viewer.is_alive:
            # Currently clock-based movement
            if i % 300 == 0:
                data.ctrl[:actuators] = [5]
                print(data.ctrl)
            elif i % 300 == 150:
                data.ctrl[:actuators] = [-5]
            # print(data.ctrl)
            mujoco.mj_step(model, data)
            viewer.render()
        else:
            break


    viewer.close()


def make_random(number):
    for _ in range(number):
        base = Node([make_random_body()], [Joint("free")], [], "base", [])
        
        make_random_children(base, 0)



        # times = random.randint(0, 4)
        # base.add_child(base, times)
        


        render(base)
         

def make_random_children(parent, level):
    if level < 3:

        num = random.randint(2 - level, 4 - level)
        num = num < 0 and 0 or num

        for i in range(num):
            body = make_random_body()
            parent_size = parent.bodies[0].size

            # Which side the limb is going on
            side = random.randint(0, 5)

            if side == 0:
                # UP
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                y_coord = (parent_size[2] + body.size[2]) 

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, 0, -body.size[2]], 5)

            if side == 1:
                # FRONT
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord = -(parent_size[1] + body.size[1]) 

                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, body.size[1], 0], 5)

            if side == 2:
                # LEFT

                x_coord = -(parent_size[0] + body.size[0]) 

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                
                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [body.size[0], 0, 0], 5)

            if side == 3:
                # BACK
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord = (parent_size[1] + body.size[1]) 

                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, -body.size[1], 0], 5)

            if side == 4:
                # RIGHT

                x_coord = (parent_size[0] + body.size[0]) 

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                
                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [-body.size[0], 0, 0], 5)

            if side == 5:
                # DOWN
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                y_coord = (parent_size[2] + body.size[2]) 

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, 0, -body.size[2]], 5)

            child = Node([body], [joint], [], "limb_" + parent.name + "_" + str(i), [])

            make_random_children(child, level + 1)

            parent.add_child(child)



        


def make_random_body():
    dimensions = [random.random() * MAX_SEGMENT_LENGTH for i in range(3)]
    return Body([0, 0, dimensions[2]], dimensions, [random.random(), random.random(), random.random(), 1])


make_random(1)

