import mujoco
import mujoco_viewer
import numpy as np
import time
import random
import xml.etree.ElementTree as ET

"""
Sides:

Up    : 0
Front : 1
Left  : 2
Back  : 3
Right : 4
Down  : 5

"""

PI = np.pi
MAX_SEGMENT_LENGTH = 0.3
MUTATION_SCALE = 0.1
NUM_SIMULATION_ITERATIONS = 10000
DIRECTIONS = [
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0]
]
CHANCE_ADD_BODY = 0.20
GLOBAL_ID = 10000

def add_elements(a, b):
    for i in range(len(a)):
        a[i] += b[i]

def is_zero(l):
    for i in range(len(l)):
        if l[i] < 0.01:
            return True
    return False

def get_pos(pos):
    return np.max(np.apply_along_axis(np.linalg.norm, 1, pos))


"""

LIMITS:

RECURSIONS NOT USED, BUT ARE IMPLEMENTED

Clock-based movement

Can overlap and cross into itself

Need to move joints and stuff after mutating

Only uses one body / joint

"""

class Stack:
    """
    Standard stack, just wanted custom print messages
    """
    def __init__(self):
        self.q = []

    def append(self, thing):
        self.q.append(thing)

    def pop(self):
        return self.q.pop(0)
    
    def __len__(self):
        return len(self.q)
    
    def __str__(self):
        string = "<Queue: ["
        for i in self.q:
            string += str(i) + ", "
        string += "]>"
        return string
    

class Graph:
    def __init__(self):
        self.graph = {}
        self.vertex_data = {}

    def add_vertex(self, data):
        if data.name in self.graph:
            raise NameError(data.name + " already in graph")
        self.graph[data.name] = []
        self.vertex_data[data.name] = data

    def add_edge(self, src, dest, limit=-1):
        if src == dest:
            assert limit > 0, "Recursive edge needs a limit"

        self.graph[src].append(Edge(dest, limit))


    #recursion
        
    def dfs(self, name):

        f, l, m = self.vertex_data[name].to_string()

        first = f
        last = l
        motors = m

        for child in self.graph[name]:
            f, l, m = self.dfs(child.dest)
            first += f
            first += l
            motors += m
        
        return (first, last, motors)



    def traverse(self, base_name):
        output = """
<mujoco>
<worldbody>
<light diffuse=".5 .5 .5" pos="0 0 2" dir="0 -1 0"/>
<geom type="plane" size="20 20 0.1" rgba="0.8 0.8 0.8 1"/>
"""
        last = ""
        motors = ""

        f, l, m = self.dfs(base_name)

        

        output += f
        last = l + last
        motors += m
        
        output += last
        output += "</worldbody>\n<actuator>\n"

        output += motors
        
        output += "</actuator>\n</mujoco>"

        print(output)
        print(self.graph)
        return output



class Vertex:
    def __init__(self, bodies, joints, name, side):
        self.bodies = bodies
        self.joints = joints
        self.name = name
        self.side = side



    def to_string(self):
        fstring = ""
        lstring = ""
        motors = ""

        fstring += self.bodies[0].traverse(self.name + "0")

        for joint in self.joints:
            j, m = joint.traverse(self.name)
            fstring += j
            motors += m

        for i in range(1, len(self.bodies)):
            fstring += self.bodies[i].traverse(self.name + str(i))
        
        for i in range(len(self.bodies)):
            lstring = "</body>\n" + lstring

        return (fstring, lstring, motors)

class Edge:
    def __init__(self, dest, limit):
        self.dest = dest
        self.limit = limit

    def copy(self):
        return Edge(self.dest, self.limit)
    
    def __str__(self):
        return "<Edge to " + self.dest + ", limit:" + str(self.limit) + ">"


class Body:
    def __init__(self, pos, size, rbga):
        self.pos = pos
        self.size = size
        self.rgba = rbga
        self.kind = "box"


    def traverse(self, name):
        string = "<body name = \"" + name + "\" pos = \"" + " ".join(map(str, self.pos)) + "\">\n"
        string += "<geom type = \"" + self.kind + "\" size = \"" + " ".join(map(str, self.size)) + "\" rgba = \"" + " ".join(map(str, self.rgba)) + "\"/>\n"
        return string
    
    def copy(self, offset = [0, 0, 0]):
        newpos = list(self.pos)
        for i in range(2):
            newpos[i] += offset[i]

        return Body(newpos, self.size[:], self.rgba)
    
    def mutate(self):
        newbody = self.copy()

        add_elements(newbody.pos, [(random.random() - 0.5) * MUTATION_SCALE for i in range(3)])
        add_elements(newbody.size, [(random.random() - 0.5) * MUTATION_SCALE for i in range(3)])
    
        if not is_zero(newbody.size):
            return newbody

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
        
    def mutate(self):
        newjoint = self.copy()
        if newjoint:
            add_elements(newjoint.pos, [(random.random() - 0.5) * MUTATION_SCALE for i in range(3)])

            return newjoint
        return self

    
    
    def traverse(self, name):
        joint = "<joint name = \"j" + name + "\" type = \"" + self.kind + "\" axis = \"" + " ".join(map(str, self.axis)) + "\" pos = \"" + " ".join(map(str, self.pos)) + "\"/>\n"
        motor = "<motor name = \"m" + name + "\" joint = \"j" + name + "\" gear = \"" + str(self.gear) + "\" ctrllimited = \"true\" ctrlrange = \"" + " ".join(map(str, self.ctrlrange)) + "\"/>\n"
        return (joint, motor)




        

def render(obj, vis = False):

    xml = obj

    with open("creature.xml", "w+") as f:
        f.write(xml)

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # create the viewer object
    

    #Get array of actuators
    actuators = model.nu

    # print(data.ctrl)

    # print(actuators)

    # for i in range(400):
    #     mujoco.mj_step(model, data)
    if vis:
        time.sleep(1)

    max_dist = 0

    # print(data.xpos)

    if vis:
        viewer = mujoco_viewer.MujocoViewer(model, data)
        for i in range(NUM_SIMULATION_ITERATIONS):
            if viewer.is_alive:
                # Currently clock-based movement
                if i % 300 == 0:
                    data.ctrl[:actuators] = [5]
                    
                elif i % 300 == 150:
                    data.ctrl[:actuators] = [-5]
                max_dist = max(get_pos(data.xpos), max_dist)
                
                mujoco.mj_step(model, data)
                if vis:
                    viewer.render()
            else:
                break

    
        viewer.close()
    
    else:
        for i in range(NUM_SIMULATION_ITERATIONS):
            if i % 300 == 0:
                data.ctrl[:actuators] = [5]
                
            elif i % 300 == 150:
                data.ctrl[:actuators] = [-5]
            max_dist = max(get_pos(data.xpos), max_dist)

            mujoco.mj_step(model, data)

    return max_dist




def make_random(number, generations):

    models = np.array([])
    scores = np.array([])
    
    for _ in range(number):
        creature = Graph()
        base = Vertex([make_random_body([0, 0, 0, 1])], [Joint("free")], "base", 0)
        creature.add_vertex(base)
        # base = bad_model
        # So the creatures don't take off by starting underground
        base.bodies[0].pos[2] += 1

        print(str(_ / number * 100) + "%")

        make_random_children(base, 0, creature)



        # times = random.randint(0, 4)
        # base.add_child(base, times)

        score = render(creature.traverse(base.name), True)
        

        models = np.append(models, base)

        if score > 20:
            scores = np.append(scores, 0)
        else:
            scores = np.append(scores, score)

    # EVOLUTION!
    # for _ in range(generations):
    #     print("Scores (gen %s):" % (_))
    #     print(scores)
        
    #     best_model = models[scores.argmax()]
    #     high_score = scores.max()


    #     models = [best_model]
    #     for i in range(number):
    #         newmodel = best_model.mutate()
    #         while newmodel == None:
    #             # print("BAD MODEL")
    #             newmodel = best_model.mutate()
    #         models.append(newmodel)

    #     scores = np.array([])

    #     for i, model in enumerate(models):
            
    #         score = render(model)
    #         if score - high_score > 20:
    #             scores = np.append(scores, 0)
    #         else:
    #             scores = np.append(scores, score)
    #         if (i / number * 100) % 10 == 0:
    #             print(str(i / number * 100) + "%")


    # print("Here's the best model with score", scores.max())

    # render(best_model, True)

    # print(scores)

    

        
         

def make_random_children(parent, level: int, graph: Graph):
    if level < 3:

        num = random.randint(1 - level, 3 - level)
        # num = 2
        num = num < 0 and 0 or num

        for i in range(num):
            # body = make_random_body(parent.bodies[0].rgba)
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


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, 0, -body.size[2]], random.randint(1, 5))

            if side == 1:
                # FRONT
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord = -(parent_size[1] + body.size[1]) 

                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, body.size[1], 0], random.randint(1, 5))

            if side == 2:
                # LEFT

                x_coord = -(parent_size[0] + body.size[0]) 

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                
                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [body.size[0], 0, 0], random.randint(1, 5))

            if side == 3:
                # BACK
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord = (parent_size[1] + body.size[1]) 

                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, -body.size[1], 0], random.randint(1, 5))

            if side == 4:
                # RIGHT

                x_coord = (parent_size[0] + body.size[0]) 

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                
                y_coord_max = parent_size[2] + body.size[2]
                y_coord = (random.random() - 0.5) * y_coord_max

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [-body.size[0], 0, 0], random.randint(1, 5))

            if side == 5:
                # DOWN
                x_coord_max = parent_size[0] + body.size[0]
                x_coord = (random.random() - 0.5) * x_coord_max

                z_coord_max = parent_size[1] + body.size[1]
                z_coord = (random.random() - 0.5) * z_coord_max

                y_coord = (parent_size[2] + body.size[2]) 

                body.pos = [x_coord, z_coord, y_coord]


                joint = Joint("hinge", DIRECTIONS[random.randint(0, 2)], [0, 0, -body.size[2]], random.randint(1, 5))


            child = Vertex([body], [joint], "limb_" + parent.name + "_" + str(i), side)
            graph.add_vertex(child)
            graph.add_edge(parent.name, child.name)
            make_random_children(child, level + 1, graph)

            # print(graph.traverse(parent.name))



        


def make_random_body(parent_color=None):
    dimensions = [random.random() * MAX_SEGMENT_LENGTH for i in range(3)]
    if parent_color == None:
        color = [random.random(), random.random(), random.random(), 1]
    else:
        color = [0, 0, 0, 1]
        color[0] = parent_color[0] + 0.3
        color[1] = parent_color[1] + 0.3
        color[2] = parent_color[2] + 0.3

    return Body([0, 0, dimensions[2]], dimensions, color)







main_side = 0.3
sub_side = 0.2

graph_model = Graph()

graph_model.add_vertex(Vertex(
    [Body(
        [0, 0, 1],
        [main_side, main_side, main_side],
        [1, 1, 0, 1]
    ),
    Body(
        [0, main_side * 2, 0],
        [main_side, main_side, main_side],
        [0, 1, 0, 1]
    )],
    [Joint("free")],
    "main",
    0
))

graph_model.add_vertex(Vertex(
    [Body(
        [0, 0, main_side + sub_side],
        [sub_side, sub_side, sub_side],
        [1, 0, 1, 1]
    )],
    [Joint("hinge", [0, 1, 0], [0, 0, -sub_side])],
    "arm",
    0
))

graph_model.add_edge("main", "arm")

# render(graph_model.traverse("main"), True)

# graph_model.dfs("main")

make_random(1, 10)

# render(bad_model, True)

# render(bad_model.mutate(), True)