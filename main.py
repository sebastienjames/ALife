import mujoco
import mujoco_viewer
import numpy as np
import time
import random
import xml.etree.ElementTree as ET
from copy import deepcopy

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
    
    def is_valid_joint(self, joint, body):
        xmax = body.size[0] / 2
        xmin = -xmax

        zmax = body.size[1] / 2
        zmin = -zmax

        ymax = body.size[2] / 2
        ymin = -ymax

        if joint.pos[0] < xmin:
            joint.pos[0] = xmin
        if joint.pos[0] > xmax:
            joint.pos[0] = xmax
        if joint.pos[1] < zmin:
            joint.pos[1] = zmin
        if joint.pos[1] > zmax:
            joint.pos[1] = zmax
        if joint.pos[2] < ymin:
            joint.pos[2] = ymin
        if joint.pos[2] > ymax:
            joint.pos[2] = ymax

        if self.side == 0:
            # print(body.size)
            joint.pos[2] = -body.size[2]
        if self.side == 1:
            joint.pos[0] = body.size[0]
        if self.side == 2:
            joint.pos[1] = body.size[1]
        if self.side == 3:
            joint.pos[2] = body.size[2]
        if self.side == 4:
            joint.pos[0] = -body.size[0]
        if self.side == 5:
            joint.pos[1] = -body.size[1]

        return joint
    
    def fit_to(self, parent, newbody):
        if parent is None:
            return
        
        if self.side == 0:
            newbody.pos[2] = parent.bodies[0].size[2] + newbody.size[2]
            # print("Up")
        elif self.side == 1:
            newbody.pos[1] = parent.bodies[0].size[1] + newbody.size[1]
            # print("Front")
        elif self.side == 2:
            newbody.pos[0] = parent.bodies[0].size[0] + newbody.size[0]
            # print("Left")
        elif self.side == 3:
            newbody.pos[1] = - parent.bodies[0].size[1] - newbody.size[1]
            # print("Back")
        elif self.side == 4:
            newbody.pos[0] = - parent.bodies[0].size[0] - newbody.size[0]
            # print("Right")
        else:
            newbody.pos[2] = - parent.bodies[0].size[2] - newbody.size[2]
            # print("Down")


    def mutate(self, parent):
        newbodies = []
        
        for body in self.bodies:
            newbody = body.mutate()
            if newbody:
                self.fit_to(parent, newbody)
                newbodies.append(newbody)

            else:
                # The body has atrophied, so remove all children
                # print(self.name)
                return None
            
        

        newjoints = []

        for joint in self.joints:
            newjoint = joint.mutate()
            # print("NWEJOINT", newjoint.ctrlrange)
            if newjoint:

                newjoint = self.is_valid_joint(newjoint, newbody)

                newjoints.append(newjoint)
            else:
                newjoints.append(joint)
        
        if random.random() < CHANCE_ADD_BODY:
            #add body
            #make_random_children(self, 1)
            pass


        return Vertex(newbodies, newjoints, self.name + "_new", self.side)

class Edge:
    def __init__(self, dest, limit):
        self.dest = dest
        self.limit = limit

    def copy(self):
        return Edge(self.dest, self.limit)
    
    def __str__(self):
        return "<Edge to " + self.dest + ", limit:" + str(self.limit) + ">"

class Graph:
    def __init__(self):
        self.graph = {}
        self.vertex_data = {}
        self.base = None #name

    def add_vertex(self, data: Vertex):

        if data.name in self.graph:
            raise NameError(data.name + " already in graph")
        if self.base == None:
            self.base = data.name
        self.graph[data.name] = []
        
        self.vertex_data[data.name] = data
        # self.print_graph()

    def remove_vertex(self, name):
        del self.graph[name]
        
        for connections in self.graph.values():
            i = 0
            while i < len(connections):
                if connections[i].dest == name:
                    # print("REMOVE")
                    del connections[i]
                else:
                    i += 1

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

        # print(output)
        # print(self.graph)
        return output
    
    def mutate(self):
        newGraph = Graph()
        newGraph.graph = deepcopy(self.graph)
        newGraph.vertex_data = deepcopy(self.vertex_data)
        newGraph.base = self.base

        self._mutate_helper(self.base, newGraph, None)

        # print("LOKK")
        # newGraph.print_graph()
        return newGraph
        
    
    def _mutate_helper(self, name, into, parent):
        # print("MUTATE HELPING", name)
        mutated_body = self.vertex_data[name].mutate(parent)
        while mutated_body == None and name == self.base:
            # print("CANT ATRPOHY INTO NOTHING")
            mutated_body = self.vertex_data[name].mutate(parent)

        if mutated_body == None:
            # print("deleting", name)
            into.remove_vertex(name)
            return
        
        into.vertex_data[name] = mutated_body

        # print(self.graph[name])

        for child in self.graph[name]:
            # print("CHILD IS ", child)
            # print("DEST IS", child.dest)
            self._mutate_helper(child.dest, into, into.vertex_data[name])
        
        if random.random() < CHANCE_ADD_BODY:
            #add body
            # make_random_children(self.vertex_data[name], 1, self)
            pass

    

    def print_graph(self):
        print("------------------------------------")
        print("Graph:")
        for k, v in self.graph.items():
            print(k, ":", list(map(lambda x: x.dest, v)))
        print("Data:")
        for k, v in self.vertex_data.items():
            print(k,":", v)
        print("------------------------------------")


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
    def __init__(self, kind, axis=[0, 1, 0], pos=[0, 0, 0], cycle=[5, 5], gear=1, ctrlrange=[-1, 1]) -> None:
        self.kind = kind
        self.axis = axis
        self.pos = pos
        self.gear = gear
        self.cycle = cycle
        self.ctrlrange = list(map(lambda x: x * self.gear, ctrlrange))
        # print("HEREd", self.ctrlrange)
        # print("ZA", ctrlrange, gear)

    def copy(self, offset=[0, 0, 0]):
        if self.kind != "free":
            newpos = self.pos[:]
            for i in range(3):
                newpos[i] += offset[i]

            oldctrl = list(map(lambda x: x / self.gear, self.ctrlrange))
            # print("OLD:", oldctrl)
            return Joint(kind=self.kind, axis=self.axis, pos=newpos, gear=self.gear, ctrlrange=oldctrl)
        
    def mutate(self):
        newjoint = self.copy()
        
        if newjoint:
            # print(newjoint.ctrlrange)
            add_elements(newjoint.pos, [(random.random() - 0.5) * MUTATION_SCALE for i in range(3)])

            return newjoint
        return self

    
    
    def traverse(self, name):
        # print("LOOK", self.ctrlrange)
        joint = "<joint name = \"j" + name + "\" type = \"" + self.kind + "\" axis = \"" + " ".join(map(str, self.axis)) + "\" pos = \"" + " ".join(map(str, self.pos)) + "\" limited = \"true\" range = \"-90 90\"/>\n"
        motor = "<motor name = \"m" + name + "\" joint = \"j" + name + "\" gear = \"" + str(self.gear) + "\" ctrllimited = \"true\" ctrlrange = \"" + " ".join(map(str, self.ctrlrange)) + "\"/>\n"
        return (joint, motor)




        

def render(obj: Graph, vis = False):

    # print("BASE:", obj.base)
    # obj.print_graph()

    xml = obj.traverse(obj.base)

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

                if np.all(data.qacc < 1e5):
                    max_dist = max(get_pos(data.xpos), max_dist)
                else:
                    return 0

                
                mujoco.mj_step(model, data)
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

        print(str(_ / number * 100) + "%", end = "\r")

        make_random_children(base, 0, creature)



        # times = random.randint(0, 4)
        # base.add_child(base, times)

        score = render(creature, True)
        

        models = np.append(models, creature)

        # if score > 20:
        #     scores = np.append(scores, 0)
        # else:
        scores = np.append(scores, score)

    # EVOLUTION!
            
    for _ in range(generations):
        print("Scores (gen %s):" % (_))
        print(scores)
        
        best_model = models[scores.argmax()]
        high_score = scores.max()


        models = [best_model]
        # best_model.print_graph()
        for i in range(number):
            newmodel = best_model.mutate()
            # best_model.print_ graph()
            while newmodel == None:
                # print("BAD MODEL")
                newmodel = best_model.mutate()
            models.append(newmodel)
        # best_model.print_graph()
        scores = np.array([])

        for i, model in enumerate(models):
            score = render(model)
            if score - high_score > 20:
                scores = np.append(scores, 0)
            else:
                scores = np.append(scores, score)
            print(str(i / number * 100) + "%", end = "\r")


    print("Here's the best model with score", scores.max())

    render(best_model, True)

    print(scores)

    

        
         

def make_random_children(parent, level: int, graph: Graph):
    if level < 4:

        num = random.randint(1 - level, 4 - level)
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

if False:
    n_creatures = int(input("How many creatures per generation?\n"))
    n_generations = int(input("How many generations?\n"))

    make_random(n_creatures, n_generations)
else:

    for i in range(5):
        make_random(10, 1)
    