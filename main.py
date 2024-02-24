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
    
    def is_valid_joint(self, joint):
        xmax = self.bodies[0].size[0] / 2
        xmin = -xmax

        zmax = self.bodies[0].size[1] / 2
        zmin = -zmax

        ymax = self.bodies[0].size[2] / 2
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
            # print(self.bodies[0].size)
            joint.pos[2] = -self.bodies[0].size[2]
        if self.side == 1:
            joint.pos[0] = self.bodies[0].size[0]
        if self.side == 2:
            joint.pos[1] = self.bodies[0].size[1]
        if self.side == 3:
            joint.pos[2] = self.bodies[0].size[2]
        if self.side == 4:
            joint.pos[0] = -self.bodies[0].size[0]
        if self.side == 5:
            joint.pos[1] = -self.bodies[0].size[1]

        return joint
    
    def mutate(self):
        newbodies = []
        
        for body in self.bodies:
            newbody = body.mutate()
            if newbody:
                newbodies.append(newbody)
            else:
                # The body has atrophied, so remove all children
                # print(self.name)
                return None

        newjoints = []

        for joint in self.joints:
            newjoint = joint.mutate()
            if newjoint:

                newjoint = self.is_valid_joint(newjoint)

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
        self.base = None

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
    def mutate(self, base_name):
        newGraph = Graph()
        newGraph.graph = deepcopy(self.graph)
        newGraph.vertex_data = deepcopy(self.vertex_data)
        newGraph.base = self.base

        self._mutate_helper(base_name, newGraph)

        # print("LOKK")
        # newGraph.print_graph()
        return newGraph
        
    
    def _mutate_helper(self, name, into):
        # print("MUTATE HELPING", name)
        mutated_body = self.vertex_data[name].mutate()
        while mutated_body == None and name == self.base:
            # print("CANT ATRPOHY INTO NOTHING")
            mutated_body = self.vertex_data[name].mutate()

        if mutated_body == None:
            # print("deleting", name)
            self.remove_vertex(name)
            return
        
        into.vertex_data[name] = mutated_body

        # print(self.graph[name])

        for child in self.graph[name]:
            # print("CHILD IS ", child)
            # print("DEST IS", child.dest)
            self._mutate_helper(child.dest, into)
        
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

class Node:
    def __init__(self, bodies, joints, children, name, side, recursions):
        self.bodies = bodies
        self.joints = joints
        self.children = children
        self.name = name
        self.side = side
        self.recursions = recursions
        

    def __str__(self) -> str:
        string = """
<mujoco>
 <worldbody>
 <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 -1 0"/>
 <geom type="plane" size="10 10 0.1" rgba="0 0 0.9 1"/>
 """

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
                

        return Node(newbodies, newjoints, self.children[:], self.name + "_new", self.side, self.recursions[:])
    
    def is_valid_joint(self, joint):
        xmax = self.bodies[0].size[0] / 2
        xmin = -xmax

        zmax = self.bodies[0].size[1] / 2
        zmin = -zmax

        ymax = self.bodies[0].size[2] / 2
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
            # print(self.bodies[0].size)
            joint.pos[2] = -self.bodies[0].size[2]
        if self.side == 1:
            joint.pos[0] = self.bodies[0].size[0]
        if self.side == 2:
            joint.pos[1] = self.bodies[0].size[1]
        if self.side == 3:
            joint.pos[2] = self.bodies[0].size[2]
        if self.side == 4:
            joint.pos[0] = -self.bodies[0].size[0]
        if self.side == 5:
            joint.pos[1] = -self.bodies[0].size[1]

        return joint


    def mutate(self):
        newbodies = []
        
        for body in self.bodies:
            newbody = body.mutate()
            if newbody:
                newbodies.append(newbody)
            else:
                # The body has atrophied, so remove all children
                # print(self.name)
                return None

        newjoints = []

        for joint in self.joints:
            newjoint = joint.mutate()
            if newjoint:

                newjoint = self.is_valid_joint(newjoint)

                newjoints.append(newjoint)
            else:
                newjoints.append(joint)

        newchildren = []

        for child in self.children:
            newchild = child.mutate()
            if newchild:
                newchildren.append(newchild)
        
        if random.random() < CHANCE_ADD_BODY:
            #add body
            make_random_children(self, 1)


        return Node(newbodies, newjoints, newchildren, self.name + "_new", self.side, self.recursions[:])

    def expand(self, tab):
        print(tab * "  " + self.data) # Visted
        for i in range(len(self.children)):
            if self.recursions[i] != 0:
                self.recursions[i] -= 1
                self.children[i].expand(tab + 1)

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

        print(str(_ / number * 100) + "%", end = "\r")

        make_random_children(base, 0, creature)



        # times = random.randint(0, 4)
        # base.add_child(base, times)

        score = render(creature)
        

        models = np.append(models, creature)

        if score > 20:
            scores = np.append(scores, 0)
        else:
            scores = np.append(scores, score)

    # EVOLUTION!
            
    for _ in range(generations):
        print("Scores (gen %s):" % (_))
        print(scores)
        
        best_model = models[scores.argmax()]
        high_score = scores.max()


        models = [best_model]
        for i in range(number):
            newmodel = best_model.mutate(best_model.base)
            while newmodel == None:
                # print("BAD MODEL")
                newmodel = best_model.mutate(best_model.base)
            models.append(newmodel)

        scores = np.array([])

        for i, model in enumerate(models):
            score = render(model)
            if score - high_score > 20:
                scores = np.append(scores, 0)
            else:
                scores = np.append(scores, score)
            if (i / number * 100) % 10 == 0:
                print(str(i / number * 100) + "%", end = "\n")


    print("Here's the best model with score", scores.max())

    render(best_model, True)

    print(scores)

    

        
         

def make_random_children(parent, level: int, graph: Graph):
    if level < 3:

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

if True:
    n_creatures = int(input("How many creatures per generation?\n"))
    n_generations = int(input("How many generations?\n"))

    make_random(n_creatures, n_generations)
else:




    make_random(1, 10)

    # render(graph_model.traverse("main"), True)

    # graph_model.dfs("main")



    # render(bad_model, True)

    # render(bad_model.mutate(), True)