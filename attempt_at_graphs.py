import collections

def printf(self, thing=None):
    if False:
        print(self)

class Queue:
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

    def add_vertex(self, name) :
        if name in self.graph:
            raise NameError(name + " already in graph")
        self.graph[name] = []

    def add_edge(self, src, dest, limit=-1):
        if src == dest:
            assert limit > 0, "Recursive edge needs a limit"

        self.graph[src].append(Edge(dest, limit))

    def traverse(self, base_name):
        q = Queue()

        for child in self.graph[base_name]:
            q.append(child.copy())

        print(base_name)

        while len(q) > 0:
            next = q.pop()
            printf("NEXT IS", next)
            # Not a recursive node
            if next.limit == -1:
                print(next.dest)
                for child in self.graph[next.dest]:
                    q.append(child.copy())

            # Is a recursive node
            if next.limit > 0:
                print(next.dest)
                for child in self.graph[next.dest]:
                    newchild = child.copy()
                    if newchild.dest == next.dest:
                        newchild.limit = next.limit - 1
                        q.append(newchild)
                    elif newchild.limit == -1:
                        q.append(newchild)
                        


class Edge:
    def __init__(self, dest, limit):
        self.dest = dest
        self.limit = limit

    def copy(self):
        return Edge(self.dest, self.limit)
    
    def __str__(self):
        return "<Edge to " + self.dest + ", limit:" + str(self.limit) + ">"
    


g = Graph()

g.add_vertex("A")
g.add_vertex("B")
# g.add_vertex("D")
# g.add_vertex("E")

g.add_edge("A", "A", 2)
g.add_edge("A", "A", 2)

g.traverse("A")