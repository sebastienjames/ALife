class Node:
    def __init__(self, data, children, recursions=None):
        self.data = data
        self.children = children
        # self.recursions = recursions
    
    def add_child(self, other, times = -1):
        self.children.append(Pointer(other, times))
        # self.debug()

    def debug(self):
        for i in self.children:
            print(i.destination.data, i.times)

    def expand(self, tab):
        # # First, expand the recursive nodes:
        # for rec in self.recursions:
        #     for _ in range(rec.get_times()):
        #         self.add_child(Node(self.data, self.children))


        print("  " * tab, self.data)
        for pointer in self.children:
            maybe_child = pointer.parcours()
            if maybe_child:
                maybe_child.expand(tab + 1)
            # if maybe_child == self:
                

            

        



class Pointer:
    def __init__(self, destination, times = -1):
        # print(times)
        self.times = times
        self.destination = destination

    def get_times(self):
        return self.times
    
    def get_destination(self):
        return self.destination
    
    def parcours(self):
        # If we haven't reached the recursion limit, return the destination node
        if self.times != 0:
            self.times -= 1
            return self.destination
        # print("NOT", self.destination.data, self.times)
        return None

class Recursion:
    def __init__(self, times):
        self.times = times

    def get_times(self):
        return self.times
    



root = Node("ROOT", [])
child = Node("CHILD", [])
child2 = Node("CHILD2", [])
root.add_child(root, 2)
root.add_child(root, 2)


root.expand(0)