class RecNode:
    def __init__(self, data, rec_times, next):
        self.rec_times = rec_times
        self.next = next
        self.data = data

    def expand(self):
        base = Node(self.data, [])
        node = base
        for i in range(self.rec_times):
            child = Node(self.data, [])
            node.add_node(child)
            node = child
        return base
            


class Node:
    def __init__(self, data, next):
        self.next = next
        self.data = data

    def add_node(self, other):
        self.next.append(other)

    def get_children(self):
        return self.next
    
    def to_string(self, tab):
        string = "\t" * tab + self.data + "\n"
        for child in self.next:
            string += child.to_string(tab + 1)
        return string
    
    def debug(self):
        print(self)
        print(self.data)
        print(self.get_children())
    
    def __str__(self):
        return self.to_string(0)
    


a = RecNode("FIRST", 6, [])



b = a.expand()

print(b)