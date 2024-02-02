class Node:
    def __init__(self, data, children, recursions):
        self.data = data
        self.children = children
        self.recursions = recursions
    
    def add_child(self, other, times = -1):
        # -1 if infinite (only for non-recursive nodes)
        # otherwise times is for the number of times this child should be recursed on
        self.children.append(other)
        self.recursions.append(times)

    def debug(self):
        print(self)
        for pointer, limit in zip(self.children, self.recursions):
            print(pointer, limit)


    def copy(self):
        return Node(self.data, self.children[:], self.recursions[:])

    def expand(self, tab):
        # # First, expand the recursive nodes:
        # for rec in self.recursions:
        #     for _ in range(rec.get_times()):
        #         self.add_child(Node(self.data, self.children))

        new_children = self.children[:]

        self.debug()
        # Replace all selfs with copies of self with the recursion level down 1
        for i in range(len(self.children)):
            if self.recursions[i] > 0:
                if self.children[i] == self: # Maybe not necessary?
                    child = self.copy()

                    # Replace all references to self with references to the new child
                    for j in range(len(child.children)):
                        if child.children[j] == self:
                            child.children[j] == child
                            child.recursions[j] -= 1

                    new_children[i] = child

        self.children = new_children


        self.debug()
        
        # Find the first child with a recursion left
        print(tab * "  " + self.data) # Visted
        for i in range(len(self.children)):
            if self.recursions[i] != 0:
                self.recursions[i] -= 1
                self.children[i].expand(tab + 1)

        # for i in range(len(self.children)):
        #     print("  " * tab, self.data)
        #     if self.recursions[i] > 0:
        #         if self.children[i] == self:
        #             child = self.copy()
        #             for grandchild_index in range(len(child.children)):
        #                 if child.children[grandchild_index] == self:
        #                     child.recursions[grandchild_index] -= 1 #MAYBE ALL SELFS?
        #                     child.children[grandchild_index] = child.copy()
        #             self.children[i] = child
        #         else:
        #             child = self.children[i]

                # self.debug()
                # # child.debug()
                # child.expand(tab + 1)



root = Node("ROOT", [], [])
child = Node("CHILD", [], [])
child2 = Node("CHILD2", [], [])
root.add_child(root, 2)
root.add_child(root, 2)


root.expand(0)