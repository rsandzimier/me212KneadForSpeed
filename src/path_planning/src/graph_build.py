#!/usr/bin/env python

class Node():
    """
    Creating datatype to represent each point in the map as a node on the graph.
    """
    def __init__(self, x, y, theta = None, parent = None):
        self.state = (x, y, theta)
        self.parent = parent
        self.cost = 0

class Graph():
    def __init__(self, img):
        self.nodes = set()
        self.edges = dict()

    def build_graph(self, nodes):
        pass

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, start_node, end_node, weight):
        self.add_node(start_node)
        self.add_node(end_node)
        if start_node not in self.edges.keys():
            sn_edges = set()
            self.edges[start_node] = sn_edges
        self.edges[start_node].add((start_node, end_node, weight))

    def get_children(self, node):
        return self.edges[node]
