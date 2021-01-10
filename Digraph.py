#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 15:53:55 2020

@author: chuckschultz
"""

class Node(object):
    def __init__(self, name):
        self.name = name
    def getName(self):
        return self.name
    def __str__(self):
        return self.name

class Edge(object):
    def __init__(self, src, dest):
        self.src = src
        self.dest = dest
    def getSource(self):
        return self.src
    def getDestination(self):
        return self.dest
    def __str__(self):
        return self.src.getName() + '->' + self.dest.getName()

class WeightedEdge(Edge):
    def __init__(self, src, dest, weight):
        Edge.__init__(self, src, dest)
        self.weight = weight
    def getWeight(self):
        return self.weight
    def __str__(self):
        return Edge.__str__(self) + ' (' + str(self.weight) + ')'

class Digraph(object):
    def __init__(self):
        self.edges = {}
    def addNode(self, node):
        if node in self.edges:
            raise ValueError('Duplicate node')
        else:
            self.edges[node] = []
    def addEdge(self, edge):
        src = edge.getSource()
        dest = edge.getDestination()
        if not(src in self.edges and dest in self.edges):
            raise ValueError('Node not in graph')
        self.edges[src].append(dest)
    def childrenOf(self, node):
        return self.edges[node]
    def hasNode(self, node):
        return node in self.edges
    def getNode(self, name):
        for n in self.edges:
            if n.getName() == name:
                return n
        raise NameError(name)
    def __str__(self):
        result = ''
        for src in self.edges:
            for dest in self.edges[src]:
                result = result + src.getName() + '->' + dest.getName() + '\n'
        return result[:-1]

class Graph(Digraph):
    def addEdge(self, edge):
        Digraph.addEdge(self, edge)
        rev = Edge(edge.getDestination(), edge.getSource())
        Digraph.addEdge(self, rev)

def buildCityGraph(graphType):
    g = graphType()
    for name in ('Boston', 'Providence', 'New York', 'Chicago', 'Denver',
                 'Phoenix', 'Los Angeles'):
        g.addNode(Node(name))
    g.addEdge(Edge(g.getNode('Boston'), g.getNode('Providence')))
    g.addEdge(Edge(g.getNode('Boston'), g.getNode('New York')))
    g.addEdge(Edge(g.getNode('Providence'), g.getNode('Boston')))
    g.addEdge(Edge(g.getNode('Providence'), g.getNode('New York')))
    g.addEdge(Edge(g.getNode('New York'), g.getNode('Chicago')))
    g.addEdge(Edge(g.getNode('Chicago'), g.getNode('Denver')))    
    g.addEdge(Edge(g.getNode('Denver'), g.getNode('Phoenix')))
    g.addEdge(Edge(g.getNode('Denver'), g.getNode('New York')))
    g.addEdge(Edge(g.getNode('Los Angeles'), g.getNode('Boston')))
    return g

#print(buildCityGraph(Digraph))
nodes = []
nodes.append("0") # nodes[0]
nodes.append("1") # nodes[1]
nodes.append("2") # nodes[2]
nodes.append("3") # nodes[3]
nodes.append("4") # nodes[4]
nodes.append("5") # nodes[5]



def buildStudentGraph(graphType):
    g = graphType()
    
    for node in nodes:
        g.addNode(Node(node))
#        print(node)
    g.addEdge(Edge(g.getNode(nodes[0]), g.getNode(nodes[1])))
    g.addEdge(Edge(g.getNode(nodes[1]), g.getNode(nodes[2])))
    g.addEdge(Edge(g.getNode(nodes[2]), g.getNode(nodes[3])))
    g.addEdge(Edge(g.getNode(nodes[2]), g.getNode(nodes[4])))
    g.addEdge(Edge(g.getNode(nodes[3]), g.getNode(nodes[4])))
    g.addEdge(Edge(g.getNode(nodes[3]), g.getNode(nodes[5])))
    g.addEdge(Edge(g.getNode(nodes[0]), g.getNode(nodes[2])))
    g.addEdge(Edge(g.getNode(nodes[1]), g.getNode(nodes[0])))
    g.addEdge(Edge(g.getNode(nodes[3]), g.getNode(nodes[1])))
    g.addEdge(Edge(g.getNode(nodes[4]), g.getNode(nodes[0])))
    
    return g


#for n in nodes:
#    g.addNode(n)
#
#for node1 in nodes:
#    for node2 in nodes:
#        if node1 != node2 and (str(node1)[0] == str(node2)[0] or\
#                               str(node1)[2]\
#                               == str(node2)[2]):
#            g.addEdge(Edge(node1, node2))
#
#
#print(g)

def printPath(path):
    '''Assumes path is a list of nodes'''
    result = ''
    for i in range(len(path)):
        result = result + str(path[i])
        if i != len(path) - 1:
            result = result + '->'
    return result

def DFS(graph, start, end, path, shortest, toPrint = False):
    path = path + [start]
    if toPrint:
        print('Current DFS path:', printPath(path))
    if start == end:
        return path
    for node in graph.childrenOf(start):
        if node not in path:
            if shortest == None or len(path) < len(shortest):
                newPath = DFS(graph, node, end, path, shortest, toPrint)
                if newPath != None:
                    shortest = newPath
        elif toPrint:
            print('Already visited', node)
    return shortest

def shortestPathD(graph, start, end, toPrint = False):
    '''Assumes graph is a Digraph; start and end are nodes
    Returns a shortest path from start to end in graph'''
    return DFS(graph, start, end, [], None, toPrint)

def testSP(source, destination):
    g = buildStudentGraph(Digraph)
    sp = shortestPathD(g, g.getNode(source), g.getNode(destination),
                      toPrint = True)
    if sp != None:
        print('Shortest path from', source, 'to', destination, 'is',
              printPath(sp))
    else:
        print('There is no path from', source, 'to', destination)
    sp = BFS(g, g.getNode(source), g.getNode(destination), toPrint=True)
    print('Shortest path found by BFS:', printPath(sp))


#testSP('Chicago', 'Boston')
#testSP('Boston', 'Phoenix')

def BFS(graph, start, end, toPrint = False):
    initPath = [start]
    pathQueue = [initPath]
    while len(pathQueue) != 0:
        # Get and remove oldest element in pathQueue
        tmpPath = pathQueue.pop(0)
        if toPrint:
            print('Current BFS path:', printPath(tmpPath))
        lastNode = tmpPath[-1]
        if lastNode == end:
            return tmpPath
        for nextNode in graph.childrenOf(lastNode):
            if nextNode not in tmpPath:
                newPath = tmpPath + [nextNode]
                pathQueue.append(newPath)
    return None

def shortestPath(graph, start, end, toPrint = False):
    '''Assumes graph is a Digraph; start and end are nodes
    Returns a shortest path from start to end in graph'''
    return BFS(graph, start, end, toPrint)

testSP(nodes[0], nodes[5])






















