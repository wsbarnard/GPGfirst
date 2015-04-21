#!/usr/bin/env python

#Class definition for coordinate point
class Coord:
    def __init__(self, x, y):
        self.x = x 
        self.y = y 

#Class definition for Node 
class Node:
    def __init__(self, x, y, h, chance): #removed neighbors.
        self.x = x
        self.y = y
        self.h = h
        self.chance = chance
        self.cost = 0
        self.est = 0
        self.parent = None

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            if(self.x == other.x) and (self.y == other.y):
                return True
            else:
                return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

