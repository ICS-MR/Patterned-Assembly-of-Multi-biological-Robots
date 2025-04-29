# Creating classes to define three basic elements: microrobots, Potential wells(called "target" in code), and barriers.

class robot:


    def __init__(self,x,y,type):
        self.x = x
        self.y = y
        self.type = type


class target:

   # in original view,parent_robot = -1, parent_target = -1

    def __init__(self,x,y,type,parent_robot,parent_target):
        self.x = x
        self.y = y
        self.type = type
        self.parent_robot = parent_robot
        self.parent_target = parent_target


class barrier:


    def __init__(self,x,y,parent_robot):
        self.x = x
        self.y = y
        self.parent_robot = parent_robot


