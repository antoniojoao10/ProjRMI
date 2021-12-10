import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from array import *
import numpy as np


CELLROWS = 7
CELLCOLS = 14
collLeft = 0
collRight = 0


class MyRob(CRobLinkAngs):

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.start = True
        self.initialPos = None
        self.currentPos = None
        self.nextPos = None
        self.previousPos = None
        self.registeredPos = set()
        self.rotating = 0
        self.node_connections = {}
        self.nodes_to_explore = set()
        self.traverse_finished = False
        self.rotations = 0
        self.cont = 0
        self.map = [[" " for i in range(55)] for j in range(27)]
        self.map_x = 28
        self.map_y = 14
        self.map[14][28] = "I"
        self.track_next = []
        self.track_next_start = True
        self.track_next_last = None

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not

    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        collLeft = False
        collRight = False

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def mapping(self, coord, direction):
        self.map_x = int(self.map_x)
        self.map_y = int(self.map_y)

        if self.currentPos != self.initialPos:
            if self.map[self.map_y][self.map_x] != " ":
                return
            else:
                self.map[self.map_y][self.map_x] = "X"

        for i, move in enumerate(coord):
            x, y = self.add_coordinates(
                (self.map_x, self.map_y), self.mult_coordinates(move, (0.5, -0.5)))
            x = int(x)
            y = int(y)
            if self.map[y][x] == " ":
                if self.add_coordinates(self.currentPos, move) in self.node_connections[self.currentPos]:
                    self.map[y][x] = "X"
                else:
                    if direction in ["up", "down"]:
                        if i in [0, 3]:
                            self.map[y][x] = "-"
                        else:
                            self.map[y][x] = "|"
                    else:
                        if i in [0, 3]:
                            self.map[y][x] = "|"
                        else:
                            self.map[y][x] = "-"

        with open(map_name, "w") as fout:
            tmp = ""
            for row in self.map:
                for line in row:
                    tmp += line
                tmp += "\n"
            fout.write(tmp)

    def wander(self):
        # if it's rotating, keep rotating
        if self.rotating == 1 or self.rotations > 0:
            self.rotating = self.rotateLeft()
            if self.rotating == 0:
                self.rotations -= 1
            if self.rotations > 0:
                self.driveMotors(-0.15, 0-15)
                self.rotating = 1
        elif self.rotating == 2:
            self.rotating = self.rotateRight()
        else:
            direction, coord = self.get_direction()

            front = self.measures.irSensor[0]
            left = self.measures.irSensor[1]
            right = self.measures.irSensor[2]
            back = self.measures.irSensor[3]

            if self.track_next:
                if self.track_next_start:
                    self.track_next_start = self.add_coordinates(
                        self.currentPos, coord[0]) != self.track_next_last
                    if self.track_next_start:
                        self.driveMotors(0.15, -0.15)
                        self.rotating = 2
                else:
                    self.traverse_path(
                        [front, left, right, back], direction, coord)
                return

            angle = self.measures.compass + 180
            if angle > 180:
                angle = angle - 360

            # while going forward make adjustment to the closest direction
            if angle in list(range(-179, -150)) + list(range(-89, -60)) + list(range(1, 30)) + list(range(91, 120)):
                self.driveMotors(0.01, -0.01)
            elif angle in list(range(150, 180)) + list(range(-120, -90)) + list(range(-30, 0)) + list(range(60, 90)):
                self.driveMotors(-0.01, 0.01)
            else:
                self.before_rotation = True
                if self.start:
                    self.start = False
                    self.initialPos = self.currentPos = (
                        self.measures.x, self.measures.y)

                    self.nodes_to_explore.add(
                        self.add_coordinates(self.currentPos, coord[3]))

                    self.add_connections(coord, self.currentPos, [
                        front, left, right, back])

                    self.nextPos = self.add_coordinates(
                        self.currentPos, coord[0])

                    self.mapping(coord, direction)

                    # self.print_details(
                    #    1, direction, [front, left, right, back])

                else:
                    if direction is "up" and self.measures.y - self.currentPos[1] >= 1.2\
                            or direction is "left" and self.currentPos[0] - self.measures.x >= 1.2\
                            or direction is "right" and self.measures.x - self.currentPos[0] >= 1.2\
                            or direction is "down" and self.currentPos[1] - self.measures.y >= 1.2:
                        self.add_connections(coord, self.nextPos, [
                            front, left, right, back])
                        self.add_nodes_to_explore(coord, self.nextPos, [
                            front, left, right, back])

                    if direction is "up" and self.measures.y - self.nextPos[1] >= 0\
                            or direction is "left" and self.nextPos[0] - self.measures.x >= 0\
                            or direction is "right" and self.measures.x - self.nextPos[0] >= 0\
                            or direction is "down" and self.nextPos[1] - self.measures.y >= 0:
                        if self.traverse_finished:
                            self.traverse_finished = False
                            return

                        self.update_env(
                            coord, 0, [front, left, right, back])

                        self.purge_connections()

                        # self.print_details(
                        #    2, direction, [front, left, right, back])

                        self.mapping(coord, direction)

                        if self.nextPos in self.node_connections:
                            self.track_next = [None] + \
                                self.get_path_to_next_node()[:-1]
                            self.nextPos = self.track_next_last = self.track_next.pop()

                            self.mapping(coord, direction)

                            # self.print_details(
                            #    4, direction, [front, left, right, back])
                            self.driveMotors(0, 0)
                            return

                        # if it's blocked while going forward, makes a turn
                        if front > 1:
                            if right > 1 and left < 1:
                                self.driveMotors(-0.15, 0.15)
                                self.rotating = 1
                            elif left > 1 and right < 1:
                                self.driveMotors(0.15, -0.15)
                                self.rotating = 2
                            elif left > 1 and right > 1:
                                self.driveMotors(-0.15, 0.15)
                                self.rotating = 1
                                self.rotations = 2
                            else:
                                self.driveMotors(-0.15, 0.15)
                                self.rotating = 1

                if self.rotating == 0:
                    self.driveMotors(0.15, 0.15)

                self.registeredPos.add(self.currentPos)

    # makes the robot go to the next node to continue exploration
    def traverse_path(self, sensors, direction, coord):
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if direction is "up" and self.measures.y - self.nextPos[1] >= 0\
                or direction is "left" and self.nextPos[0] - self.measures.x >= 0\
                or direction is "right" and self.measures.x - self.nextPos[0] >= 0\
                or direction is "down" and self.nextPos[1] - self.measures.y >= 0:
            self.previousPos = self.currentPos
            self.currentPos = self.nextPos
            self.nextPos = self.track_next.pop()

            front = sensors[0]
            left = sensors[1]
            right = sensors[2]
            back = sensors[3]

            if not self.track_next:
                if front > 1:
                    if right > 1 and left < 1:
                        self.nextPos = self.add_coordinates(
                            self.currentPos, coord[1])
                        self.driveMotors(-0.15, 0.15)
                        self.rotating = 1
                    elif left > 1 and right < 1:
                        self.nextPos = self.add_coordinates(
                            self.currentPos, coord[2])
                        self.driveMotors(-0.15, 0.15)
                        self.rotating = 2
                    elif right > 1 and left > 1:
                        self.nextPos = self.add_coordinates(
                            self.currentPos, coord[3])
                        self.driveMotors(-0.15, 0.15)
                        self.rotating = 1
                        self.rotations = 2
                    else:
                        self.nextPos = self.add_coordinates(
                            self.currentPos, coord[1])
                        self.driveMotors(-0.15, 0.15)
                        self.rotating = 1
                else:
                    self.nextPos = self.add_coordinates(
                        self.currentPos, coord[0])
                self.traverse_finished = True

            curr_coord = self.sub_coordinates(
                self.currentPos, self.previousPos)

            next_coord = self.sub_coordinates(
                self.nextPos, self.currentPos)

            self.map_x, self.map_y = self.add_coordinates(
                (self.map_x, self.map_y), self.mult_coordinates(curr_coord, (1, -1)))

            if next_coord == coord[1]:
                self.driveMotors(-0.15, 0.15)
                self.rotating = 1
            elif next_coord == coord[2]:
                self.driveMotors(0.15, -0.15)
                self.rotating = 2
            elif next_coord == coord[3]:
                self.driveMotors(-0.15, 0.15)
                self.rotating = 1
                self.rotations = 2

            # self.print_details(
            #    3, direction, [front, left, right, back])

        # while going forward make adjustment to the closest direction
        if angle in list(range(-179, -150)) + list(range(-89, -60)) + list(range(1, 30)) + list(range(91, 120)):
            self.driveMotors(0.01, -0.01)
        elif angle in list(range(150, 180)) + list(range(-120, -90)) + list(range(-30, 0)) + list(range(60, 90)):
            self.driveMotors(-0.01, 0.01)
        else:
            if self.rotating == 0:
                self.driveMotors(0.15, 0.15)

    # when it steps on a registered position, calculates the path to the nearest non-registered position
    def get_path_to_next_node(self):

        goal = self.get_node_to_explore()
        open_nodes = {Node(self.currentPos, None, 0, self.manhattan_distance(
            self.currentPos, goal)): self.manhattan_distance(self.currentPos, goal)}

        closed_nodes = {}

        while open_nodes:

            node = next(iter(open_nodes))
            open_nodes.pop(node)

            if node.pos == goal:
                print(node.get_path())
                return node.get_path()

            if closed_nodes.get(node.pos) == None:
                closed_nodes[node.pos] = node.g

            if node.pos in self.node_connections:
                for con in self.node_connections[node.pos]:
                    new_g = node.g + 1

                    if closed_nodes.get(con) != None:
                        if closed_nodes.get(con) <= new_g:
                            continue
                        else:
                            closed_nodes.pop(con)

                    new_node = Node(con, node, new_g,
                                    self.manhattan_distance(con, goal))

                    open_nodes[new_node] = new_node.f

            open_nodes = {k: v for k, v in sorted(
                open_nodes.items(), key=lambda item: item[1])}

    # aux function to get the nearest node of nodes_to_explore to the current position
    def get_node_to_explore(self):
        lst = list(self.nodes_to_explore)
        lst = [node for node in sorted(
            lst, key=lambda x: self.manhattan_distance(x, self.currentPos), reverse=True)]
        tmp = lst.pop()
        self.nodes_to_explore.remove(tmp)
        return tmp

    def manhattan_distance(self, begin, end):
        return abs(begin[0] - end[0]) + abs(begin[1] - end[1])

    # removes faulty connections
    def purge_connections(self):
        for key in self.node_connections:
            self.node_connections[key] = set([con for con in self.node_connections[key]
                                             if con in list(self.node_connections.keys()) +
                                              list(self.nodes_to_explore) +
                                              [self.nextPos]])

    # updates the robot environment variables
    def update_env(self, coord, direction, sensors):
        self.previousPos = self.currentPos
        self.currentPos = self.nextPos

        self.map_x, self.map_y = self.add_coordinates(
            (self.map_x, self.map_y), self.mult_coordinates(coord[direction], (1, -1)))

        if self.currentPos in self.nodes_to_explore:
            self.nodes_to_explore.remove(self.currentPos)

        front = sensors[0]
        left = sensors[1]
        right = sensors[2]
        back = sensors[3]
        if front > 1:
            if right > 1 and left < 1:
                self.nextPos = self.add_coordinates(
                    self.currentPos, coord[1])
            elif left > 1 and right < 1:
                self.nextPos = self.add_coordinates(
                    self.currentPos, coord[2])
            elif right > 1 and left > 1:
                self.nextPos = self.add_coordinates(
                    self.currentPos, coord[3])
            else:
                self.nextPos = self.add_coordinates(
                    self.currentPos, coord[1])
        else:
            self.nextPos = self.add_coordinates(
                self.currentPos, coord[0])

        self.nodes_to_explore = set(
            [node for node in self.nodes_to_explore if node not in self.node_connections and node != self.nextPos])

    # prints detailed information on the current state of the robot
    def print_details(self, num, direction, sensors):
        self.cont += 1
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        print("\n\nUpdate No " + str(self.cont) + "\t", str(num))
        print("Initial Pos\t", self.initialPos)
        print("(x,y)\t\t", (self.map_x, self.map_y))
        print("Previous Pos\t", self.previousPos)
        print("Current Pos\t", self.currentPos)
        print("GPS\t\t", self.measures.x, self.measures.y)
        print("Next Pos\t", self.nextPos)
        print("Direction\t", direction)
        print("Angle\t\t", angle)
        print("Sensors\t\t", sensors)
        #print("Exploration\t", self.nodes_to_explore)
        #print("Connections\t", self.node_connections)
        print("---------------------------\n")

    # while going forward adds nodes that have not been yet visited
    def add_nodes_to_explore(self, coord, pos, sensors):
        front = sensors[0]
        left = sensors[1]
        right = sensors[2]
        back = sensors[3]

        if pos in self.registeredPos:
            return

        if left < 1:
            self.nodes_to_explore.add(
                self.add_coordinates(pos, coord[1]))
        if right < 1:
            self.nodes_to_explore.add(
                self.add_coordinates(pos, coord[2]))

    # stores adjacents nodes to the current node that are not blocked by walls
    def add_connections(self, coord, pos, sensors):

        if pos not in self.node_connections:
            self.node_connections[pos] = set()

        for i in range(len(sensors)):
            if sensors[i] < 1:
                self.node_connections[pos].add(
                    self.add_coordinates(pos, coord[i]))

    # return middle between two points
    def get_middle_point(self, a, b):
        return ((a[0]+b[0])/2, (a[1]+b[1])/2)

    # returns the direction the robot is facing and a tuple with the coordinates to get to adjacent nodes (front,left,right,back)
    def get_direction(self):
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if angle in [-91, -90, -89]:
            return "up", [(0, 2), (-2, 0), (2, 0), (0, -2)]
        elif angle in [-1, 0, 1]:
            return "left", [(-2, 0), (0, -2), (0, 2), (2, 0)]
        elif angle in [179, 180, -179]:
            return "right", [(2, 0), (0, 2), (0, -2), (-2, 0)]
        elif angle in [89, 90, 91]:
            return "down", [(0, -2), (2, 0), (-2, 0), (0, 2)]
        return "None", []

    # subs tuples with 2 values
    def sub_coordinates(self, a, b):
        return (a[0]-b[0], a[1]-b[1])

    # adds tuples with 2 variables
    def add_coordinates(self, a, b):
        return (a[0]+b[0], a[1]+b[1])

    # multiplies tuples with 2 variables
    def mult_coordinates(self, a, b):
        return (a[0]*b[0], a[1]*b[1])

    def rotateLeft(self):
        rotate = False
        slow_rotation_angle = 30

        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if angle not in [-1, 0, 1]+[89, 90, 91]+[179, 180, -179]+[-91, -90, -89]:
            rotate = True

        if rotate:
            if 0 < 0 - angle < slow_rotation_angle or 0 < -90 - angle < slow_rotation_angle or\
                    0 < 180 - angle < slow_rotation_angle or 0 < 90 - angle < slow_rotation_angle:
                # print("Rotating slowly left")
                self.driveMotors(-0.01, 0.01)
            else:
                self.driveMotors(-0.05, 0.05)
            return 1

        return 0

    def rotateRight(self):
        rotate = False
        slow_rotation_angle = 30

        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if angle not in [-1, 0, 1]+[89, 90, 91]+[179, 180, -179]+[-91, -90, -89]:
            rotate = True

        if rotate:
            if 0 < angle - 0 < slow_rotation_angle or 0 < angle - -90 < slow_rotation_angle or\
                    0 < angle - -180 < slow_rotation_angle or 0 < angle - 90 < slow_rotation_angle:
                # print("Rotating slowly right")
                self.driveMotors(0.01, -0.01)
            else:
                self.driveMotors(0.05, -0.05)
            return 2

        return 0


class Node():
    def __init__(self, currentPos, parent, g, h):
        self.pos = currentPos
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    def __str__(self):
        return "["+str(self.pos) + ", [" + str(self.parent) + "]"

    def get_path(self):
        path = [self.pos]
        while self.parent != None:
            self = self.parent
            path.append(self.pos)
        if None in path:
            path.remove(None)
        return path


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c+1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c+1)//3*2-1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c//3*2] = '-'
                        else:
                            None

            i = i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
map_name = "mapping.out"

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        map_name = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
