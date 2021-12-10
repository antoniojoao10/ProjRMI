
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
        self.nextPos = None
        self.registeredPos = []
        self.rotating = 0
        self.node_connections = {}
        self.nodes_to_explore = set()
        self.d_counter = 0
        self.finished_rotation = False

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

    def mapping(self):
        open('mapping.out', 'w').close()
        rows, cols = (27, 56)
        arr = [[" " for i in range(cols)] for j in range(rows)]

        for i in self.registeredPos:
            lin = 23 - (i[1]-self.initialPos[1] + 10)
            col = i[0]-self.initialPos[0] + 2 + 25
            # print(str(lin) + "\t" + str(col))
            if 26 <= lin <= 10 or 54 <= col <= 24:
                continue
            if arr[lin][col] == " " or arr[lin][col] != "I":
                arr[lin][col] = i[2]

        fout = open("mapping.out", "w")
        tmp = ""
        for row in arr:
            for line in row:
                tmp += line
            tmp += "\n"
        fout.write(tmp)
        fout.close()

    def wander(self):
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        #currentPos = (floor(self.measures.x), floor(self.measures.y))

        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        # if it's rotating, keep rotating
        if self.rotating == 1:
            self.rotating = self.rotateLeft()
            self.finished_rotation = True
        elif self.rotating == 2:
            self.rotating = self.rotateRight()
            self.finished_rotation = True
        else:
            # if it's blocked while going forward, makes a turn
            if front > 2:
                if right > 1:
                    #print("Left rotation")
                    self.driveMotors(-0.01, 0.01)
                    self.rotating = self.rotateLeft()
                elif left > 1:
                    #print("Right rotation")
                    self.driveMotors(0.01, -0.01)
                    self.rotating = self.rotateRight()
                else:
                    self.driveMotors(-0.01, 0.01)
                    self.rotating = self.rotateLeft()
            else:
                # while going forward make adjustment to the closest direction
                if angle in list(range(-179, -150)) + list(range(-89, -60)) + list(range(1, 30)) + list(range(91, 120)):
                    # print("right adjustment")
                    self.driveMotors(0.01, -0.01)
                elif angle in list(range(150, 180)) + list(range(-120, -90)) + list(range(-30, 0)) + list(range(60, 90)):
                    # print("left adjustment")
                    self.driveMotors(-0.01, 0.01)
                else:
                    if self.start:
                        self.start = False

                        self.initialPos = (0, 0)

                        direction, coord = self.get_direction()
                        self.nextPos = self.add_coordinates(
                            self.initialPos, coord)

                        if back < 1:
                            if direction == "up":
                                self.nodes_to_explore.add(
                                    self.add_coordinates(self.initialPos, (0, -1)))
                            elif direction == "left":
                                self.nodes_to_explore.add(
                                    self.add_coordinates(self.initialPos, (1, 0)))
                            elif direction == "right":
                                self.nodes_to_explore.add(
                                    self.add_coordinates(self.initialPos, (-1, 0)))
                            elif direction == "down":
                                self.nodes_to_explore.add(
                                    self.add_coordinates(self.initialPos, (0, 1)))

                        self.add_connections(direction, self.initialPos)
                        self.add_nodes_to_explore(direction, self.initialPos)
                        self.registeredPos.append(self.initialPos)
                        print("\n\nConnections", self.node_connections)
                        print("\nExploration", self.nodes_to_explore)
                    else:

                        direction, coord = self.get_direction()

                        self.driveMotors(0.1, 0.1)

                        # when the robot "walks" 2 diameters a.k.a changes node
                        if self.d_counter == 20:

                            currentPos = self.nextPos
                            #print("\n\n", currentPos)
                            self.d_counter = 0
                            self.add_connections(direction, currentPos)
                            # print("\n\n", currentPos, "\nConnections",
                            # self.node_connections)

                            self.add_nodes_to_explore(
                                direction, currentPos)
                            #print("\nExploration", self.nodes_to_explore)

                            if front < 1 and not self.finished_rotation:
                                self.nextPos = self.add_coordinates(
                                    currentPos, coord)

                            if currentPos not in self.registeredPos:
                                self.registeredPos.append(currentPos)

                        if self.finished_rotation:
                            self.finished_rotation = False
                            self.nextPos = self.add_coordinates(
                                self.nextPos, coord)

                        print(self.nextPos)
                        print("--------", front)

                        # when the robot "walks" 1 diameter
                        if self.d_counter == 10:
                            pass

                    self.d_counter += 1
                    # print(self.d_counter)

    # while going forward adds place that have not been yet visited

    def add_nodes_to_explore(self, direction, currentPos):
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        if currentPos in self.registeredPos:
            return

        if front > 1:
            if direction == "up":
                if right < 1 and left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (1, 0)))
            elif direction == "left":
                if right < 1 and left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, 1)))
            elif direction == "right":
                if right < 1 and left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, -1)))
            elif direction == "down":
                if right < 1 and left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (-1, 0)))
        else:
            if direction == "up":
                if left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (-1, 0)))
                if right < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (1, 0)))
            elif direction == "left":
                if left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, -1)))
                if right < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, 1)))
            elif direction == "right":
                if left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, 1)))
                if right < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (0, -1)))
            elif direction == "down":
                if left < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (1, 0)))
                if right < 1:
                    self.nodes_to_explore.add(
                        self.add_coordinates(currentPos, (-1, 0)))

    # stores adjacents nodes to the current node that are not blocked by walls
    def add_connections(self, direction, currentPos):

        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        if currentPos in self.node_connections:
            return
        else:
            self.node_connections[currentPos] = []

        if direction == "up":
            if front < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, 1))]
            if left < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (-1, 0))]
            if right < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (1, 0))]
            if back < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, -1))]
        elif direction == "left":
            if front < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (-1, 0))]
            if left < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, -1))]
            if right < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, 1))]
            if back < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (1, 0))]
        elif direction == "right":
            if front < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (1, 0))]
            if left < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, 1))]
            if right < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, -1))]
            if back < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (-1, 0))]
        elif direction == "down":
            if front < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, -1))]
            if left < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (1, 0))]
            if right < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (-1, 0))]
            if back < 1:
                self.node_connections[currentPos] += [
                    self.add_coordinates(currentPos, (0, 1))]

    # returns the direction the robot is facing and a tuple with the coordinates to get to the next node

    def get_direction(self):
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if angle in [-1, 0, 1]:
            return "left", (-1, 0)
        elif angle in [89, 90, 91]:
            return "down", (0, -1)
        elif angle in [179, 180, -179]:
            return "right", (1, 0)
        elif angle in [-91, -90, -89]:
            return "up", (0, 1)
        return 0

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
                #print("Rotating slowly left")
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
                #print("Rotating slowly right")
                self.driveMotors(0.01, -0.01)
            else:
                self.driveMotors(0.05, -0.05)
            return 2

        return 0


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

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
