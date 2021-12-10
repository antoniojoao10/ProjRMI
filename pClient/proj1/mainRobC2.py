
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from array import *
import numpy as np


CELLROWS=7
CELLCOLS=14
collLeft=0
collRight=0

class MyRob(CRobLinkAngs):
    
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.start = True
        self.initialPos = None
        self.currentPos = None
        self.registeredPos = []

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

        collLeft=False
        collRight=False

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
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
    
    def mapping(self):
        open('mapping.out', 'w').close()
        rows, cols = (27, 56)
        arr = [[" " for i in range(cols)] for j in range(rows)]
        
        for i in self.registeredPos:
            lin = 23 - (i[1]-self.initialPos[1] + 10) 
            col = i[0]-self.initialPos[0] + 2 + 25
            #print(str(lin) + "\t" + str(col))
            if 26<=lin<=10 or 54<=col<=24:
                continue
            if arr[lin][col] == " " or arr[lin][col]!="I":
                arr[lin][col]=i[2]

        fout= open("mapping.out","w")
        tmp = ""
        for row in arr:
            for line in row:
                tmp += line
            tmp+="\n"
        fout.write(tmp)
        fout.close()

    def wander(self):
        x = floor(self.measures.x)
        y = floor(self.measures.y)

        if self.start:
            self.start = False
            self.initialPos = (x,y)
            self.currentPos = (x,y,"I")
            self.registeredPos.append(self.currentPos)

        if (x != self.currentPos[0] or y != self.currentPos[1]):
            if 135 > self.measures.compass > 45 or -135 < self.measures.compass < -45:
                self.currentPos = (x-1,y,"X")
                if  self.currentPos not in self.registeredPos:
                    self.currentPos = (x-1,y,"|")
                    self.registeredPos.append(self.currentPos)
                self.currentPos = (x+1,y,"X")
                if  self.currentPos not in self.registeredPos:
                    self.currentPos = (x+1,y,"|")
                    self.registeredPos.append(self.currentPos)
            elif -45 < self.measures.compass < 45 or (-135 > self.measures.compass > -180 and 180 < self.measures.compass < 135):
                self.currentPos = (x,y+1,"X")
                if  self.currentPos not in self.registeredPos:
                    self.currentPos = (x,y+1,"-")
                    self.registeredPos.append(self.currentPos)
                self.currentPos = (x,y-1,"X")
                if  self.currentPos not in self.registeredPos:
                    self.currentPos = (x,y-1,"-")
                    self.registeredPos.append(self.currentPos)
            self.currentPos = (x,y,"X")
            if  self.currentPos not in self.registeredPos and (x,y,"I") not in self.registeredPos:
                self.registeredPos.append(self.currentPos)
    
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        self.mapping()
        
        global collLeft
        global collRight
        
        if collLeft>3:
            self.driveMotors(0.15,0.05)
            collLeft=0
        elif collRight>3:
            self.driveMotors(0.05,0.15)
            collRight=0
            #print(str(list(sorted(self.registeredPos,key=lambda k: [k[0],k[1]]))))
        elif (self.measures.collision and self.measures.irSensor[right_id] < self.measures.irSensor[left_id] and collRight==0) or \
            (self.measures.irSensor[center_id] > 4 and self.measures.irSensor[right_id] < self.measures.irSensor[left_id] and collRight==0) or collLeft!=0:
            self.driveMotors(-0.05,-0.15)
            collLeft +=1
            print("collision left")
        elif (self.measures.collision and self.measures.irSensor[right_id] > self.measures.irSensor[left_id]) or\
            (self.measures.irSensor[center_id] > 4 and self.measures.irSensor[right_id] > self.measures.irSensor[left_id] or collRight!=0):
            self.driveMotors(-0.15,-0.05)
            collRight +=1
            print("collision right")
        elif self.measures.irSensor[center_id] > 10:
            self.driveMotors(-0.15,-0.15)
        elif self.measures.irSensor[right_id] > 10:
            self.driveMotors(-0.15,0.15)
            print('Left')
        elif self.measures.irSensor[left_id] > 10:
            self.driveMotors(0.15,-0.15)
            print('Right')
        elif self.measures.irSensor[center_id] > 2.5 and self.measures.irSensor[left_id] > 2.5:
            self.driveMotors(0.15,-0.15)
            print('Right corner')
        elif self.measures.irSensor[center_id] > 2.5 and self.measures.irSensor[right_id] > 2.5:
            self.driveMotors(-0.15,0.15)
            print('Left corner')
        elif self.measures.irSensor[right_id] < 4 and self.measures.irSensor[right_id] < self.measures.irSensor[left_id]:
            self.driveMotors(0.15,0)
            print('Right back')
        elif self.measures.irSensor[left_id] < 4 and self.measures.irSensor[right_id] > self.measures.irSensor[left_id]:
            self.driveMotors(0,0.15)
            print('Left back')
        elif self.measures.irSensor[center_id] < 3:
            self.driveMotors(0.15,0.15)
            collLeft=False
            collRight=False
            print('Front')
        else:
            if self.measures.irSensor[right_id] > self.measures.irSensor[left_id]:
                self.driveMotors(-0.15,0.15)
            elif self.measures.irSensor[left_id] > self.measures.irSensor[right_id]:
                self.driveMotors(0.15,-0.15)
            print('Back')
    
         
    
class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()