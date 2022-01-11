
from enum import Flag
import sys
from tree_search import *
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.cnt = 0 #moving cnt
        self.moving = False #está se a mover
        self.rotating = 0
        self.currentPos=[0,0]
        rows, cols = (27, 54)
        self.map = [[" " for i in range(cols)] for j in range(rows)]
        self.mapping(True) #posição inicial
        self.prevRotate = False #se rodou anteriormente
        self.finished_rotation = False
        self.rotCnt = 0 #contação de rotações
        self.initAngle = None #primeiro angulo antes da rotação
        self.searchU = False #começar a procura do U
        self.moves = [] # todos os movimentos no mapa
        self.alreadyMoved = [] # posições já visitadaas na procura de U
        self.alreadyVisited = []
        self.Ulocation = [] #todas as localizações de U no mapa
        self.path = [] #path até U
        self.prev = [] #posição previa
        self.tree = None #arvore de pesquisa
        self.startPath = True
        self.overMotor = False # se na deslocação para U se movimentar demasiado para a frente, não tendo espaço para virar para algum lado 

        #tentar corrigir a erros de sensores nas posições de U
        self.fixMissU = 0 

        self.test = True            


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

    def wander(self):
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        """import numpy as np
        print(np.matrix(self.map))"""

        direction, coord = self.get_direction()

        
        if self.moving:
            if self.centerDirection():
                self.move() # movimento aleatorio
        elif self.rotating == 1:
            self.rotating = self.rotateLeft()
            self.prevRotate = True
            self.finished_rotation = True
        elif self.rotating == 2:
            self.rotating = self.rotateRight()
            self.prevRotate = True
            self.finished_rotation = True
        elif self.rotating == 3:
            self.rotating = self.rotate180()
        elif self.searchU:
            if self.centerDirection():
                self.moveUnknown() #moviemnto até posição U
        else:
            if self.centerDirection():
                """if self.test:
                    self.move()
                    self.moving = True
                    """
                print(self.fixMissU)
                print(left)
                print(right)
                print(front)
                print(self.prevRotate)
                
                if front <= 1.5:
                    tmp = self.currentPos.copy()
                    if direction == "right":
                        tmp[1] = tmp[1] + 1
                    elif direction == "left":
                        tmp[1] = tmp[1] - 1
                    elif direction == "up":
                        tmp[0] = tmp[0] - 1
                    elif direction == "down":
                        tmp[0] = tmp[0] + 1

                    if tmp in self.alreadyVisited:
                        self.Ulocation = []
                        print("khakdhfbdsakhfbasdlhfbasldhfbadlkfhbdafklhb")
                        self.rotating = self.rotate180()
                    else:
                        self.move()
                        self.moving = True 
                        self.prevRotate = False
                elif left <= 1.2 and self.prevRotate == False:
                    self.driveMotors(-0.1,0.1)
                    self.rotating = self.rotateLeft()
                elif right <= 1.2 and self.prevRotate == False:
                    self.driveMotors(0.1,-0.1)
                    self.rotating = self.rotateRight()
                else: #ir para uma posição não explorada do mapa
                    self.Ulocation = []
                    self.rotating = self.rotate180()

    
    def move(self):
        #mover exatamente 1 telha para a frente desde o ponto da anterior
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        num = 10
        #if self.finished_rotation: num = 20
   
        direction, coord = self.get_direction()

        #if self.cnt % 5 == 0: self.driveMotors(0.09,0.09) 
        if self.cnt < num :
            self.driveMotors(0.115,0.115)   
        if self.cnt < num  and (direction == "down"):
            if self.cnt % 5 == 0: self.driveMotors(0.099,0.099)  
            else: self.driveMotors(0.101,0.101) 
        if self.cnt < num  and (direction == "up"):
            self.driveMotors(0.116,0.116)  
        if self.cnt == num or front > 2:
            if direction == None : exit()
            print(direction)
            self.driveMotors(0,0)  
            self.moving = False 
            self.test = False
            self.cnt = 0
            self.finished_rotation = False

            if direction == "right":
                self.currentPos[1] = self.currentPos[1] + 1
            elif direction == "left":
                self.currentPos[1] = self.currentPos[1] - 1
            elif direction == "up":
                self.currentPos[0] = self.currentPos[0] - 1
            elif direction == "down":
                self.currentPos[0] = self.currentPos[0] + 1
            
            self.fixMissU +=1
            self.mapping()

            print(self.currentPos)
            #print(self.alreadyVisited)
            x = self.currentPos[0]
            y = self.currentPos[1]
            self.alreadyVisited.append([x,y])
            
            if self.fixMissU % 2 == 0:
                if direction == "right":
                    if left < 1.2: self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2: self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    if front < 1.2: self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2: self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                if direction == "left":
                    if left < 1.2: self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2: self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    if front < 1.2: self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2: self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                if direction == "up":
                    if left < 1.6: self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.6: self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    if front < 1.2: self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2: self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                if direction == "down":
                    if left < 1.6: self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.6: self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    if front < 1.2: self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2: self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
        
        """elif self.cnt == 10 :
            print(direction)
            if direction == "right":
                self.currentPos[1] = self.currentPos[1] + 1
            elif direction == "left":
                self.currentPos[1] = self.currentPos[1] - 1
            elif direction == "up":
                self.currentPos[0] = self.currentPos[0] - 1
            elif direction == "down":
                self.currentPos[0] = self.currentPos[0] + 1
            self.mapping()"""

        self.cnt += 1
    
    def moveUnknown(self): #movimento até posição U
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        def searchU(self, x, y):
            current = [x,y]
            up = x - 1
            down = x + 1
            right = y + 1
            left = y - 1
            self.alreadyMoved.append(current)

            #ver map e procurar X à volta e enviar para search tree
            if self.map[x][right] == "X" and [x,right] not in self.alreadyMoved:
                self.moves.append([current, [x,right], "right" ])
                searchU(self,x,right)
            elif self.map[x][right] == "U": 
                self.moves.append([current, [x,right], "right" ])
                self.Ulocation.append([x,right])
            if self.map[x][left] == "X" and [x,left] not in self.alreadyMoved: 
                self.moves.append([current, [x,left], "left"])
                searchU(self,x,left)
            elif self.map[x][left] == "U": 
                self.moves.append([current, [x,left], "left"])
                self.Ulocation.append([x,left])
            if self.map[up][y] == "X" and [up,y] not in self.alreadyMoved:
                self.moves.append([current, [up,y], "up"])
                searchU(self,up,y)
            elif self.map[up][y] == "U": 
                self.moves.append([current, [up,y], "up"])
                self.Ulocation.append([up,y])
            if self.map[down][y] == "X" and [down,y] not in self.alreadyMoved: 
                self.moves.append([current, [down,y], "down"])
                searchU(self,down,y)
            elif self.map[down][y] == "U": 
                self.moves.append([current, [down,y], "down"])
                self.Ulocation.append([down,y])

        if self.startPath == True and self.path == []:
            
            searchU(self, self.currentPos[0], self.currentPos[1])
            print("Looking")
            self.tree = Unknown(self.moves)
            p = SearchProblem(self.tree,self.currentPos,self.Ulocation.pop(0))
            t = SearchTree(p,'uniform')
            self.path = t.search()
            self.prevRotate = False
            print(self.path)
            print(self.moves)
        
        elif self.prevRotate:
            self.move()
            self.moving = True
            self.prevRotate = False

        elif self.path != []:
        #ver map e procurar U à volta e mover se encontrar 
            if self.startPath: 
                self.prev = self.path.pop(0)
            direction, coord = self.get_direction()

            self.startPath = False
            actual = self.path.pop(0)

            cost = self.tree.cost(0,[self.prev,actual])
            
            if cost == "right":
                if direction == "right":
                    self.move()
                    self.moving = True
                elif direction == "up":
                    if right > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(0.1,-0.1)
                        self.rotating = self.rotateRight()
                        self.prevRotate = True
                elif direction == "down":
                    if left > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(-0.1,0.1)
                        self.rotating = self.rotateLeft()
                        self.prevRotate = True
                elif direction == "left":
                    self.overMotor = True
                    self.rotate180()
            elif cost == "left":
                if direction == "left":
                    self.move()
                    self.moving = True
                elif direction == "up":
                    if left > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(-0.1,0.1)
                        self.rotating = self.rotateLeft()
                        self.prevRotate = True
                elif direction == "down":
                    if right > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(0.1,-0.1)
                        self.rotating = self.rotateRight()
                        self.prevRotate = True
                elif direction == "right":
                    self.overMotor = True
                    self.rotate180()
            elif cost == "up":
                if direction == "up":
                    self.move()
                    self.moving = True
                elif direction == "left":
                    if left > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(0.1,-0.1)
                        self.rotating = self.rotateRight()
                        self.prevRotate = True
                elif direction == "right":
                    if left > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(-0.1,0.1)
                        self.rotating = self.rotateLeft()
                        self.prevRotate = True
                elif direction == "down":
                    self.overMotor = True
                    self.rotate180()
            elif cost == "down":
                if direction == "down":
                    self.move()
                    self.moving = True
                elif direction == "left":
                    if left > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(-0.1,0.1)
                        self.rotating = self.rotateLeft()
                        self.prevRotate = True
                elif direction == "right":
                    if right > 0.7:
                        self.overMotor = True
                        self.driveMotors(-0.15,-0.15)
                        self.path.insert(0,actual)
                    else:
                        self.driveMotors(0.1,-0.1)
                        self.rotating = self.rotateRight()
                        self.prevRotate = True
                elif direction == "up":
                    self.overMotor = True
                    self.rotate180()
            else:
                self.move()
                self.moving = True

            if self.overMotor == False:
                self.prev = actual

            self.overMotor = False

            print(self.path)
            print("cost\t" + cost)

            
        else:
            self.searchU = False
            self.startPath = True
            self.alreadyMoved = []
            self.moves = []


    def rotate180(self):
        self.fixMissU = 0
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        if self.initAngle == None: self.initAngle = self.measures.compass
        finalangle = self.initAngle + 180 + 180

        if finalangle > 180 :
            finalangle = finalangle - 360
        if angle != finalangle -1 and self.rotCnt < 159: 
            self.driveMotors(-0.01, 0.01)
            self.rotCnt += 1
            return 3
        else:
            self.driveMotors(0, 0)
            self.initAngle = None
            self.searchU = True
            self.rotCnt = 0
            self.test = False

            return 0
        
    def rotateLeft(self):
        self.fixMissU = 0
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        if self.initAngle == None: self.initAngle = self.measures.compass
        finalangle = self.initAngle + 90 + 180
        if finalangle > 180:
            finalangle = finalangle - 360

        if angle != finalangle - 1 and self.rotCnt < 80: 
            self.driveMotors(-0.01, 0.01)
            self.rotCnt += 1
            return 1
        else:

            self.driveMotors(0, 0)
            self.initAngle = None
            self.test = False
            self.rotCnt = 0
            return 0

    def rotateRight(self):
        self.fixMissU = 0
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        if self.initAngle == None: self.initAngle = self.measures.compass
        finalangle = self.initAngle - 90 + 180
        if finalangle > 180 :
            finalangle = finalangle - 360
        
        if angle != finalangle + 2 and self.rotCnt < 79: 
            self.driveMotors(0.01, -0.01)
            self.rotCnt += 1
            return 2
        else:
            self.driveMotors(0, 0)
            self.initAngle = None
            self.rotCnt = 0
            self.test = False

            return 0

    # returns the direction the robot is facing and a tuple with the coordinates to get to adjacent nodes (front,left,right,back)
    def get_direction(self):
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if angle in [-95,-94,-93, -92,-91, -90, -89, -88, -87,-86,-85]:
            return "up", [(0, 2), (-2, 0), (2, 0), (0, -2)]
        elif angle in [-5,-4,-3,-2,-1, 0, 1, 2,3,4,5]:
            return "left", [(-2, 0), (0, -2), (0, 2), (2, 0)]
        elif angle in [175,176,177, 178, 179, 180, -179, -178, -177,-176,-175]:
            return "right", [(2, 0), (0, 2), (0, -2), (-2, 0)]
        elif angle in [84,85,86, 87, 89, 90, 91, 92,93,94,95]:
            return "down", [(0, -2), (2, 0), (-2, 0), (0, 2)]
        return "None", []
    
    def mapping(self, inital = False, direct = ""):
         #posições por explorar, apontar no mapping out e no array
        if direct != "":
            x = self.currentPos[0]
            y = self.currentPos[1]
            
            #consuante a direção adiciona valores às coordenadas temporárias
            if direct == "right":
                y = self.currentPos[1] + 1
            elif direct == "left":
                y = self.currentPos[1] - 1
            elif direct == "up":
                x = self.currentPos[0] - 1
            elif direct == "down":
                x = self.currentPos[0] + 1

            if self.map[x][y] == " " :
                self.map[x][y] = "U"
                open('mapping.out', 'w').close()
                fout= open("mapping.out","w")
                tmp = ""
                for row in self.map:
                    for line in row:
                        tmp += line
                    tmp+="\n"
                fout.write(tmp)
                fout.close()
        # Defenir a posição inicial no array
        if inital:
            open('mapping.out', 'w').close()
            
            self.map[14-1][28-1] = "O"
            self.currentPos = [13,27]

            fout= open("mapping.out","w")
            tmp = ""
            for row in self.map:
                for line in row:
                    tmp += line
                tmp+="\n"
            fout.write(tmp)
            fout.close()
        # Apontar a posição atual no array
        else:
            if self.map[self.currentPos[0]][self.currentPos[1]] != "O" :
                self.map[self.currentPos[0]][self.currentPos[1]] = "X"
                open('mapping.out', 'w').close()
                fout= open("mapping.out","w")
                tmp = ""
                for row in self.map:
                    for line in row:
                        tmp += line
                    tmp+="\n"
                fout.write(tmp)
                fout.close()

    def centerDirection(self):
        direction, coord = self.get_direction()


        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        #debug
        """print(self.measures.compass )
        print(angle)
        print("stop")"""

        if direction == "up":
            if angle > -88: 
                self.driveMotors(0.01, -0.01)
                return False
            elif angle < -92: 
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                self.driveMotors(0, 0)
                return True
        elif direction == "down":
            if angle > 92: 
                self.driveMotors(0.01, -0.01)
                return False
            elif angle < 88: 
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                self.driveMotors(0, 0)
                return True
        elif direction == "right":
            if 0 > angle >= -177: 
                self.driveMotors(0.01, -0.01)
                return False
            elif 0 < angle < 178:
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                self.driveMotors(0, 0)
                return True
        elif direction == "left":
            if angle > 2: 
                self.driveMotors(0.01, -0.01)
                return False
            elif angle < -2:
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                self.driveMotors(0, 0)
                return True
    
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

class Unknown(SearchDomain):
    def __init__(self,connect):
        self.moves = connect

    def actions(self,connect):
        actlist = []
        for (C1,C2,D) in self.moves:
            if (C1==connect):
                actlist += [(C1,C2)]
        return actlist 
    def result(self,connect,action):
        (C1,C2) = action
        if C1==connect:
            return C2
    def cost(self, connect, action):
        (C3,C4) = action
        for (C1,C2,D) in self.moves:
            if C1 == C3 and C2 == C4:
                return D
        return ""
    def heuristic(self, goal):
        for (C1,C2,D) in self.moves:
            if C2 == goal:
                return 0
        return 0
    def satisfies(self, connect, goal):
        return goal==connect


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
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
