
from enum import Flag
from os import remove
import sys
from tree_search import *
from croblink import *
from math import *
from cmath import log, sqrt
from random import betavariate, randrange
import xml.etree.ElementTree as ET
from itertools import permutations

CELLROWS=7
CELLCOLS=14

# IMPORTANTE:
#   O x e y representam x+1 e y+1 no mapping.map. Logo os impares no self.map são pares no mapping.map e vice versa. 

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.cnt = 0 #moving cnt
        self.moving = False #está se a mover
        self.rotating = 0
        self.currentPos=[0,0]
        rows, cols = (27, 55)
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
        self.nearU = False
        self.beaconLoc = []
        self.plan = False


        self.inpower= 0
        self.outpower= 0
        self.prevpower=0
        self.mov = 0
        


        #tentar corrigir a erros de sensores nas posições de U. Com o ruido dos sensores pode haver U
        self.fixMissU = 0 
        self.rightCnt = 0
        self.leftCnt = 0
        self.upCnt = 0
        self.downCnt = 0

        self.test = True  #debug


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
                print(rob_name + " exiting")
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

        if self.measures.collision: self.finish()

        if self.plan: 
            self.planning()
            self.plan = False      
        elif self.moving:
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
                    self.rotating = self.rotate180()"""
                    
                print(self.fixMissU)
                print(left)
                print(right)
                print(front)
                print(self.prevRotate)
                
                if front <= 1.2:
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

        direction, coord = self.get_direction()

        if self.mov >= 0.8 or front > 1.4: 
            self.inpower = 0.126
            self.driveMotors(0.126,0.126)
            if direction == None : quit()
            print(direction)
            #self.driveMotors(0,0)  
            self.moving = False 
            self.test = False
            self.cnt = 0
            self.finished_rotation = False
            self.inpower= 0
            self.outpower= 0
            #self.prevpower=0
            self.mov = 0

            #atualiza a current position quando acaba o movimento 
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
            self.alreadyVisited.append([x,y]) # já vistou a posição atual
            
            if self.fixMissU % 2 == 0: # só em linhas pares é podes haver movimento, ou seja, só em pares é que pode haver caminhos por explorar
                if direction == "right":
                    if left < 1.2 : self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2 : self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    #if front < 1.2 : self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2 : self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                if direction == "left":
                    if left < 1.2 : self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2 : self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    #if front < 1.2 : self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2 : self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                if direction == "up":
                    if left < 1.2 : self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2 : self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    #if front < 1.2 : self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2 : self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                if direction == "down":
                    if left < 1.2 : self.mapping(False, "right") #Posições por explorar, apontar no mapping out e no array
                    if right < 1.2 : self.mapping(False, "left") #Posições por explorar, apontar no mapping out e no array
                    #if front < 1.2 : self.mapping(False, "down") #Posições por explorar, apontar no mapping out e no array
                    if back < 1.2 : self.mapping(False, "up") #Posições por explorar, apontar no mapping out e no array
        elif self.mov >= 0.6: 
            self.inpower = 0.138
            self.driveMotors(0.138,0.138)
        else: 
            self.inpower = 0.15
            self.driveMotors(0.15,0.15)
        x =self.randNormal(100, 1.5*1.5) / 100;
        self.outpower = (0.5*self.inpower + 0.5*self.prevpower) * x
        z = complex(self.outpower)
        self.outpower = z.real
        self.prevpower = self.outpower
        self.mov += self.outpower
        
    
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

            #ver map e procurar X à volta e enviar para search tree, se encontrar o U vai guardar em moves
            if (self.map[x][right] == "X" or self.map[x][right] == "0" or self.map[x][right] == "U" or self.map[x][right] == "1" or self.map[x][right] == "2" or self.map[x][right] == "3" or self.map[x][right] == "4") and [current,[x,right]] not in self.alreadyMoved:
                self.moves.append([current, [x,right], "right" ])
                self.alreadyMoved.append([current,[x,right]])
                searchU(self,x,right)
            elif self.map[x][right] == "U": 
                self.moves.append([current, [x,right], "right" ])
                self.Ulocation.append([x,right])
            if (self.map[x][left] == "X"  or self.map[x][left] == "0"  or self.map[x][left] == "U" or self.map[x][left] == "1" or self.map[x][left] == "2" or self.map[x][left] == "3" or self.map[x][left] == "4" ) and [current,[x,left]] not in self.alreadyMoved: 
                self.moves.append([current, [x,left], "left"])
                self.alreadyMoved.append([current,[x,left]])
                searchU(self,x,left)
            elif self.map[x][left] == "U": 
                self.moves.append([current, [x,left], "left"])
                self.Ulocation.append([x,left])
            if (self.map[up][y] == "X" or self.map[up][y] == "0" or self.map[up][y] == "U" or self.map[up][y] == "1" or self.map[up][y] == "2" or self.map[up][y] == "3" or self.map[up][y] == "4") and [current,[up,y]] not in self.alreadyMoved:
                self.moves.append([current, [up,y], "up"])
                self.alreadyMoved.append([current,[up,y]])
                searchU(self,up,y)
            elif self.map[up][y] == "U": 
                self.moves.append([current, [up,y], "up"])
                self.Ulocation.append([up,y])
            if (self.map[down][y] == "X" or self.map[down][y] == "0" or self.map[down][y] == "U" or self.map[down][y] == "1" or self.map[down][y] == "2" or self.map[down][y] == "3" or self.map[down][y] == "4") and [current,[down,y]] not in self.alreadyMoved: 
                self.moves.append([current, [down,y], "down"])
                self.alreadyMoved.append([current,[down,y]])
                searchU(self,down,y)
            elif self.map[down][y] == "U": 
                self.moves.append([current, [down,y], "down"])
                self.Ulocation.append([down,y])

        #Após já ter todos os moves e as localizações dos U, enviar para a searching tree
        if self.startPath == True and self.path == []:
            
            searchU(self, self.currentPos[0], self.currentPos[1])
            print("Looking")
            self.tree = Unknown(self.moves)

            tmp = [ b.copy() for b in self.Ulocation ]
            
            # se não houver U voltar para a posição inicial
            if self.Ulocation == [] :
                p = SearchProblem(self.tree,self.currentPos,[13,27])
                t = SearchTree(p,'a*')
                self.path = t.search()
                self.prevRotate = False

            #porcura qual o U mais proximo da posição atual
            else:
                bestLoc = []
                print(tmp)
                for a in tmp:
                    loc = a
                    if bestLoc == []: bestLoc.append(loc)
                    else:
                        x = self.currentPos[0]
                        y = self.currentPos[1]
                        xb = bestLoc[0][0]
                        yb = bestLoc[0][1]
                        xl = loc[0]
                        yl = loc[1]
                        if (abs(x - xb) + abs(y - yb)) >= (abs(x - xl) + abs(y - yl)): bestLoc.append(loc)
                
                print(self.currentPos)
                print(bestLoc) 

                if bestLoc != []:
                    for loc in bestLoc:
                        p = SearchProblem(self.tree,self.currentPos,loc)
                        t = SearchTree(p,'a*')
                        tmpPath = t.search()
                        if self.path == [] : self.path = tmpPath
                        if( len(tmpPath) < len(self.path) ): self.path = tmpPath
                
                self.prevRotate = False
                print(self.path)
                print(self.moves)
        
        #caso tenha havido rotação anteriormente no caminho encontrado pela tree, vai se movimentar para a frente
        elif self.prevRotate:
            self.move()
            self.moving = True
            self.prevRotate = False

        elif self.path != []:
            #seguir o caminho defenido no path até ao U
            if self.startPath: 
                self.prev = self.path.pop(0)
            direction, coord = self.get_direction()

            self.startPath = False

            if self.path == [] and self.measures.ground == 0: 
                self.finish() #end
                return

            actual = self.path.pop(0)

            cost = self.tree.cost(0,[self.prev,actual])

            x = self.currentPos[0]
            y = self.currentPos[1]

            current = [x,y]
            up = x - 1
            down = x + 1
            rightM = y + 1
            leftM = y - 1

            #se encontrar um U pelo caminho dado pela tree, o robot optar por ir para o U encontrado
            if direction == "right":
                if self.map[x][rightM] == "U":
                    self.move()
                    self.moving = True    
                    self.nearU = True
                elif self.map[up][y] == "U":
                    if left > 1:
                        self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateLeft()
                        self.nearU = True
                elif self.map[down][y] == "U": 
                    if right > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateRight()
                        self.nearU = True
            elif direction == "left":
                if self.map[x][leftM] == "U":
                    self.move()
                    self.moving = True   
                    self.nearU = True          
                elif self.map[up][y] == "U":
                    if right > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateRight()
                        self.nearU = True
                elif self.map[down][y] == "U": 
                    if left > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateLeft()
                        self.nearU = True
            elif direction == "up":
                if self.map[x][rightM] == "U":
                    if right > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateRight()  
                        self.nearU = True                
                elif self.map[up][y] == "U":
                    self.move()
                    self.moving = True    
                    self.nearU = True
                elif self.map[x][leftM] == "U":
                    if left > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateLeft() 
                        self.nearU = True  
            elif direction == "down":
                if self.map[x][rightM] == "U":
                    if left > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateLeft()  
                        self.nearU = True                
                elif self.map[down][y] == "U":
                    self.move()
                    self.moving = True    
                    self.nearU = True
                elif self.map[x][leftM] == "U":
                    if right > 1:
                            self.driveMotors(-0.15,-0.15)
                    else:
                        self.rotating = self.rotateRight() 
                        self.nearU = True
                    
            # segue o caminho feito pela tree normalmente senão houver nenhum U pelo caminho
            if self.nearU == False:
                if cost == "right":
                    if direction == "right":
                        self.move()
                        self.moving = True
                    elif direction == "up":
                        if right > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(0.1,-0.1)
                            self.rotating = self.rotateRight()
                            self.prevRotate = True
                    elif direction == "down":
                        if left > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(-0.1,0.1)
                            self.rotating = self.rotateLeft()
                            self.prevRotate = True
                    elif direction == "left":
                        self.prevRotate = True
                        self.rotating = self.rotate180()
                elif cost == "left":
                    if direction == "left":
                        self.move()
                        self.moving = True
                    elif direction == "up":
                        if left > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(-0.1,0.1)
                            self.rotating = self.rotateLeft()
                            self.prevRotate = True
                    elif direction == "down":
                        if right > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(0.1,-0.1)
                            self.rotating = self.rotateRight()
                            self.prevRotate = True
                    elif direction == "right":
                        self.prevRotate = True
                        self.rotating = self.rotate180()
                elif cost == "up":
                    if direction == "up":
                        self.move()
                        self.moving = True
                    elif direction == "left":
                        if right > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(0.1,-0.1)
                            self.rotating = self.rotateRight()
                            self.prevRotate = True
                    elif direction == "right":
                        if left > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(-0.1,0.1)
                            self.rotating = self.rotateLeft()
                            self.prevRotate = True
                    elif direction == "down":
                        self.prevRotate = True
                        self.rotating = self.rotate180()
                elif cost == "down":
                    if direction == "down":
                        self.move()
                        self.moving = True
                    elif direction == "left":
                        print(left)
                        if left > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(-0.1,0.1)
                            self.rotating = self.rotateLeft()
                            self.prevRotate = True
                    elif direction == "right":
                        if right > 1:
                            self.overMotor = True
                            self.driveMotors(-0.15,-0.15)
                            self.path.insert(0,actual)
                        else:
                            self.driveMotors(0.1,-0.1)
                            self.rotating = self.rotateRight()
                            self.prevRotate = True
                    elif direction == "up":
                        self.prevRotate = True
                        self.rotating = self.rotate180()
                else:
                    self.move()
                    self.moving = True
            else:
                #dá reset às variaveis se encontrar um U pelo caminho
                self.nearU = False
                self.searchU = False
                self.startPath = True
                self.alreadyMoved = []
                self.moves = []
                self.path = []

            if self.overMotor == False:
                self.prev = actual

            self.overMotor = False

            print(self.path)
            print("cost\t" + cost)

            
        else:
            #quando acaba dá reset às variaveis
            self.searchU = False
            self.startPath = True
            self.alreadyMoved = []
            self.moves = []
            self.path = []


    def rotate180(self):
        self.fixMissU = 0
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        if self.initAngle == None: self.initAngle = self.measures.compass
        finalangle = self.initAngle + 180 + 180

        if finalangle > 180 :
            finalangle = finalangle - 360
        if angle != finalangle -1 and self.rotCnt < 16: 
            self.driveMotors(-0.1, 0.1)
            self.rotCnt += 1
            return 3
        else:
            self.driveMotors(-0.004, -0.004)
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
        if self.initAngle == None: 
            #self.rotateFix()
            self.initAngle = self.measures.compass
        finalangle = self.initAngle + 90 + 180
        if finalangle > 180:
            finalangle = finalangle - 360

        if angle != finalangle - 1 and self.rotCnt < 8: 
            self.driveMotors(-0.1, 0.1)
            self.rotCnt += 1
            return 1
        else:
            self.driveMotors(0.015, 0.015)
            self.initAngle = None
            self.test = False
            self.rotCnt = 0
            return 0

    def rotateRight(self):
        self.fixMissU = 0
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360
        if self.initAngle == None: 
            #self.rotateFix()
            self.initAngle = self.measures.compass
        finalangle = self.initAngle - 90 + 180
        if finalangle > 180 :
            finalangle = finalangle - 360
        
        if angle != finalangle + 2 and self.rotCnt < 8: 
            self.driveMotors(0.1, -0.1)
            self.rotCnt += 1
            return 2
        else:
            self.driveMotors(0.015, 0.015)
            self.initAngle = None
            self.rotCnt = 0
            self.test = False

            return 0
    
    #apenas podem haver rotações em pontos impares do mapa(mapa do codigo, porque se for o mapping.map é nos pontos pares)
    def rotateFix(self):
        direction, coord = self.get_direction()

        if self.currentPos[0] % 2 == 0:
            if direction == "right":
                self.currentPos[1] = self.currentPos[1] - 1
            elif direction == "left":
                self.currentPos[1] = self.currentPos[1] + 1
            elif direction == "up":
                self.currentPos[0] = self.currentPos[0] + 1
            elif direction == "down":
                self.currentPos[0] = self.currentPos[0] - 1
            self.mapping()
        if self.currentPos[1] % 2 == 0:
            if direction == "right":
                self.currentPos[1] = self.currentPos[1] - 1
            elif direction == "left":
                self.currentPos[1] = self.currentPos[1] + 1
            elif direction == "up":
                self.currentPos[0] = self.currentPos[0] + 1
            elif direction == "down":
                self.currentPos[0] = self.currentPos[0] - 1
            self.mapping()

    # returns the direction the robot is facing and a tuple with the coordinates to get to adjacent nodes (front,left,right,back)
    def get_direction(self):
        angle = self.measures.compass + 180
        if angle > 180:
            angle = angle - 360

        if -130 < angle < -40:
            return "up", [(0, 2), (-2, 0), (2, 0), (0, -2)]
        elif -40 < angle < 40:
            return "left", [(-2, 0), (0, -2), (0, 2), (2, 0)]
        elif 130 < angle <=180 or -179 <= angle < -130:
            return "right", [(2, 0), (0, 2), (0, -2), (-2, 0)]
        elif 40 < angle < 130:
            return "down", [(0, -2), (2, 0), (-2, 0), (0, 2)]
        return "None", []

    def planning(self):
        def searchU(self, x, y):
            current = [x,y]
            up = x - 1
            down = x + 1
            right = y + 1
            left = y - 1

            #ver map e procurar X à volta e enviar para search tree, se encontrar o U vai guardar em moves
            if (self.map[x][right] == "X" or self.map[x][right] == "0" or self.map[x][right] == "U" or self.map[x][right] == "1" or self.map[x][right] == "2" or self.map[x][right] == "3" or self.map[x][right] == "4") and [current,[x,right]] not in self.alreadyMoved:
                self.moves.append([current, [x,right], "right" ])
                self.alreadyMoved.append([current,[x,right]])
                searchU(self,x,right)
            elif self.map[x][right] == "U": 
                self.moves.append([current, [x,right], "right" ])
                self.Ulocation.append([x,right])
            if (self.map[x][left] == "X"  or self.map[x][left] == "0"  or self.map[x][left] == "U" or self.map[x][left] == "1" or self.map[x][left] == "2" or self.map[x][left] == "3" or self.map[x][left] == "4" ) and [current,[x,left]] not in self.alreadyMoved: 
                self.moves.append([current, [x,left], "left"])
                self.alreadyMoved.append([current,[x,left]])
                searchU(self,x,left)
            elif self.map[x][left] == "U": 
                self.moves.append([current, [x,left], "left"])
                self.Ulocation.append([x,left])
            if (self.map[up][y] == "X" or self.map[up][y] == "0" or self.map[up][y] == "U" or self.map[up][y] == "1" or self.map[up][y] == "2" or self.map[up][y] == "3" or self.map[up][y] == "4") and [current,[up,y]] not in self.alreadyMoved:
                self.moves.append([current, [up,y], "up"])
                self.alreadyMoved.append([current,[up,y]])
                searchU(self,up,y)
            elif self.map[up][y] == "U": 
                self.moves.append([current, [up,y], "up"])
                self.Ulocation.append([up,y])
            if (self.map[down][y] == "X" or self.map[down][y] == "0" or self.map[down][y] == "U" or self.map[down][y] == "1" or self.map[down][y] == "2" or self.map[down][y] == "3" or self.map[down][y] == "4") and [current,[down,y]] not in self.alreadyMoved: 
                self.moves.append([current, [down,y], "down"])
                self.alreadyMoved.append([current,[down,y]])
                searchU(self,down,y)
            elif self.map[down][y] == "U": 
                self.moves.append([current, [down,y], "down"])
                self.Ulocation.append([down,y])

        if self.beaconLoc == []: return

        perm = permutations( self.beaconLoc )

        bestPerm = []
        lines = 0
        bestLines = 0
        for beacon in perm:
            lines = 0
            open('planning.path', 'w').close()
            fout = open('planning.path', 'w')
            prevPos = [13,27]
            s = ""
            print(beacon)
            removePath = 0
            searchU(self, prevPos[0], prevPos[1])
            for loc in beacon: 
                self.tree = Unknown(self.moves)
                print(prevPos)
                p = SearchProblem(self.tree,prevPos,loc[0])
                prevPos = loc[0]
                t = SearchTree(p,'uniform')
                path = t.search()
                cnt = 0
                if removePath > 0 : 
                    path.pop(0)
                    cnt += 1
                removePath +=1
                for pa in path:
                    if cnt % 2 == 0:
                        lines += 1
                        s += str(pa[1] - 27)
                        s += " "
                        s += str(13 - pa[0])
                        if pa == loc[0]:   
                            s += " # "
                            s += str(loc[1])
                        s += "\n"
                    cnt += 1
            self.tree = Unknown(self.moves)
            p = SearchProblem(self.tree,prevPos,[13,27])
            t = SearchTree(p,'uniform')
            path = t.search()
            removePath +=1
            cnt = 1
            path.pop(0)
            for pa in path:
                if cnt % 2 == 0:
                    lines += 1
                    s += str(pa[1] - 27)
                    s += " "
                    s += str(13 - pa[0])
                    if pa == loc[0]:   
                        s += " # "
                        s += str(loc[1])
                    s += "\n"
                cnt += 1
            fout.write(s)
            fout.close()
            self.alreadyMoved = []
            self.moves = []
            if bestPerm == [] or bestLines >= lines: 
                bestPerm = beacon
                bestLines = lines

        open('planning.path', 'w').close()
        fout = open('planning.path', 'w')
        prevPos = [13,27]
        s = ""
        
        removePath = 0
        searchU(self, prevPos[0], prevPos[1])
        for loc in bestPerm: 
            self.tree = Unknown(self.moves)
            print(prevPos)
            p = SearchProblem(self.tree,prevPos,loc[0])
            prevPos = loc[0]
            t = SearchTree(p,'uniform')
            path = t.search()
            cnt = 0
            if removePath > 0 : 
                path.pop(0)
                cnt += 1
            removePath +=1
            for pa in path:
                if cnt % 2 == 0:
                    lines += 1
                    s += str(pa[1] - 27)
                    s += " "
                    s += str(13 - pa[0])
                    if pa == loc[0]:   
                        s += " # "
                        s += str(loc[1])
                    s += "\n"
                cnt += 1
        self.tree = Unknown(self.moves)
        p = SearchProblem(self.tree,prevPos,[13,27])
        t = SearchTree(p,'uniform')
        path = t.search()
        removePath +=1
        cnt = 1
        path.pop(0)
        self.driveMotors(-0.01,-0.01)
        for pa in path:
            if cnt % 2 == 0:
                lines += 1
                s += str(pa[1] - 27)
                s += " "
                s += str(13 - pa[0])
                if pa == loc[0]:   
                    s += " # "
                    s += str(loc[1])
                s += "\n"
            cnt += 1
        fout.write(s)
        fout.close()
        self.alreadyMoved = []
        self.moves = []


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
                open('mapping.map', 'w').close()
                fout= open("mapping.map","w")
                tmp = ""
                for row in self.map:
                    for line in row:
                        tmp += line
                    tmp+="\n"
                fout.write(tmp)
                fout.close()
        # Defenir a posição inicial no array
        if inital:
            open('mapping.map', 'w').close()
            
            self.map[14-1][28-1] = "0"
            self.currentPos = [13,27]

            fout= open("mapping.map","w")
            tmp = ""
            for row in self.map:
                for line in row:
                    tmp += line
                tmp+="\n"
            fout.write(tmp)
            fout.close()
        # Apontar a posição atual no array
        else:            
            if self.map[self.currentPos[0]][self.currentPos[1]] != "0"  and self.map[self.currentPos[0]][self.currentPos[1]] != "1" \
                and self.map[self.currentPos[0]][self.currentPos[1]] != "2" and self.map[self.currentPos[0]][self.currentPos[1]] != "3" and self.map[self.currentPos[0]][self.currentPos[1]] != "4":
                
                if self.measures.ground == 1: 
                    self.map[self.currentPos[0]][self.currentPos[1]] = "1"
                    self.beaconLoc.append((self.currentPos.copy(),1))
                elif self.measures.ground == 2:
                    self.map[self.currentPos[0]][self.currentPos[1]] = "2"
                    self.beaconLoc.append((self.currentPos.copy(),2))
                elif self.measures.ground == 3:
                    self.map[self.currentPos[0]][self.currentPos[1]] = "3"
                    self.beaconLoc.append((self.currentPos.copy(),3))
                elif self.measures.ground == 4:
                    self.map[self.currentPos[0]][self.currentPos[1]] = "4"
                    self.beaconLoc.append((self.currentPos.copy(),4))
                else:
                    self.map[self.currentPos[0]][self.currentPos[1]] = "X"
                open('mapping.map', 'w').close()
                fout= open("mapping.map","w")
                tmp = ""
                for row in self.map:
                    for line in row:
                        tmp += line
                    tmp+="\n"
                fout.write(tmp)
                fout.close()
            
            if self.measures.ground != -1: 
                self.plan = True
                self.driveMotors(0,0)

    #vai centar a diração do robot consuante a direção que este esteja
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
                #self.driveMotors(0, 0)
                return True
        elif direction == "down":
            if angle > 92: 
                self.driveMotors(0.01, -0.01)
                return False
            elif angle < 88: 
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                #self.driveMotors(0, 0)
                return True
        elif direction == "right":
            if 0 > angle >= -177: 
                self.driveMotors(0.01, -0.01)
                return False
            elif 0 < angle < 178:
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                #self.driveMotors(0, 0)
                return True
        elif direction == "left":
            if angle > 2: 
                self.driveMotors(0.01, -0.01)
                return False
            elif angle < -2:
                self.driveMotors(-0.01, 0.01)
                return False
            else:
                #self.driveMotors(0, 0)
                return True
    
    def randNormal(self,  mean,  stdev):
        x = y = r = 0
        while True:
            x = 2 * ( randrange(0,2147483647) / 2147483647 + 1) - 1
            y = 2 * ( randrange(0,2147483647) / 2147483647 + 1) - 1
            r = x*x + y*y
            if(r >1 ): break
        return x*sqrt((-2*log(r)/r)) * stdev + mean
    
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
    def heuristic(self,connect, goal):
        return abs(connect[0] - goal[0]) + abs(connect[1] - goal[1]) 
        
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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
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
