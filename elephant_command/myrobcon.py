# -*- coding: utf-8 -*-

from elephant import elephant_command
import time
import random

GRID_NUM_HEIGHT=14
GRID_NUM_WIDTH=10


erobot = elephant_command()
def get_piece():
    erobot.set_digital_out(0,1)
    time.sleep(1)   
def free_piece():
    erobot.set_digital_out(0,0)

#------------------------------


def SET(x1,y1):
    print(erobot.get_angles())
    erobot.set_coord('z', 50, 1000)
    time.sleep(0.1)
    while erobot.check_running():
        print('downing')
        time.sleep(0.1)

    get_piece()
    time.sleep(3)

    erobot.set_coord('z', 200, 1000)
    time.sleep(0.1)
    while erobot.check_running():
        print('running2')
        time.sleep(0.1)
    time.sleep(0.1)
        
    erobot.set_angle('0', 180, 400)
    time.sleep(0.1)
    while erobot.check_running():
        print('running2')
        time.sleep(0.1)
    time.sleep(0.1)
	
    erobot.set_coord('x', x1, 1000)
    time.sleep(0.1)
    while erobot.check_running():
        print('downing')
        time.sleep(0.1)
	
    erobot.set_coord('y', y1, 1000)
    time.sleep(0.1)
    while erobot.check_running():
        print('downing')
        time.sleep(0.1)

    erobot.set_coord('z', 107, 800)
    time.sleep(0.1)
    while erobot.check_running():
        print('downing')
        time.sleep(0.1)

    
    free_piece()
    time.sleep(2)

    erobot.set_coord('z', 200, 1000)
    time.sleep(0.1)
    while erobot.check_running():
        print('downing')
        time.sleep(0.1)

    erobot.set_angle('0', 90, 400)
    time.sleep(0.1)
    while erobot.check_running():
        print('running2')
        time.sleep(0.1)
    time.sleep(0.1)

	

class RobotWorker():
	SHAPES = ['I', 'J', 'L', 'O', 'S', 'T', 'Z']
	I = [[(0, -1), (0, 0), (0, 1), (0, 2)],
		 [(-1, 0), (0, 0), (1, 0), (2, 0)]]
	J = [[(-2, 0), (-1, 0), (0, 0), (0, -1)],
		 [(-1, 0), (0, 0), (0, 1), (0, 2)],
		 [(0, 1), (0, 0), (1, 0), (2, 0)],
		 [(0, -2), (0, -1), (0, 0), (1, 0)]]
	L = [[(-2, 0), (-1, 0), (0, 0), (0, 1)],
		 [(1, 0), (0, 0), (0, 1), (0, 2)],
		 [(0, -1), (0, 0), (1, 0), (2, 0)],
		 [(0, -2), (0, -1), (0, 0), (-1, 0)]]
	O = [[(0, 0), (0, 1), (1, 0), (1, 1)]]
	S = [[(-1, 0), (0, 0), (0, 1), (1, 1)],
		 [(1, -1), (1, 0), (0, 0), (0, 1)]]
	T = [[(0, -1), (0, 0), (0, 1), (-1, 0)],
		 [(-1, 0), (0, 0), (1, 0), (0, 1)],
		 [(0, -1), (0, 0), (0, 1), (1, 0)],
		 [(-1, 0), (0, 0), (1, 0), (0, -1)]]
	Z = [[(0, -1), (0, 0), (1, 0), (1, 1)],
		 [(-1, 0), (0, 0), (0, -1), (1, -1)]]

	SHAPES_WITH_DIR = {
		'I': I, 'J': J, 'L': L, 'O': O, 'S': S, 'T': T, 'Z': Z
	}
	
	def __init__(self,center,shape,station,color,matrix):
		self.center = center
		self.shape = shape
		self.station = station
		self.color = color
		self.matrix = matrix

	def get_all_gridpos(self, center,shape,dir):
		curr_shape = self.SHAPES_WITH_DIR[shape][dir]

		return [(cube[0] + center[0], cube[1] + center[1])
				for cube in curr_shape] #返回中心确定后每个点的位置
	#碰撞检测
	def conflict(self, center,matrix,shape,dir):
		for cube in self.get_all_gridpos(center,shape,dir):
			# 超出屏幕之外，说明不合法
			if cube[0] < 0 or cube[1] < 0 or cube[0] >= GRID_NUM_HEIGHT or  cube[1] >= GRID_NUM_WIDTH:
				return True

			screen_color_matrix = self.copyTheMatrix( matrix )
			# 不为None，说明之前已经有小方块存在了，也不合法
			if screen_color_matrix[cube[0]][cube[1]] is not None:
				return True

		return False

	def copyTheMatrix(self,screen_color_matrix):
		newMatrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
		for i in range( len( screen_color_matrix ) ):
			for j in range( len( screen_color_matrix[i] ) ):
				newMatrix[i][j] = screen_color_matrix[i][j]

		return newMatrix

	def getAllPossiblePos(self,thisShape):
		theMatrix = self.matrix
		theStationNum = len(self.SHAPES_WITH_DIR[thisShape])
		theResult = []
		for k in range(theStationNum):
			for j in range(len(theMatrix[1])):
				for i in range(len(theMatrix) - 1):
					if self.conflict([i + 1, j], theMatrix, thisShape, k) == True and self.conflict([i, j], theMatrix,
																								thisShape, k) == False:
						if {"center": [i, j], "station": k} not in theResult:
							theResult.append({"center": [i, j], "station": k})

		return theResult

	# 获取方块海拔。
	def getLandingHeight(self,center):
		return GRID_NUM_HEIGHT-1-center[0]


	def getErodedPieceCellsMetric(self,center,station):  #获取消除的行数
		theNewMatrix = self.getNewMatrix(center,station)
		lines = 0
		usefulBlocks = 0
		theAllPos = self.get_all_gridpos(center,self.shape,station)
		for i in range(len(theNewMatrix)-1,0,-1):
			count = 0
			for j in range(len(theNewMatrix[1])):
				if theNewMatrix[i][j] is not None:
					count += 1
			# 满一行
			if count == GRID_NUM_WIDTH:
				lines +=1
				for k in range(len(theNewMatrix[1])):
					if [i,k] in theAllPos:
						usefulBlocks +=1
			# 整行未填充，则跳出循环
			if count == 0:
				break
		return lines*usefulBlocks

	# 把可能的坐标位置放进去颜色矩阵，形成新的颜色矩阵。
	def getNewMatrix(self,center,station):
		theNewMatrix = self.copyTheMatrix(self.matrix)
		theAllPos = self.get_all_gridpos(center,self.shape,station)
		for cube in theAllPos:
			theNewMatrix[cube[0]][cube[1]] = self.color
		return theNewMatrix

	# 获取行变换数
	def getBoardRowTransitions(self,theNewmatrix):
		transition = 0
		for i in range( len(theNewmatrix)-1 , 0 , -1 ):
			count = 0
			for j in range( len(theNewmatrix[1])-1 ):
				if theNewmatrix[i][j] is not None :
					count += 1
				if theNewmatrix[i][j] == None and theNewmatrix[i][j+1] != None:
					transition += 1
				if theNewmatrix[i][j] != None and theNewmatrix[i][j+1] == None:
					transition += 1
		return transition

	# 获取列变换数
	def getBoardColTransitions(self,theNewmatrix):
		transition = 0
		for j in range( len(theNewmatrix[1]) ):
			for i in range( len(theNewmatrix)-1,1,-1 ):
				if theNewmatrix[i][j] == None and theNewmatrix[i-1][j] != None:
					transition += 1
				if theNewmatrix[i][j] != None and theNewmatrix[i-1][j] == None:
					transition += 1
		return transition

	def getBoardBuriedHoles(self,theNewmatrix):
		holes = 0
		for j in range(len( theNewmatrix[1] )):
			colHoles = None
			for i in range( len( theNewmatrix ) ):
				if colHoles == None and theNewmatrix[i][j] != None:
					colHoles = 0

				if colHoles != None and theNewmatrix[i][j] == None:
					colHoles += 1
			if colHoles is not None:
				holes += colHoles
		return holes

	def getBoardWells(self,theNewmatrix):
		sum_n = [0,1,3,6,10,15,21,28,36,45,55]
		wells = 0
		sum = 0

		for j in range( len(theNewmatrix[1]) ):
			for i in range( len(theNewmatrix) ):
				if theNewmatrix[i][j] == None:
					if (j-1<0 or theNewmatrix[i][j-1] != None) and (j+1 >= GRID_NUM_WIDTH or theNewmatrix[i][j+1] != None):
						wells += 1
					else:
						sum += sum_n[wells]
						wells = 0
		return sum
		# 计算优先度的函数
	def getPrioritySelection(self,point):
		tarStation = point['station']
		nowStation = self.station
		#
		colNum = abs(7 - point['center'][1] )
		if tarStation >= nowStation:
			changeTimes = tarStation - nowStation
		else :
			changeTimes = len(self.SHAPES_WITH_DIR[self.shape]) - nowStation + tarStation

		result = colNum*100 + changeTimes
		if point['center'][1] <=7 :
			result += 10
		return result

	# 根据点的中心位置计算分数
	def evaluateFunction(self,point):
		newMatrix = self.getNewMatrix( point['center'],point['station'] )
		lh = self.getLandingHeight( point['center'] )
		epcm = self.getErodedPieceCellsMetric(point['center'],point['station'])
		brt = self.getBoardRowTransitions(newMatrix)
		bct = self.getBoardColTransitions(newMatrix)
		bbh = self.getBoardBuriedHoles(newMatrix)
		bw = self.getBoardWells(newMatrix)

		# 两个计算分数的式子，前者更优，后者是PD算法的原始设计
		score = -45*lh + 34*epcm - 32*brt - 98*bct - 79* bbh -34*bw
		# score = -1*lh + epcm - brt - bct - 4*bbh - bw
		return score

	def mainProcess(self):
		pos = self.getAllPossiblePos(self.shape)
		bestScore = -999999
		bestPoint = None
		for point in pos:
			theScore = self.evaluateFunction( point)
			if theScore > bestScore:
				bestScore = theScore
				bestPoint = point
			elif theScore == bestScore:
				if self.getPrioritySelection( point ) < self.getPrioritySelection(bestPoint):
					bestScore = theScore
					bestPoint = point

		return bestPoint

def printt(a):
	for i in range(len(a)):
		for j in range(len(a[i])):
			if(a[i][j]== None):
				print(0,end="")
			else:
				print("1",end="")
		print("")
	print("")

def main():
	a = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
	while 1 :
		s=input()
		now=RobotWorker([0,0],s,0,1,a)
		pt=now.mainProcess()
		a=now.getNewMatrix(pt['center'],pt['station'])
		x0=476.44
		y0=-159.18
		l=16
		print(pt['center'][0],pt['center'][1],"------------",sep=" ")
		SET(x0-pt['center'][0]*l,y0+pt['center'][1]*l)
		printt(a)

main()



#erobot.set_coord('z', 150, 400)




#print(erobot.get_angles())
#erobot.set_coord('z', 50, 400)
#time.sleep(0.1)
#while erobot.check_running():
    #print('running')
    #time.sleep(0.1)
#erobot.set_angle(1, 130, 100)
#time.sleep(0.1)
#while erobot.check_running():
#    print('running2')
#    time.sleep(0.1)

#time.sleep(0.5)   控制吸盘的代码

#for _ in range(2):
#erobot.set_digital_out(0,0)
#time.sleep(5)
#erobot.set_digital_out(0,0)
#time.sleep(1)

# erobot.set_angle('6', -180, 100)
# 0 - > -147.6

#erobot.task_stop()

# X0 = -173.0
# Y0 = 298.0
# Z0 = 320.0 # mm
# # RX0 = 180.0
# RY0 = 0.0
# RZ0 = -22.0
# ZG = 240.0 # mm

# cur_poses_str = erobot.get_coords()
# print(cur_poses_str)
# cur_poses = [float(p) for p in cur_poses_str[12:-1].split(',')]
# print(cur_poses)
# cur_poses[5] = -10

# erobot.set_coords(cur_poses, 500)
# time.sleep(2)
# cur_poses_str = erobot.get_coords()
# print(cur_poses_str)
# # erobot.set_coord('rz', -90, 800)
# # erobot.set_coord('z', 360, 800)
# time.sleep(2)

# 9.14测试
# 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[2] = 200
# erobot.set_coords(cur_pose, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running', time.time())
#     time.sleep(0.1)

# erobot.set_coord('z', 250, 800)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)

# 关节空间
# cur_angles = erobot.get_angles()
# print(cur_angles)
# cur_angles[0] = -80.0
# erobot.set_angles(cur_angles, 500)
# time.sleep(0.1)
# run_flag = erobot.check_running()
# while run_flag:
#     print('running')
#     time.sleep(0.1)
#     run_flag = erobot.check_running()

# erobot.set_angle(1, -90, 500)
# time.sleep(0.1)
# while erobot.check_running():
#     print('running')
#     time.sleep(0.1)

# for i in range(3):
#     print(i)
#     # 读取位姿
#     cur_poses = erobot.get_coords()
#     print(cur_poses)
#     time.sleep(0.1)
#     # 设定位姿
#     erobot.set_coords([100.0, 320.0, 365.0, cur_poses[3], 0.0, -22.0], 1200)
#     time.sleep(0.1)
#     while erobot.check_running():
#         print('running')
#         time.sleep(0.1)
#     erobot.set_digital_out(0, 0)
#     time.sleep(0.5)


#     for _ in range(2):
#         for _ in range(100):
#             erobot.jog_coord('x',1,50)
#             erobot.jog_coord('y',1,50)
#             erobot.jog_coord('rz',1,50)
#             time.sleep(0.02)
#         for _ in range(100):
#             erobot.jog_coord('x',-1,50)
#             erobot.jog_coord('y',-1,50)
#             erobot.jog_coord('rz',-1,50)
#             time.sleep(0.02)
 
#     print("close the speed mode")
#     erobot.jog_stop('x')
#     erobot.jog_stop('y')
#     erobot.jog_stop('rz')
#     time.sleep(0.1)
#     # erobot.task_stop()
#     # time.sleep(0.5)


# # 速度控制
# erobot.jog_coord('x',1,1000)
# erobot.jog_coord('y',1,1000)
# erobot.jog_coord('rz',1,300)
# # erobot.jog_angle(6,1,500)
# time.sleep(2)
# erobot.jog_coord('x',-1,1000)
# erobot.jog_coord('y',-1,1000)
# erobot.jog_coord('rz',-1,300)
# # erobot.jog_angle(6,-1,500)
# time.sleep(2)
# erobot.jog_coord('x',0,200)
# erobot.jog_coord('y',0,200)
# erobot.jog_coord('rz',0,200)
# # erobot.jog_angle(6,0,500)
# time.sleep(0.2)

# erobot.jog_stop('x')
# erobot.jog_stop('y')
# erobot.jog_stop('rz')


# erobot = elephant_command()

# X0 = -173.0
# Y0 = 298.0
# Z0 = 320.0 # mm
# # RX0 = 180.0
# RY0 = 0.0
# RZ0 = -114.0
# ZG = 240.0 # mm

# # 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# erobot.set_coords([X0,Y0,Z0,cur_pose[3],RY0,RZ0], 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)

# time.sleep(2)
# # 笛卡尔空间
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[0] = -140
# cur_pose[5] -= 90
# erobot.set_coords(cur_pose, 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)

# time.sleep(2)
# cur_pose = erobot.get_coords()
# print(cur_pose)
# cur_pose[2] = ZG
# erobot.set_coords(cur_pose, 1000)
# time.sleep(0.1)
# while erobot.check_running():
#     # print('running', time.time())
#     time.sleep(0.1)


# I/O
# time.sleep(0.5)
# for _ in range(5):
#     erobot.set_digital_out(0,1)
#     time.sleep(1)
#     erobot.set_digital_out(0,0)
#     time.sleep(1)
# erobot.set_digital_out(0, 0)
# time.sleep(0.5)

# print(erobot.check_running())
# time.sleep(0.1)
