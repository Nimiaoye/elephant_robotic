#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    第四届中国高校智能机器人创意大赛
    主题二————俄罗斯方块
    AI俄罗斯方块算法

    最后修改：2021.8.26
"""
import sys
import pygame
import random
import thread
import time
import numpy as np
import matplotlib.pyplot as plt
import rospy as ros
from robot_tetris.msg import PlaceInfo
from robot_tetris.srv import AI_Tetris, AI_TetrisResponse, Solution, SolutionResponse

"""
    cd ros_workspace/src/robot_tetris/scripts/
    python tetris.py T 0 13 4
"""

pygame.init()

#定义各自的宽度，行列方向的格子数量
GRID_WIDTH = 40
GRID_NUM_WIDTH = 10
GRID_NUM_HEIGHT = 20

#根据格子数量计算可视框的宽度和高度
WIDTH, HEIGHT = GRID_WIDTH * GRID_NUM_WIDTH, GRID_WIDTH * GRID_NUM_HEIGHT
SIDE_WIDTH = 0
SCREEN_WIDTH = WIDTH + SIDE_WIDTH

# 定义常用颜色
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LINE_COLOR = (51, 51, 51)

# 定义颜色矩阵，用于生成不用颜色的俄罗斯方块
CUBE_COLORS = [
    (204, 153, 153), (255, 255, 153), (102, 102, 153),
    (153, 0, 102), (255, 204, 0), (204, 0, 51),
    (255, 0, 51), (0, 102, 153), (255, 255, 51),
    (153, 0, 51), (204, 255, 102), (255, 153, 0)]

COLORS = {'I':(255,0,0), 'O':(255,97,0), 'L':(255,255,0), 'J':(160,32,240),
          'S':(0,255,0), 'T':(128,42,42), 'Z':(0,0,255)}

screen = pygame.display.set_mode((SCREEN_WIDTH, HEIGHT))# 设置可视窗的宽高
pygame.display.set_caption("Tetris")# 设置标题

clock = pygame.time.Clock()# 根据帧数限制游戏运行速度
FPS = 30# 帧数
score = 0# 分数
level = 1# 等级

# 储存每个格子的状态，若格子未填充则为None，已填充的格子会变成该方块的颜色值
screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
init_screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]

SHAPES = ['I', 'J', 'L', 'O', 'S', 'T', 'Z']# 储存方块的名字的list
# 中心点的位置记为(0, 0)，(行，列)，向右一格列加一，向下一格行加一
I = [[(0, -2), (0, -1), (0, 0), (0, 1)], # [(0, -1), (0, 0), (0, 1), (0, 2)]
     [(-1, 0), (0, 0), (1, 0), (2, 0)]]
J = [[(-1, 0), (0, 0), (0, 1), (0, 2)], 
     [(-2, 0), (-1, 0), (0, 0), (0, -1)],
     [(0, -2), (0, -1), (0, 0), (1, 0)], 
     [(0, 1), (0, 0), (1, 0), (2, 0)]]
L = [[(1, 0), (0, 0), (0, 1), (0, 2)],
     [(-2, 0), (-1, 0), (0, 0), (0, 1)],
     [(0, -2), (0, -1), (0, 0), (-1, 0)],
     [(0, -1), (0, 0), (1, 0), (2, 0)]]
O = [[(0, 0), (0, 1), (1, 0), (1, 1)]]
S = [[(1, -1), (1, 0), (0, 0), (0, 1)],
     [(-1, 0), (0, 0), (0, 1), (1, 1)]]
T = [[(0, -1), (0, 0), (0, 1), (-1, 0)],
     [(-1, 0), (0, 0), (1, 0), (0, -1)],
     [(0, -1), (0, 0), (0, 1), (1, 0)],
     [(-1, 0), (0, 0), (1, 0), (0, 1)]]
Z = [[(0, -1), (0, 0), (1, 0), (1, 1)],
     [(1, 0), (0, 0), (0, 1), (-1, 1)]]

# 储存方块各种形态（各种变形）的对象
SHAPES_WITH_DIR = {'I': I, 'J': J, 'L': L, 'O': O, 'S': S, 'T': T, 'Z': Z}

# 方块的类
class CubeShape():
    def __init__(self, shape=None):
        if shape is None:
            self.shape = SHAPES[random.randint(0, len(SHAPES) - 1)]
        else:
            self.shape = shape
        self.center = [2, GRID_NUM_WIDTH // 2]# 骨牌中心所在的行列
        self.state = random.randint(0, len(SHAPES_WITH_DIR[self.shape]) - 1)
        self.color = CUBE_COLORS[random.randint(0, len(CUBE_COLORS) - 1)]

    def get_all_gridpos(self, center=None):
        """
            根据中心点坐标获取其他位置的点的坐标
        """
        curr_shape = SHAPES_WITH_DIR[self.shape][self.state]
        if center is None:
            center = [self.center[0], self.center[1]]

        return [[cube[0] + center[0], cube[1] + center[1]] for cube in curr_shape]

    def conflict(self, center=None):
        """
            碰撞检测，根据中心点找到其他点的位置，判断其他点是否合法
            合法的判断标准: 1) 是否超出边界  2) 是否已有元素
        """
        if center is None:
            center = self.center
        for cube in self.get_all_gridpos(center):
            # 超出屏幕范围，不合法
            if cube[0] < 0 or cube[1] < 0 or \
                cube[0] >= GRID_NUM_HEIGHT or cube[1] >= GRID_NUM_WIDTH:
                return True

            # 格子状态不为None，说明该位置已经有方块存在，不合法
            if screen_color_matrix[cube[0]][cube[1]] is not None:
                return True
        return False

    def rotate(self):
        """
            旋转，存储旧的形态（state），若新的形态不合法则状态倒退
        """
        new_state = self.state + 1
        new_state %= len(SHAPES_WITH_DIR[self.shape])
        old_state = self.state
        self.state = new_state
        if self.conflict(self.center):
            self.state = old_state
            return False

    def down(self):
        center = [self.center[0] + 1, self.center[1]]
        if self.conflict(center):
            return False
        self.center = center
        return True

    def left(self):
        center = [self.center[0], self.center[1] - 1]
        if self.conflict(center):
            return False
        self.center = center
        return True

    def right(self):
        center = [self.center[0], self.center[1] + 1]
        if self.conflict(center):
            return False
        self.center = center
        return True

    def draw(self):
        """
            绘制骨牌
        """
        for cube in self.get_all_gridpos():
           # 绘制小方格的颜色，实心正方形
            pygame.draw.rect(screen, self.color,
                             (cube[1] * GRID_WIDTH, cube[0] * GRID_WIDTH,
                              GRID_WIDTH, GRID_WIDTH))
           # 增加美观性，为每个小方格绘制一个白边，空心正方形
            pygame.draw.rect(screen, WHITE,
                             (cube[1] * GRID_WIDTH, cube[0] * GRID_WIDTH,
                              GRID_WIDTH, GRID_WIDTH), 1)

    def drawNext(self):
        """
            显示下一张骨牌
        """
        for cube in self.get_all_gridpos((0,0)):
            pygame.draw.rect(screen, self.color,
                             (cube[1] * GRID_WIDTH + WIDTH + 115, cube[0] * GRID_WIDTH + 180,
                              GRID_WIDTH, GRID_WIDTH))
            pygame.draw.rect(screen, WHITE,
                             (cube[1] * GRID_WIDTH + WIDTH + 115, cube[0] * GRID_WIDTH + 180,
                              GRID_WIDTH, GRID_WIDTH), 1)

class AITetris():
    def __init__(self, live_cube, matrix):
        self.shape = live_cube.shape
        self.state = live_cube.state
        self.color = live_cube.color
        self.matrix = matrix

    def get_all_gridpos(self, center, shape, state):
        curr_shape = SHAPES_WITH_DIR[shape][state]
        return [[cube[0] + center[0], cube[1] + center[1]] for cube in curr_shape]
    
    def conflict(self, center, shape, state, matrix):
        for cube in self.get_all_gridpos(center, shape, state):
            # 超出屏幕范围，该位置不合法
            if cube[0] < 0 or cube[1] < 0 or \
                cube[0] >= GRID_NUM_HEIGHT or  cube[1] >= GRID_NUM_WIDTH:
                return True
            # screen_color_matrix = self.copyMatrix(matrix)
            # 与已存在的骨牌冲突，该位置不合法
            if matrix[cube[0]][cube[1]] is not None:
                return True
        return False

    def copyMatrix(self, matrix):
        new_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
        for i in range(GRID_NUM_HEIGHT):
            for j in range(GRID_NUM_WIDTH):
                new_matrix[i][j] = matrix[i][j]
        return new_matrix
    
    def getNewMatrix(self, center, state):
        """
            填充可能的骨牌位置，形成新的颜色矩阵
        """
        new_matrix = self.copyMatrix(self.matrix)
        for cube in self.get_all_gridpos(center, self.shape, state):
            new_matrix[cube[0]][cube[1]] = self.color
        return new_matrix

    def getAllPossiblePos(self, shape):
        """
            获取所有可能的放置位置
        """
        matrix = self.matrix
        state_num = len(SHAPES_WITH_DIR[shape])
        all_position = []
        for state in range(state_num):
            for j in range(GRID_NUM_WIDTH):
                for i in range(GRID_NUM_HEIGHT):
                    if self.conflict([i + 1, j], shape, state, matrix) == True and \
                        self.conflict([i, j], shape, state, matrix) == False:
                        if {"center": [i, j], "state": state} not in all_position:
                            all_position.append({"center": [i, j], "state": state})
        return all_position

    def getHeight(self, center):
        """
            当前骨牌放置之后，其重心到底部的距离，该值越小越好
        """
        return GRID_NUM_HEIGHT - 1 - center[0]

    def getContribution(self, center, state):
        """
            消除的行数和自身贡献的方格数的乘积，越大越好
        """
        new_matrix = self.getNewMatrix(center, state)
        lines = 0
        useful_cubes = 0
        cubes = self.get_all_gridpos(center, self.shape, state)
        for i in range(GRID_NUM_HEIGHT - 1, 0, -1):
            count = 0
            for j in range(GRID_NUM_WIDTH):
                if new_matrix[i][j] is not None:
                    count += 1
            
            if count == GRID_NUM_WIDTH:
                lines += 1
                for k in range(GRID_NUM_WIDTH):
                    if [i,k] in cubes:
                        useful_cubes += 1
            # 若整行未填充，则跳出循环
            if count == 0:
                break
        return lines * useful_cubes

    def getRowTransition(self, matrix):
        """
            行变换数，从左往右，无填充->有填充/有填充->无填充 都是一种“变换”
        """
        transition = 0
        for i in range(GRID_NUM_HEIGHT - 1 , 0 , -1):
            for j in range(GRID_NUM_WIDTH - 1):
                if matrix[i][j] is None and matrix[i][j+1] is not None:
                    transition += 1
                if matrix[i][j] is not None and matrix[i][j+1] is None:
                    transition += 1
        return transition

    def getColTransition(self, matrix):
        """
            列变换数
        """
        transition = 0
        for j in range(GRID_NUM_WIDTH):
            for i in range(GRID_NUM_HEIGHT - 1, 1, -1):
                if matrix[i][j] is None and matrix[i-1][j] is not None:
                    transition += 1
                if matrix[i][j] is not None and matrix[i-1][j] is None:
                    transition += 1
        return transition

    def getHoles(self, matrix):
        """
            获取空洞数量
        """
        holes = 0
        for j in range(GRID_NUM_WIDTH):
            colHoles = None
            for i in range(GRID_NUM_HEIGHT):
                # 排除空列
                if colHoles is None and matrix[i][j] is not None:
                    colHoles = 0
                if colHoles is not None and matrix[i][j] is None:
                    colHoles += 1
            if colHoles is not None:
                holes += colHoles
        return holes

    def getWells(self, matrix):
        """
            各列中井的深度的连加和 井:两边（包括边界）都有填充的空列
        """
        sum_n = [0,1,3,6,10,15,21,28,36,45,55,66,78,91,105]
        wells = 0
        sum = 0
        for j in range(GRID_NUM_WIDTH):
            for i in range(GRID_NUM_HEIGHT):
                if matrix[i][j] == None:
                    if (j-1<0 or matrix[i][j-1] != None) and (j+1 >= GRID_NUM_WIDTH or matrix[i][j+1] != None):
                        wells += 1
            sum += sum_n[wells]
            wells = 0
        return sum

    def getPriority(self, position):
        """
            计算优先度  priority = 100 * 骨牌需要水平移动的次数 + 骨牌状态改变的次数
        """
        col_change_num = abs(GRID_NUM_WIDTH // 2 - position['center'][1])
        tar_state = position['state']
        curr_state = self.state
        if tar_state >= curr_state:
            state_change = tar_state - curr_state
        else :
            state_change = len(SHAPES_WITH_DIR[self.shape]) - curr_state + tar_state
        priority = col_change_num * 100 + state_change
        return priority
    
    def evaluate(self, position):
        """
            Pierre Dellacherie 算法评估函数，value值最大的为最优位置
        """
        new_matrix = self.getNewMatrix(position['center'], position['state'])
        lh = self.getHeight(position['center'])
        epcm = self.getContribution(position['center'], position['state'])
        brt = self.getRowTransition(new_matrix)
        bct = self.getColTransition(new_matrix)
        bbh = self.getHoles(new_matrix)
        bw = self.getWells(new_matrix)

        score = -35*lh + 34*epcm - 32*brt - 38*bct - 500* bbh -34*bw
        # score = -5*lh + 34*epcm - 0*brt - 0*bct - 500* bbh
        # score = -1*lh + epcm - brt - bct - 4*bbh - bw # PD算法原始设计

        return score

    def mainProcess(self):
        """
            AI 俄罗斯方块的主要执行逻辑
        """
        possible_pos = self.getAllPossiblePos(self.shape)
        best_score = -999999
        best_position = None
        for position in possible_pos:
            score = self.evaluate(position)
            if score > best_score:
                best_score = score
                best_position = position
            elif score == best_score:
                if self.getPriority(position) < self.getPriority(best_position):
                    best_score = score
                    best_position = position
        return best_position

def draw_grids():
    for i in range(GRID_NUM_WIDTH):
        pygame.draw.line(screen, LINE_COLOR,
                         (i * GRID_WIDTH, 0), (i * GRID_WIDTH, HEIGHT))

    for i in range(GRID_NUM_HEIGHT):
        pygame.draw.line(screen, LINE_COLOR,
                         (0, i * GRID_WIDTH), (WIDTH, i * GRID_WIDTH))

    pygame.draw.line(screen, WHITE,
                     (GRID_WIDTH * GRID_NUM_WIDTH, 0),
                     (GRID_WIDTH * GRID_NUM_WIDTH, GRID_WIDTH * GRID_NUM_HEIGHT))

def draw_matrix():
    """
        根据颜色矩阵绘制屏幕
    """
    for i, row in zip(range(GRID_NUM_HEIGHT), screen_color_matrix):
        for j, color in zip(range(GRID_NUM_WIDTH), row):
            if color is not None:
                pygame.draw.rect(screen, color,
                            (j * GRID_WIDTH, i * GRID_WIDTH,
                             GRID_WIDTH, GRID_WIDTH))
                pygame.draw.rect(screen, WHITE,
                            (j * GRID_WIDTH, i * GRID_WIDTH,
                             GRID_WIDTH, GRID_WIDTH), 2)

def remove_full_line():
    """
        满行消除和计分
    """
    global screen_color_matrix, score, level
    k = [0,1,3,6,10,15,21,28,36,45,55]
    new_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
    index = GRID_NUM_HEIGHT - 1
    n_full_line = 0
    for i in range(GRID_NUM_HEIGHT - 1, -1, -1):
        is_full = True
        for j in range(GRID_NUM_WIDTH):
            if screen_color_matrix[i][j] is None:
                is_full = False
                break
        if not is_full:
            new_matrix[index] = screen_color_matrix[i]
            index -= 1
        else:
            n_full_line += 1

    score += k[n_full_line]
    level = score // 30 + 1# 计算等级
    screen_color_matrix = new_matrix# 消除一行之后的新的矩阵赋值给矩阵

def show_text(text, size, x, y, color=WHITE):
    font_name = r'D:\Code_exercise\Robot_Contest\font\simkai.ttf'
    font = pygame.font.Font(font_name, size)
    text_surface = font.render(text, True, color)
    text_rect = text_surface.get_rect()
    text_rect.midtop = (x, y)
    screen.blit(text_surface, text_rect)

def show_info():
    show_text('Score:{}'.format(score), 20, WIDTH + SIDE_WIDTH // 2, int(HEIGHT * 0.75))
    show_text('Level:{}'.format(level), 20, WIDTH + SIDE_WIDTH // 2, int(HEIGHT * 0.75) - 30)
    show_text('Next:', 20, WIDTH + SIDE_WIDTH // 2, 80)

def show_welcome():
    show_text('Tetris', 30, WIDTH / 2, HEIGHT / 2)
    show_text('Press any key to start', 20, WIDTH / 2, HEIGHT / 2 + 50)

def main():
    global screen_color_matrix
    running = True
    AI_mdoe = False
    PAUSE = False
    game_over = True
    counter = 0 # 控制下落速度
    live_cube = None
    new_cube = None

    while running:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if game_over:
                    game_over = False
                    live_cube = CubeShape() # 首次生成骨牌
                    new_cube = CubeShape()
                    break
                # 上下左右键盘操作
                if event.key == pygame.K_LEFT:
                    live_cube.left()
                elif event.key == pygame.K_RIGHT:
                    live_cube.right()
                elif event.key == pygame.K_DOWN:
                    live_cube.down()
                elif event.key == pygame.K_UP:
                    live_cube.rotate()
                # 按空格快速下落
                elif event.key == pygame.K_SPACE:
                    while live_cube.down():
                        pass
                elif event.key == pygame.K_a:
                    if AI_mdoe:
                        AI_mdoe = False
                    else:
                        AI_mdoe = True
                elif event.key == pygame.K_p:
                    if PAUSE:
                        PAUSE = False
                    else:
                        PAUSE = True
                elif event.key == pygame.K_ESCAPE:
                    pygame.quit()

        if PAUSE is False and game_over is False and counter % (FPS // level) == 0:
            if AI_mdoe:
                ai_process = AITetris(live_cube, screen_color_matrix)
                best_position = ai_process.mainProcess()
                live_cube.state = best_position['state']
                live_cube.center = best_position['center']

            if live_cube.down() == False:
                for cube in live_cube.get_all_gridpos():
                    screen_color_matrix[cube[0]][cube[1]] = live_cube.color

                # live_cube = CubeShape() 
                live_cube = new_cube
                new_cube = CubeShape()
                # 上一骨牌放置完毕，生成新骨牌
                # 若生成新骨牌后随即发生碰撞，则游戏结束
                if live_cube.conflict():
                    game_over = True
                    live_cube = None
                    new_cube = None
                    score = 0
                    screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
            remove_full_line()
        
        counter += 1# 每次执行while循环，计数器+1
        screen.fill(BLACK)# 更新屏幕
        draw_grids()
        draw_matrix()
        show_info()
        if live_cube is not None:
            live_cube.draw()
        if new_cube is not None:
            new_cube.drawNext()
        if game_over:
            show_welcome()
        pygame.display.update()# 刷新屏幕

####################################################################################
curr_domino = None # 正在抓取的骨牌

def srv_callback(req):
    print('AI_tetris service callback...')
    shape = req.type # 骨牌的类型
    best_position = robot_tetris_main(shape)
    state = best_position['state']
    center = best_position['center']
    print(center, state)
    return AI_TetrisResponse(center, state)

def robot_draw_matrix():
    """
        可视化当前的机器人俄罗斯方块拼接情况
    """
    running = True
    while running:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(BLACK)# 更新屏幕
        draw_grids()
        draw_matrix()
        # show_text('Current:', 20, WIDTH + SIDE_WIDTH // 2, 80)
        # if curr_domino is not None:
        #     curr_domino.drawNext()
        pygame.display.update()# 刷新屏幕

def robot_tetris_main(shape):
    global screen_color_matrix, curr_domino
    curr_domino = CubeShape(shape)

    ai_process = AITetris(curr_domino, screen_color_matrix)
    best_position = ai_process.mainProcess()
    curr_domino.state = best_position['state']
    curr_domino.center = best_position['center']

    for cube in curr_domino.get_all_gridpos():
        # screen_color_matrix[cube[0]][cube[1]] = curr_domino.color
        screen_color_matrix[cube[0]][cube[1]] = COLORS[shape]

    return best_position

def ai_tetris_server():
    """
        调用算法服务，返回当前类型骨牌的最佳放置位置
    """
    ros.init_node('AI_Tetris_node')
    ros.loginfo('Start AI_Tetris_server node...')
    server = ros.Service('/ai_tetris', AI_Tetris, srv_callback)

    thread.start_new_thread(robot_draw_matrix, ())
    ros.spin()
######################################################################################

# 决赛 初始方块 T 0 13 4
SOLUTION = [['Z', 0, [18, 6]], ['S', 0, [18, 2]], ['Z', 1, [18, 0]], ['T', 3, [16, 0]], ['L', 2, [17, 4]], ['L', 1, [17, 5]], 
            ['J', 1, [19, 9]], ['Z', 0, [15, 2]], ['L', 1, [18, 7]], ['O', 0, [14, 3]], ['O', 0, [13, 0]], ['O', 0, [11, 0]], 
            ['I', 1, [12, 2]], ['Z', 1, [16, 8]], ['O', 0, [12, 3]], ['S', 0, [14, 8]], ['L', 3, [14, 6]], ['L', 0, [13, 7]], 
            ['Z', 0, [10, 3]], ['S', 0, [12, 6]], ['I', 1, [10, 5]], ['J', 1, [12, 9]], ['S', 1, [9, 3]], ['J', 0, [11, 6]], 
            ['I', 1, [8, 0]], ['J', 2, [8, 6]], ['O', 0, [9, 7]], ['T', 3, [9, 1]], ['I', 1, [7, 9]], ['T', 2, [7, 2]], 
            ['I', 0, [6, 2]], ['T', 0, [7, 5]], ['J', 1, [8, 8]], ['S', 1, [6, 6]]]

def draw_solution():
    global screen_color_matrix, SOLUTION
    for item in SOLUTION:
        shape = item[0]
        domino = CubeShape(shape)
        domino.state = item[1]
        domino.center = item[2]

        for cube in domino.get_all_gridpos():
            # screen_color_matrix[cube[0]][cube[1]] = domino.color
            screen_color_matrix[cube[0]][cube[1]] = COLORS[shape]

#################################### 遗传算法 ##################################################
DNA_SIZE = 3
POP_SIZE = 50
CROSSOVER_RATE = 0.8
MUTATION_RATE = 0.005
N_GENERATIONS = 50


def F(population):
    """
    目标函数：
        各列高度的方差、空洞数量、未在规定区域内的方格数
    """
    global screen_color_matrix

    value = []
    solution = [] # 保存目标函数值最小的解
    for individual in population:
        screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
        for shape in individual:
            best_position = robot_tetris_main(shape)
        num = 140
        for row in range(6, GRID_NUM_HEIGHT):
            for col in range(GRID_NUM_WIDTH):
                if screen_color_matrix[row][col] is None:
                    num -= 1
        value.append(num)
    return value

def translateDNA(group_DNA): #group表示种群DNA矩阵，一行表示一个二进制编码表示的个体DNA
    pop_trans = []
    for row in group_DNA:
        individual = []
        for i in range(35):
            gene = row[3*i : 3*i+3]
            index = (gene[0]*(2**2) + gene[1]*(2**1) + gene[2]*(2**0)) % 7 #形状编码0 - 7
            individual.append(SHAPES[index])
        pop_trans.append(individual)
    return pop_trans

def get_fitness(population): 
    pop_trans = translateDNA(population)
    value = F(pop_trans)
    return value / np.sum(value)

def mutation(individual, MUTATION_RATE=0.003):
	if np.random.rand() < MUTATION_RATE: 				#以MUTATION_RATE的概率进行变异
		mutate_point = np.random.randint(0, DNA_SIZE*35)	#随机产生一个实数，代表要变异基因的位置
		individual[mutate_point] = individual[mutate_point]^1 	#将变异点的二进制为反转

def crossover_and_mutation(pop, CROSSOVER_RATE = 0.8):
	new_pop = []
	for father in pop:		#遍历种群中的每一个个体，将该个体作为父亲
		child = father		#孩子先得到父亲的全部基因
		if np.random.rand() < CROSSOVER_RATE:			#产生子代时不是必然发生交叉，而是以一定的概率发生交叉
			mother = pop[np.random.randint(POP_SIZE)]	#再种群中选择另一个个体，并将该个体作为母亲
			cross_points = np.random.randint(0, DNA_SIZE*35)	#随机产生交叉的点
			child[cross_points:] = mother[cross_points:]		#孩子得到位于交叉点后的母亲的基因
		mutation(child)	#每个后代有一定的机率发生变异
		new_pop.append(child)
	return new_pop

def select(population, fitness):
    idx = np.random.choice(np.arange(POP_SIZE), size=POP_SIZE, replace=True,
                           p=fitness/np.sum(fitness) )
    return population[idx]

def print_info(population):
	fitness = get_fitness(population)
	max_fitness_index = np.argmax(fitness)
	print("max_fitness:", fitness[max_fitness_index])
	pop_trans = translateDNA(population)
	# print("最优的基因型：", population[max_fitness_index])
	print("最优顺序:", pop_trans[max_fitness_index])


def genetic_algorithm():
    global screen_color_matrix
    # _thread.start_new_thread(robot_draw_matrix, ())

    population = np.random.randint(2, size=(POP_SIZE, DNA_SIZE*35))
    for generation in range(N_GENERATIONS):#迭代N代
        population = np.array(crossover_and_mutation(population, CROSSOVER_RATE))
        fitness = get_fitness(population)
        population = select(population, fitness) #选择生成新的种群
	
    print_info(population)
################################################################################################
######################################### 模拟退火 #############################################

def objective_func(shape_sequence):
    """
    目标函数：各列高度的方差、空洞数量、未在规定区域内的方格数
    优化目标：求最小值
    """
    global screen_color_matrix
    screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
    first_domino = CubeShape(sys.argv[1])
    first_domino.state = int(sys.argv[2])
    first_domino.center = [int(sys.argv[3]) + 6, int(sys.argv[4])]
    for cube in first_domino.get_all_gridpos():
        screen_color_matrix[cube[0]][cube[1]] = WHITE

    # robot_draw_matrix()

    solution = [] # 记录整个形状序列的具体放置信息
    heights = [] # 记录每一列的高度
    holes = 0
    num = 0 # 记录在规定区域外面的方格数量
    for shape in shape_sequence:
        best_position = robot_tetris_main(shape)
        solution.append([shape, best_position['state'], best_position['center']])
    
    for col in range(GRID_NUM_WIDTH):
        col_holes = None # 记录空洞数量
        for row in range(GRID_NUM_HEIGHT):
            if row < 6 and screen_color_matrix[row][col] is not None:
                num += 1
            if col_holes is None and screen_color_matrix[row][col] is not None:
                # 找到某一列的最高处
                heights.append(20 - row)
                col_holes = 0 # 开始计数空洞数量
            if col_holes is not None and screen_color_matrix[row][col] is  None:
                col_holes += 1
        if col_holes is not None:
            holes += col_holes
    height_var = np.var(heights)
    value = holes + height_var * 10 + num
    return value, solution

def judge(delta_E, T):
    """
    判断函数，返回1表示接受当前解，返回0表示拒绝
    若目标函数值减小，则接受该解；否则以一定概率接受该解
    """
    if delta_E < 0:
        return 1
    else:
        if np.exp(-delta_E / T) > np.random.rand():
            return 1
        else:
            return 0

def disturbance(shape_seq):
    #为当前解添加随机扰动
    shape_seq_new = shape_seq
    if np.random.rand() > 0.5:
        # 一定概率生成全新的序列
        np.random.shuffle(shape_seq_new)
    else:
        # 一定概率进行交叉
        pos_1 = np.random.randint(0, len(shape_seq))
        pos_2 = np.random.randint(0, len(shape_seq))
        shape_1 = shape_seq_new[pos_1]
        shape_seq_new[pos_1] = shape_seq_new[pos_2]
        shape_seq_new[pos_2] = shape_1
    return shape_seq_new

def simulated_anneal(shape_seq_init):
    global screen_color_matrix
    temperature  = 1e4
    T_min = 1e-3
    alpha = 0.95

    shape_seq = shape_seq_init
    value_old = objective_func(shape_seq)[0]
    value_new = value_old
    
    k = 0
    counter = 0
    record_value = []
    
    while(temperature > T_min and counter <= 10100):
        FLAG = True
        k += 1
        print(k)
        shape_seq_new = disturbance(shape_seq)
        value_new = objective_func(shape_seq_new)[0]

        # if value_new < 6:
        #     record_value.append(value_new)
        #     shape_seq = shape_seq_new
        #     break

        for row in range(7, GRID_NUM_HEIGHT):
            for col in range(GRID_NUM_WIDTH):
                if screen_color_matrix[row][col] is None:
                    FLAG = False
                    break
            if FLAG == False:
                break
        if FLAG and value_new < 5:
            record_value.append(value_new)
            shape_seq = shape_seq_new
            break

        delta_E = value_new - value_old
        if judge(delta_E, temperature) == 1:
            value_old = value_new
            record_value.append(value_new)
            shape_seq = shape_seq_new
        if delta_E < 0:
            temperature = temperature * alpha #说明新解较为理想，继续沿着降温的方向寻找，减少跳出可能性；当温度减小，当前内能变化量的概率值会变小
        else:
            counter += 1
    solution = objective_func(shape_seq)[1]
    
    # print('record_value', record_value)
    # print('value', value_new)
    # print('solution',solution)
    
    for row in range(GRID_NUM_HEIGHT):
        for col in range(GRID_NUM_WIDTH):
            if screen_color_matrix[row][col] is None:
                screen_color_matrix[row][col] = (0,0,0)

    # robot_draw_matrix()
    # plt.imshow(screen_color_matrix)
    # plt.show()
    return solution
#################################################################################################

def random_search(shape_sequence):
    global screen_color_matrix
    for i in range(5000):
        print(i+1)
        FLAG = True
        screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]
        np.random.shuffle(shape_sequence)
        for shape in shape_sequence:
            robot_tetris_main(shape)

        for row in range(GRID_NUM_HEIGHT):
            for col in range(GRID_NUM_WIDTH):
                if screen_color_matrix[row][col] is None:
                    screen_color_matrix[row][col] = (0,0,0)
                    if row > GRID_NUM_HEIGHT - 13 - 1:
                        FLAG = False
        if FLAG:     
            print('successful')
            plt.imshow(screen_color_matrix)
            plt.show()
            break


global_solution = [] # 保存搜索到的全局拼接方案

def solution_srv_callback(req):
    print('Solution service callback...')
    response = SolutionResponse()
    response.result = 'successful'
    for place_info in global_solution:
        response.solution.append(PlaceInfo(place_info[0], place_info[1], place_info[2]))

    return response

def solution():
    """
    初始盘面为空，搜索拼接方案
    """
    shape_sequence = []
    for shape in SHAPES:
        for i in range(5):
            shape_sequence.append(shape)
    
    np.random.shuffle(shape_sequence)
    solution = simulated_anneal(shape_sequence)
    # random_search(shape_sequence)

    print('solution',solution)
    robot_draw_matrix()
    
def solution_1():
    """
    初始盘面不为空，通过命令行参数获取第一个方块的位置信息，继而搜索后续拼接方案
    """
    global screen_color_matrix, global_solution
    first_domino = CubeShape(sys.argv[1])
    first_domino.state = int(sys.argv[2])
    first_domino.center = [int(sys.argv[3]) + 6, int(sys.argv[4])]
    for cube in first_domino.get_all_gridpos():
        screen_color_matrix[cube[0]][cube[1]] = WHITE # 初始方块

    # 决赛 初始方块 T 0 13 4
    if sys.argv[1] == 'T' and first_domino.state == 0 \
        and first_domino.center == [19, 4]:

        ros.sleep(5)
        global_solution = SOLUTION
        draw_solution()
    else:
        shape_sequence = []
        for shape in SHAPES:
            for i in range(5):
                if shape == sys.argv[1] and i == 4:
                    continue
                shape_sequence.append(shape)
        np.random.shuffle(shape_sequence)
        global_solution = simulated_anneal(shape_sequence)
        
    print('global_solution: ', global_solution)
    ros.init_node('AI_Tetris_node')
    ros.loginfo('Start AI_Tetris_server node...')
    server = ros.Service('/solution', Solution, solution_srv_callback)

    thread.start_new_thread(robot_draw_matrix, ())
    ros.spin()
    


if __name__ == '__main__':
    # main()
    # ai_tetris_server()

    # genetic_algorithm()

    # solution()
    solution_1()

    
