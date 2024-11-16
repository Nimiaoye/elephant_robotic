#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    第四届中国高校智能机器人创意大赛
    主题二————俄罗斯方块
"""
import pygame
import random
import thread
import rospy as ros
from robot_tetris.srv import AI_Tetris, AI_TetrisResponse

pygame.init()

#定义各自的宽度，行列方向的格子数量
GRID_WIDTH = 20
GRID_NUM_WIDTH = 15
GRID_NUM_HEIGHT = 25

#根据格子数量计算可视框的宽度和高度
WIDTH, HEIGHT = GRID_WIDTH * GRID_NUM_WIDTH, GRID_WIDTH * GRID_NUM_HEIGHT
SIDE_WIDTH = 200
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


screen = pygame.display.set_mode((SCREEN_WIDTH, HEIGHT))# 设置可视窗的宽高
pygame.display.set_caption("Tetris")# 设置标题
clock = pygame.time.Clock()# 根据帧数限制游戏运行速度
FPS = 30# 帧数
score = 0# 分数
level = 1# 等级

# 储存每个格子的状态，若格子未填充则为None，已填充的格子会变成该方块的颜色值
screen_color_matrix = [[None] * GRID_NUM_WIDTH for i in range(GRID_NUM_HEIGHT)]

SHAPES = ['I', 'J', 'L', 'O', 'S', 'T', 'Z']# 储存方块的名字的list
# 中心点的位置记为(0, 0)，(行，列)，向右一格列加一，向下一格行加一
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
                             (cube[1] * GRID_WIDTH + WIDTH + 85, cube[0] * GRID_WIDTH + 150,
                              GRID_WIDTH, GRID_WIDTH))
            pygame.draw.rect(screen, WHITE,
                             (cube[1] * GRID_WIDTH + WIDTH + 85, cube[0] * GRID_WIDTH + 150,
                              GRID_WIDTH, GRID_WIDTH), 1)

class AI_Tetris():
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
        sum_n = [0,1,3,6,10,15,21,28,36,45,55]
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

        score = -35*lh + 34*epcm - 72*brt - 78*bct - 79* bbh -34*bw # [-45,34,-32,-98,-79,-34]
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

    score += n_full_line
    level = score // 20 + 1# 计算等级
    screen_color_matrix = new_matrix# 消除一行之后的新的矩阵赋值给矩阵

def show_text(text, size, x, y, color=WHITE):
    font_name = '/home/zyh/ros_workspace/src/robot_tetris/font/simkai.ttf'
    font = pygame.font.Font(font_name, size)
    text_surface = font.render(text, True, color)
    text_rect = text_surface.get_rect()
    text_rect.midtop = (x, y)
    screen.blit(text_surface, text_rect)

def show_info():
    show_text('Score:{}'.format(score), 20, WIDTH + SIDE_WIDTH // 2, 400)
    show_text('Next:', 20, WIDTH + SIDE_WIDTH // 2, 80)

def show_welcome():
    show_text('Tetris', 30, WIDTH / 2, HEIGHT / 2)
    show_text('Press any key to start', 20, WIDTH / 2, HEIGHT / 2 + 50)

curr_domino = None # 正在抓取的骨牌

def srvCallBack(req):
    print('AI_tetris service processing...')
    shape = req.type # 骨牌的类型
    best_position = robot_tetris_main(shape)
    state = best_position['state']
    center = best_position['center']
    return AI_TetrisResponse(center, state)

def ai_tetris_server():
    """
        调用算法服务，返回当前类型骨牌的最佳放置位置
    """
    ros.init_node('ai_tetris_node')
    ros.loginfo('Start ai_tetris_node...')
    server = ros.Service('/ai_tetris', AI_Tetris, srvCallBack)

    thread.start_new_thread(robot_draw_matrix, ())
    ros.spin()

def robot_draw_matrix():
    """
        可视化当前的机器人俄罗斯方块拼接情况
    """
    while True:
        screen.fill(BLACK)# 更新屏幕
        draw_grids()
        draw_matrix()
        show_info()
        if curr_domino is not None:
            curr_domino.drawNext()
        pygame.display.update()# 刷新屏幕

def main():
    global screen_color_matrix
    running = True
    AI_mdoe = False
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
                elif event.key == pygame.K_ESCAPE:
                    pygame.quit()
                # remove_full_line()

        if game_over is False and counter % (FPS // level) == 0:
            if AI_mdoe:
                ai_process = AI_Tetris(live_cube, screen_color_matrix)
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

def robot_tetris_main(shape):
    global screen_color_matrix
    curr_domino = CubeShape(shape)

    ai_process = AI_Tetris(curr_domino, screen_color_matrix)
    best_position = ai_process.mainProcess()
    curr_domino.state = best_position['state']
    curr_domino.center = best_position['center']

    for cube in curr_domino.get_all_gridpos():
        screen_color_matrix[cube[0]][cube[1]] = curr_domino.color

    return best_position


if __name__ == '__main__':
    # main()

    ai_tetris_server()