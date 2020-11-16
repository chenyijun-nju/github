"""
Reinforcement learning maze example.
Red rectangle:          explorer.
Black rectangles:       hells       [reward = -1].
Yellow bin circle:      paradise    [reward = +1].
All other states:       ground      [reward = 0].
This script is the environment part of this example. The RL is in RL_brain.py.
View more on my tutorial page: https://morvanzhou.github.io/tutorials/
"""

import numpy as np
import time
import sys
if sys.version_info.major == 2:
    import Tkinter as tk
else:
    import tkinter as tk


UNIT = 20   # pixels
MAZE_H = 22  # grid height
MAZE_W = 22  # grid width


class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('maze2')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self._build_maze()
        self.fitness = []

    def creat_barrier(self,origin,abscissa,ordinate):
        barrier_center = origin + np.array([UNIT * abscissa,UNIT * ordinate])
        self.barrier = self.canvas.create_rectangle(
            barrier_center[0] - 7, barrier_center[1] - 7,
            barrier_center[0] + 7, barrier_center[1] + 7,
            fill='black')
        return self.barrier

    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # create grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_W * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        # create origin
        origin = np.array([10, 10])

        # barrier
        self.barrier1 = []
        for i in range(22):
            for j in range(22):
                if(i==0 or i==21):
                    self.barrier1.append(self.creat_barrier(origin, j, i))
                elif(i==1):
                    for k in [0,8,9,10,13,17,18,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==2):
                    for k in [0,5,8,10,18,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==3):
                    for k in [0,4,5,6,8,13,14,16,17,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==4):
                    for k in [0,1,5,6,7,9,11,14,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==5):
                    for k in [0,1,9,11,14,15,16,17,18,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==6):
                    for k in range(5):
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                    for k in [6,7,9,12,20,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==7):
                    for k in range(12, 18):
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                    for k in [0,2,3,7,20,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==8):
                    for k in range(5, 9):
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                    for k in [0,3,11,12,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==9):
                    for k in [0, 8, 12, 16, 17, 19,20,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==10):
                    for k in [0, 20, 21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                    for k in range(8, 14):
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==11):
                    for k in [0,3,4,8,11,16,17,18,20,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==12):
                    for k in [0,4,5,8,9,11,13,18,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==13):
                    for k in [0,1,4,9,13,16,17,18,19,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==14):
                    for k in [0,4,5,6,11,12,13,19,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==15):
                     for k in [0,2,4,8,11,19,21]:
                         self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==16):
                    for k in [0,3,6,7,8,11,14,17,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==17):
                    for k in [0,6,8,9,13,14,18,20,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==18):
                    for k in [0,3,4,9,12,19,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==19):
                    for k in [0,2,9,11,13,14,17,18,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
                elif(i==20):
                    for k in [0,4,5,9,15,21]:
                        self.barrier1.append(self.creat_barrier(origin, k, i))
        self.m = len(self.barrier1)

        self.cobarrier = []
        for i in range(self.m):
            self.cobarrier.append(self.canvas.coords(self.barrier1[i]))        
        
        # create red rect
        start = origin + np.array([UNIT * 1, UNIT * 20])
        self.rect = self.canvas.create_rectangle(
            start[0] - 7, start[1] - 7,
            start[0] + 7, start[1] + 7,
            fill='red')
       
        # create terminus
        terminus = origin + np.array([UNIT * 19,UNIT * 1])
        self.terminus = self.canvas.create_rectangle(
            terminus[0] - 7, terminus[1] - 7,
            terminus[0] + 7, terminus[1] + 7,
            fill='green')

        # pack all
        self.canvas.pack()

    def reset(self):
        self.update()
        time.sleep(0.001)
        self.canvas.delete(self.rect)
        origin = np.array([10, 10])
        start = origin + np.array([UNIT * 1, UNIT * 20])
        self.lenth = 0
        self.rect = self.canvas.create_rectangle(
            start[0] - 7, start[1] - 7,
            start[0] + 7, start[1] + 7,
            fill='red')
        # return observation
        return self.canvas.coords(self.rect)

    def step(self, action):
        self.lenth += 1
        s = self.canvas.coords(self.rect)
        base_action = np.array([0, 0])
        if action == 0:   # up
            if s[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:   # down
            if s[1] < (MAZE_H - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:   # right
            if s[0] < (MAZE_W - 1) * UNIT:
                base_action[0] += UNIT
        elif action == 3:   # left
            if s[0] > UNIT:
                base_action[0] -= UNIT

        self.canvas.move(self.rect, base_action[0], base_action[1])  # move agent

        s_ = self.canvas.coords(self.rect)  # next state

        # reward function
        
        if s_ in self.cobarrier:
            reward = -30
            done = True
            s_ = 'terminal'

        elif s_ == self.canvas.coords(self.terminus):
            print('we arrived!')
            reward = 50
            done = True
            s_ = 'terminal'
            self.fitness.append(self.lenth)

        else:
            reward = -1
            done = False

        return s_, reward, done

    def render(self):
        time.sleep(0.001)
        self.update()


def update():
    for t in range(10):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s, r, done = env.step(a)
            if done:
                break

if __name__ == '__main__':
    env = Maze()
    env.after(100, update)
    env.mainloop()