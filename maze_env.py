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


UNIT = 40   # pixels
MAZE_H = 12  # grid height
MAZE_W = 12  # grid width


class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self._build_maze()
        self.fitness = [200]
        self.rect2 = []
        self.lenth = 0

    def creat_barrier(self,origin,abscissa,ordinate):
        barrier_center = origin + np.array([UNIT * abscissa,UNIT * ordinate])
        self.barrier = self.canvas.create_rectangle(
            barrier_center[0] - 15, barrier_center[1] - 15,
            barrier_center[0] + 15, barrier_center[1] + 15,
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
        origin = np.array([20, 20])

        # barrier
        self.barrier1 = []
        for i in range(12):
            for j in range(12):
                if(i==0 or i==1 or i==11):
                    self.barrier1.append(self.creat_barrier(origin, j, i))
                if((i==2) or (i==3) or (i==4) or (i==5) or (i==6) or (i==7) or (i==8)):
                    self.barrier1.append(self.creat_barrier(origin, 0, i))
                    self.barrier1.append(self.creat_barrier(origin, 1, i))
                    self.barrier1.append(self.creat_barrier(origin, 10, i))
                    self.barrier1.append(self.creat_barrier(origin, 11, i))
                if(i==9):
                    self.barrier1.append(self.creat_barrier(origin, 0, i))
                    self.barrier1.append(self.creat_barrier(origin, 3, i))
                    self.barrier1.append(self.creat_barrier(origin, 11, i))
                    self.barrier1.append(self.creat_barrier(origin, 10, i))
                if(i==10):
                    self.barrier1.append(self.creat_barrier(origin, 0, i))
                    self.barrier1.append(self.creat_barrier(origin, 2, i))
                    self.barrier1.append(self.creat_barrier(origin, 3, i))
                    self.barrier1.append(self.creat_barrier(origin, 4, i))
                    self.barrier1.append(self.creat_barrier(origin, 7, i))
                    self.barrier1.append(self.creat_barrier(origin, 8, i))
                    self.barrier1.append(self.creat_barrier(origin, 9, i))
                    self.barrier1.append(self.creat_barrier(origin, 11, i))
                    self.barrier1.append(self.creat_barrier(origin, 10, i))
        self.m = len(self.barrier1)

        self.cobarrier = []
        for i in range(self.m):
            #print('coload')
            self.cobarrier.append(self.canvas.coords(self.barrier1[i]))

        # create red rect
        start = origin + np.array([UNIT * 5, UNIT * 10])
        self.rect = self.canvas.create_rectangle(
            start[0] - 15, start[1] - 15,
            start[0] + 15, start[1] + 15,
            fill='red')
        
        # create terminus
        terminus = origin + np.array([UNIT * 1,UNIT * 9])
        self.terminus = self.canvas.create_rectangle(
            terminus[0] - 15, terminus[1] - 15,
            terminus[0] + 15, terminus[1] + 15,
            fill='green')

        # pack all
        self.canvas.pack()

    def reset(self):
        self.update()
        time.sleep(0.001)
        self.canvas.delete(self.rect)
        origin = np.array([20, 20])
        start = origin + np.array([UNIT * 5, UNIT * 10])
        l = len(self.rect2)
        if min(self.fitness) < l:
            for i in range(l):
                self.canvas.delete(self.rect2[i])
        self.lenth = 0
        self.rect = self.canvas.create_rectangle(
            start[0] - 15, start[1] - 15,
            start[0] + 15, start[1] + 15,
            fill='red')
        # return observation
        return self.canvas.coords(self.rect)

    def step(self, action):
        s = self.canvas.coords(self.rect)
        base_action = np.array([0, 0])
        self.lenth += 1
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

        self.rect2.append(self.canvas.create_rectangle(s[0], s[1], s[2], s[3], fill='yellow'))

        s_ = self.canvas.coords(self.rect)  # next state

        # reward function
       
        if s_ in self.cobarrier:
            reward = -10
            done = True
            s_ = 'terminal'
        elif s_ == self.canvas.coords(self.terminus):
            reward = 50
            done = True
            s_ = 'terminal'
            self.fitness.append(self.lenth)
        else:
            reward = -1
            done = False

        return s_, reward, done

    def render(self):
        time.sleep(0)
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