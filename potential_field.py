import matplotlib.pyplot as plt
import numpy as np
"""
    author: Dhilan IM

    Path planning for  mobile robot using Artificial Potential Field  approach with no sensors

"""

class Robot:
    # Diferencial Robot 
    def __init__(self, r = 0.0975, l = 0.381*0.5 , D = 0.2) -> None:
        self.wheel_radio = r
        self.len_center2wheel = l
        self.offset = D
        self.init_xcoord = 0
        self.init_ycoord = 1

    def draw_robot(self)-> None:
        #Circle
        theta = np.linspace( 0 , 2 * np.pi , 150 )
        rad = 0.2
        a = rad * np.cos( theta ) + self.init_xcoord
        b = rad * np.sin( theta ) + self.init_ycoord
        plt.figure("animation")
        # plt.clf()
        # plt.gcf().canvas.mpl_connect(
        # 'key_release_event',
        # lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(a, b, 'k', linewidth = 5)
        plt.xlim(-5,5)
        plt.ylim(-5,5)
        



class Obstacle:
    def __init__(self, xcoord = 0, ycoord = 0) -> None:
        self.xcoord = xcoord
        self.ycoord = ycoord
        


    def draw_obstacle(self) -> None:
        plt.figure("animation")
        plt.plot(self.xcoord,self.ycoord,'ys', markersize = 15)


def main():
    robot1 = Robot()

    obstacle1 = Obstacle(1,1)
    obstacle2 = Obstacle(0,-1)
    obstacles_list = [obstacle1, obstacle2]

    robot1.draw_robot()
    for obst in obstacles_list:
        obst.draw_obstacle()

    plt.show()


if __name__ == '__main__':
    main()