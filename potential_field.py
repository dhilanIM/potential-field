import matplotlib.pyplot as plt
import numpy as np
import time
"""

    Path planning for  mobile robot using Artificial Potential Field  approach with no sensors

"""

class Robot:
    # Diferential Robot 
    def __init__(self, x = 0, y = 0) -> None:
        self.xcoord = x
        self.ycoord = y

    def draw_robot(self)-> None:
        #Circle
        theta = np.linspace( 0 , 2 * np.pi , 150 )
        rad = 0.2
        a = rad * np.cos( theta ) + self.xcoord
        b = rad * np.sin( theta ) + self.ycoord
        plt.figure("animation")
        plt.clf()
        plt.gcf().canvas.mpl_connect(
         'key_release_event',
         lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(a, b, 'k', linewidth = 5)
        plt.xlim(-5,5)
        plt.ylim(-5,5)    


class Obstacle:
    def __init__(self, xcoord = 0, ycoord = 0, influence = 1.0) -> None:
        self.xcoord = xcoord
        self.ycoord = ycoord
        self.influence = influence          
        
    def draw_obstacle(self) -> None:
        # Influence Radio
        theta = np.linspace( 0 , 2 * np.pi , 150 )
        rad = self.influence
        a = rad * np.cos( theta ) + self.xcoord
        b = rad * np.sin( theta ) + self.ycoord

        plt.figure("animation")
        plt.plot(self.xcoord,self.ycoord,'ys', markersize = 15, )
        plt.plot(a, b, 'g--', linewidth = 2, markersize = 5 )


def get_atraction_force(k_att, x_goal, y_goal, x, y):
    return (-k_att*(x-x_goal), -k_att*(y-y_goal))
    

def get_repulsion_force(k_rep,phi,xo,yo,x,y):
    phi_o = np.sqrt((x-xo)**2 + (y-yo)**2)
    s = k_rep*((1/phi_o - 1/phi))*((1/phi_o)**2)  ### !!!!! ()**2

    if phi_o <= phi:
        return (s*((x-xo)/phi_o), s*((y-yo)/phi_o))
    else:
        return (0, 0)


def main():
    # Goal position
    x_goal, y_goal = 3.5, 3.0 

    # Robot initial position
    x, y= 0, 0

    #Influence radio
    phi = 1.5

    # Attraction and repulsion  factor
    k_att = 1.0
    k_rep = 5.0

    # Force scaler
    h = 0.05

    # Position error
    error = 99

    #Obstacle positions
    obsts_x = [2.0, 1]
    obsts_y = [2.5, 0]

    robot1 = Robot(x=x,y=y)

    obstacle1 = Obstacle(obsts_x[0], obsts_y[0],phi)
    obstacle2 = Obstacle(obsts_x[1], obsts_y[1],phi)
    obstacles_list = [obstacle1, obstacle2]

    force_x_rep = [1] * len(obstacles_list)
    force_y_rep = [1] * len(obstacles_list)

    itr = 0
    while error >= 0.001:
        robot1.draw_robot()
        
    
        for idx in range(len(obstacles_list)):
            obstacles_list[idx].draw_obstacle()
            force_x_rep[idx],force_y_rep[idx] = get_repulsion_force(k_rep,phi,obsts_x[idx],obsts_y[idx],robot1.xcoord,robot1.ycoord)
        
        # Plot goal
        plt.figure("animation")
        plt.plot(x_goal,y_goal, 'rx', linewidth = 10)

        #plt.figure("position")
        # plt.plot(itr,robot1.xcoord)
        # plt.plot(itr, robot1.ycoord)
        plt.grid(True)
        plt.pause(0.01)
        plt.ion()

        if robot1.xcoord == 0 : time.sleep(2)

        force_x_att, force_y_att = get_atraction_force(k_att, x_goal, y_goal, robot1.xcoord, robot1.ycoord)
        
        error = np.sqrt((x_goal-robot1.xcoord)**2 + (y_goal-robot1.ycoord)**2)

        if error > 0.1:
            #Net Force
            force_x = force_x_att + sum(force_x_rep)
            force_y = force_y_att + sum(force_y_rep)
            #Normalizaed Force
            force_x = force_x/(np.sqrt(force_x**2 + force_y**2))
            force_y = force_y/(np.sqrt(force_x**2 + force_y**2))
        else:
            #Net Force
            force_x = force_x_att + sum(force_x_rep)
            force_y = force_y_att + sum(force_y_rep)

        # Next position
        robot1.xcoord = robot1.xcoord + h*force_x
        robot1.ycoord = robot1.ycoord + h*force_y
        
        itr += 1
    


if __name__ == '__main__':
    main()