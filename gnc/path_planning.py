#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# TODO: change variable names to understand what they are for

#enter sp1 as [x,y]
#enter sp2 as [x,y]
class pp:
    #Creates inputs for the setpoints ("sp") of a square or rectangle with sp1 as the first initial point as [x,y] input and sp2 as the second point also as [x,y] input.
    def __init__(self,sp1,sp2,dx,dy, is_plot=False):
        self.sp1=sp1
        self.sp2=sp2
        self.sp=[sp1,sp2]
        self.dx=dx #number of x intervals needed (user input)
        self.dy=dy #number of y intervals needed (user input)
        self.deltax=1
        self.is_plot=is_plot
        
    def path(self):
        #Based on two initial setpoints, creates the remaining two points based on square/rectangle geometry 
        
        self.x_distance=self.sp2[0]-self.sp1[0]
        self.y_distance=self.sp2[1]-self.sp1[1]
        
        self.sp3=[self.sp1[0]+self.x_distance,self.sp1[1]]
        self.sp4=[self.sp1[0],self.sp1[1]+self.y_distance]
        self.square=[self.sp1,self.sp2,self.sp3,self.sp4
            
        #Sets up variables for the semi-circular cases for the path 
        S=[]
   
        h=range(self.sp1[0],self.sp3[0],self.dy) #circle case
        i=np.arange((self.sp1[0]+(self.dy/2)),self.sp3[0],self.dy) #circle case

        r=(self.dy-self.sp1[0])/4 #radius of the turn
        k=self.sp4[1]-((1/2)*self.dy) #top semi-circular case
        l=self.sp1[1]+((1/2)*self.dy) #bottom semi-circular case
            
        #Creates and iterates the straight cases for the path    
        T=np.arange(self.sp1[0],(self.sp3[0]+self.dy/2),self.dy/2) #x-component (straight)
        # TODO: automatically calculate the number of points or have it as an input
        S=np.arange((self.sp1[1]+(self.dy/2)),((k)),8) #y-component (straight)
           
        xh = []
        yh = []

        # TODO: do not have an overlap of green with red and green with blue points
                     
        #Creates and iterates the semi-circular cases (both top and bottom) for the path            
        for ii, h in enumerate(h):
            x0 = (h - r)
            x1 = (h + r)
            x = np.linspace(x0, x1, 5)  
            y = k + np.sqrt(r**2 - (x - h)**2)  
            xh.append(x+(self.dy/4))
            yh.append(y)
            plt.scatter(x+(self.dy/4),y,c='b') if self.is_plot else None
            
            
        yi = []
        xi = []
                
        for i in i:
            x0 = (i - r)
            x1 = (i + r)
            x = np.linspace(x0, x1, 5)  
            y = l - np.sqrt(r**2 - (x - i)**2)  
            xi.append(x+(self.dy/4))
            yi.append(y)
            plt.scatter(x+(self.dy/4),y,c='red') if self.is_plot else None
           
        xT = []
        yT = []

        for x in T:
            for y in S:
                xT.append(x)
                yT.append(y)
                plt.scatter(x,y,c='g') if self.is_plot else None
                    
        plt.show() if self.is_plot else None
        xh = np.concatenate(xh)
        yh = np.concatenate(yh)
        xi = np.concatenate(xi)
        yi = np.concatenate(yi)
        X = np.concatenate((xh,xi,xT))
        Y = np.concatenate((yh,yi,yT))

        return [X,Y]

def main():
    sp1=[0,0]
    sp2=[160,160]
    dx=40
    dy=40
    is_plot=True
    path = pp(sp1,sp2,dx,dy,is_plot).path()
    return 0

if __name__ == "__main__":
    main()
