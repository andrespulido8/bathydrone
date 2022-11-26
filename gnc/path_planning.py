#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


#enter sp1 as [x,y]
#enter sp2 as [x,y]
class pp:
    def __init__(self,sp1,sp2,dx,dy):
        self.sp1=sp1
        self.sp2=np.transpose(sp2)
        self.sp=[sp1,sp2]
        self.dx=dx
        self.dy=dy
        self.deltax=1
        
        
        self.x_distance=sp2[0]-sp1[0]
        self.y_distance=sp2[1]-sp1[1]
        
        self.sp3=[sp1[0]+self.x_distance,sp1[1]]
        self.sp4=[sp1[0],sp1[1]+self.y_distance]
        self.square=[sp1,sp2,self.sp3,self.sp4]

        #if self.x_distance % dx == 0 and self.y_distance % dy==0:
           # print("Distance is divisible")
            
          
        S=[]

        h=range(self.sp1[0],self.sp3[0],dy) #circle case
        i=np.arange((self.sp1[0]+(dy/2)),self.sp3[0],dy) #circle case

        x=[]
        y=[]

        r=(dy-sp1[0])/4 #radius
        k=self.sp4[1]-((1/2)*dy) #top case
        l=self.sp1[1]+((1/2)*dy) #bottom case
            
            
        T=np.arange(self.sp1[0],(self.sp3[0]+dy/2),dy/2) #x-component (straight)
        S=np.arange((sp1[1]+(dy/2)),((k)),1) #y-component (straight)
           
        for h in h:
            x0 = (h - r)
            x1 = (h + r)
            x = np.linspace(x0, x1, 5)  
            y = k + np.sqrt(r**2 - (x - h)**2)  
            plt.scatter(x+(dy/4),y,c='red')
            
            
                
        for i in i:
            x0 = (i - r)
            x1 = (i + r)
            x = np.linspace(x0, x1, 5)  
            y = l - np.sqrt(r**2 - (x - i)**2)  
            plt.scatter(x+(dy/4),y,c='red')
           
            

        for x in T:
            for y in S:
                (x,y)
                plt.scatter(x,y,c='red')
                    
        plt.show()

if __name__ == "__main__":
    G=pp([0,0],[160,160],40,40)