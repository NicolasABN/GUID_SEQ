from math import *
from geometryToSEQ import *
import sys
import numpy as np
import global_variables as g
import send_msg as s


def xtk(aircraft, path):  #xtk positive si l'avion est a droite et négatif si l'avion est à gauche
    
    if path.boolorth==True and path.booltrans==False:
        
        proj=ortho_projection(aircraft, path1.ortho, None)
        se=[path.ortho.end.x-path.ortho.start.x,path.ortho.end.y-path.ortho.start.y,0]
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]
        s=np.sign(np.cross(se,ap)[2])
        
        return s*aircraft.distance(proj)
    
    elif path.boolorth==False and path.booltrans==True:
        
        proj=ortho_projection(aircraft, None, path.transition)
        se=[proj.x-path.ortho.end.x,proj.y-path.ortho.end.y,0]
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]
        s=np.sign(np.cross(se,ap)[2])
        
        return s*aircraft.distance(proj)


def tae(aircraft, path): # Signé en fonction du sens trigo tae : va du heading vers la trajectoire; heading entre nordMag et cap
    
    if path.boolorth==True and path.booltrans==False:
        
        angle=atan((path.ortho.end.y-path.ortho.start.y)/(path.ortho.end.x-path.ortho.start.x))-pi/2
        
        return angle+aircraft.hdg
  
    if path.boolorth==False and path.booltrans==True:
        
        xc, yc = path.transition.centre.x, path.transition.centre.y
        a=(yc-aircraft.y)/(xc-aircraft.x)    # Attention au cas ou droite verticale ou horizontale
        
        if a==0:
            angle=0
        else:    
            angle= atan(-1/a)-pi/2
            
        return angle+aircraft.hdg
        

    
    

def transition_distance(p1,p2,transition): # Calcul la distance entre deux points sur la transition

    center=transition.centre
    radius=transition.turn_radius
    c=p1.distance(p2)
    alpha=acos((-c**2+2*(radius**2))/(2*(radius**2)))
    
    return alpha*radius

def ortho_distance(p1,ortho): # Calcul la distance entre un point sur le segment et le point end du segment

    return p1.distance(ortho.end)

def alongpath_distance(aircraft, path1, path2):
    
    if path1.boolorth==True and path1.booltrans==False:
        
        distseg=ortho_distance(ortho_projection(aircraft,path1.ortho,None),path1.ortho)
        disttrans=transition_distance(path1.ortho.end,ortho_projection(_TOWPT,None,path1.transition),path1.transition)
        
        return distseg+disttrans
    
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==True: # Cas ou on est au niveau de la transition et on a pas changé de leg actif
    
        disttrans=transition_distance(ortho_projection(aircraft,None,path1.transition),ortho_projection(_TOWPT,None,path1.transition),path1.transition)
        
        return disttrans
    
    else :  # Cas ou on est au niveau de la transition et on a changé de leg actif
    
        disttrans1=transition_distance(ortho_projection(aircraft,None,path1.transition),path2.ortho.start,path1.transition)
        distseg=ortho_distance(path2.ortho.start,path2.ortho)
        disttrans2=transition_distance(path2.ortho.end,ortho_projection(_TOWPT,None,path2.transition),path2.transition)
        
        return disttrans1+distseg+disttrans2
  



def ortho_projection(point, ortho=None, transition=None): #Renvoie la projection du point sur un segment (ortho) ou une transition 
    
    if ortho!=None:
        
        if(ortho.start.x==ortho.end.x):
            return(Point(ortho.start.x,point.y))
        elif(ortho.start.y==ortho.end.y):
            return(Point(point.x,ortho.start.y))
        
        else:
            
            a = (ortho.start.y-ortho.end.y)/(ortho.start.x-ortho.end.x)
            b = ortho.start.y - a*ortho.start.x
            a_norm = -1/a
            b_norm = point.y-(-1/a)*point.x
            x = (b_norm-b)/(a-a_norm)
            y = a*x + b 
            
        return (Point(x,y))
    
    elif transition!=None:       # Problème si l'avion se trouve au centre de la transition (infinité de projections orthogonales sur le cercle !!!!)
    
        a = (transition.centre.y-point.y)/(transition.centre.x-point.x)
        b = point.y - a*point.x
        distcp=point.distance(transition.centre)
        radius=transition.turn_radius
        xa, ya, xc, yc = point.x, point.y, transition.centre.x, transition.centre.y
        x=(((1/(2*(yc-ya)))*(distcp**2-2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya)))
        y=a*x+b
        
        return(Point(x,y))



def path_sequencing(point, path1, path2):

    ortho1, trans1, ortho2 = path1.ortho, path1.transition, path2.ortho
    xs1, ys1, xe1, ye1 = ortho1.start.x, ortho1.start.y, ortho1.end.x, ortho1.end.y
    xs2, ys2, xe2, ye2 = ortho2.start.x, ortho2.start.y, ortho2.end.x, ortho2.end.y 
    xc, yc = trans1.centre.x, trans1.centre.y
    xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
    
    if path1.boolorth==True and path1.booltrans==False:
        
        proj = ortho_projection(point, ortho1, None)
        print(proj)
        x1, y1 = proj.x, proj.y
        
        if not (((x1>=xs1 and x1<=xe1) or (x1<=xs1 and x1>=xe1)) and ((y1>=ys1 and y1<=ye1) or (y1<=ys1 and y1>=ye1))):
            
            path1.boolorth=False
            path1.booltrans=True
            a=(yc-ywpt)/(xc-xwpt)    # Attention au cas ou droite verticale ou horizontale
            b=yc-a*xc
            g._SIGN=np.sign(point.y-(a*point.x+b))
            
    elif path1.boolorth==False and path1.booltrans==True:
        
        proj = ortho_projection(point, ortho2, None)
        x2, y2 = proj.x, proj.y
        
        if (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))):
            
            path1.boolorth=False
            path1.booltrans=False
            g._LISTPATHS=g._LISTPATHS[1:]
 
    
def active_leg(legs_list):  # renvoie la leg active et la supprime

    legs_list=legs_list[1:]
    active_leg=legs_list[0]  # active_leg contient le numéro de la leg
    
    return active_leg,legs_list


            
def sequencing_conditions(aircraft, path):

    if path.boolorth==False and path.booltrans==True:
        
        xc, yc = path.transition.centre.x, path.transition.centre.y
        xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
        a=(yc-ywpt)/(xc-xwpt)    # Attention au cas ou droite verticale ou horizontale
        b=yc-a*xc
        
        if g._SIGN!=np.sign(aircraft.y-(a*aircraft.x+b)):
            
            g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)
            g._TOWPT=Waypoint(g._ACTIVELEG[3],g._ACTIVELEG[4]) # A CONVERTIR EN NM ? (active_leg[3] et 4 et lat et long)
            g._LISTPATHS[0].boolactive=False
            
            return True
        
    return False
          
'''
#Test TAE        
path1=Path(Ortho(Point(-1,0),Point(0,1)),Transition(Point(0,0),1,1))   
path1.boolorth=False
path1.booltrans=True    
act=Aircraft(sqrt(3)/2,0.5,0)
print(tae(act, path1))     
'''

 
"""
test pour alongpathdistance        
_TOWPT=Waypoint('wpt1',2,-3)
path1=Path(Ortho(Point(-1,0),Point(0,1)),Transition(Point(0,0),1,1))
path2=Path(Ortho(Point(1,0),Point(2,-2)),Transition(Point(3,-2),1,1))
path1.boolorth=False
path1.booltrans=True
path1.boolactive=False
aircraft=Aircraft(sqrt(3),1,0)
print(alongpath_distance(aircraft, path1, path2))
"""

"""
tests de xtk
path1=Path(Ortho(Point(-1,0),Point(0,1)),Transition(Point(0,0),1,1))
path2=Path(Ortho(Point(1,0),Point(2,-2)),Transition(Point(3,-3),1,1))
path1.boolorth=False
path1.booltrans=True
aircraft=Aircraft(0.5,0.5,0)
print(xtk(aircraft, path1))
aircraft2=Aircraft(1,1,0)
print(xtk(aircraft2, path1))
"""

"FL_LegList Time=1 LegList=ID=WPT1 SEQ=0 COURSE=110  LAT=1 LON=1 ;ID=WPT2 SEQ=1 COURSE=10  LAT=2 LON=-2"    
"GT_TRAJ TRAJ_Paths=[Path(Ortho(Point(-2,0),Point(0,1)),Transition(Point(0,0),1,1)),Path(Ortho(Point(1,0),Point(2,-2)),Transition(Point(3,-3),1,1))]"
"StateVector x=-1 y=1 z=0 Vp=2 fpa=4 psi=5 phi=4"
"StateVector x=0.5 y=1 z=0 Vp=2 fpa=4 psi=5 phi=4"
"StateVector x=1 y=0.5 z=0 Vp=2 fpa=4 psi=5 phi=4"
"""path_sequencing(point, path1, path2)
print(path1.boolorth)
print(path1.booltrans)
point2=Point(0.5,-1)
path_sequencing(point2, path1, path2)
print(path1.boolorth)
print(path1.booltrans)
"""




