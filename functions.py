





from math import *
from geometryToSEQ import *

_SIGN1=0
_AIRCRAFT=Aircraft(0,0,0)
_LEGLIST=[]
_ACTIVELEG=[]
_TOWPT=Waypoint('',0,0)
_LISTPOINTS=[]
_LISTSEGMENTS=[]
_LISTTRANSITIONS=[]
_LISTPATHS=[]

# def dataCompute(aircraft,trajectory,legActif):
    #compute xtk
    

#def intersect(aircraft, droite): #droite : la droite normale à la trajectoire et passant par le wpt

   # a=sign(aircraft.y-f(aircraft.x))
   # while a=sign(aircraft.y-f(aircraft.x)):
     #   pass

    #return True



def active_leg(legs_list):  # renvoie la leg active et la supprime

    legs_list=legs_list[1:]
    active_leg=legs_list[0]  # active_leg contient le numéro de la leg
    
    return active_leg,legs_list


#def distanceAlongPath(aircraft, waypoint, )

def transition_distance(p1,p2,transition): # Calcul la distance sur l'arc de cercle de la transition entre deux points situés sur l'arc de cercle.
    
    center=transition.centre
    radius=transition.turn_radius
    c=p1.distance(p2)
    alpha=acos((-c**2+2*(radius**2))/(2*(radius**2)))
    
    return alpha*radius

def ortho_distance(p1,ortho): # Calcul la distance d'un point situé sur le segment au point end du segment
    return p1.distance(ortho.end)

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
        a = (transition.centre.x-point.x)/(transition.centre.y-point.y)
        b = point.y - a*point.y
        distcp=point.distance(transition.centre)
        radius=transition.turn_radius
        xa, ya, xc, yc = point.x, point.y, transition.centre.x, transition.centre.y
        x=(((1/(2*(yc-ya)))*(distcp**2-2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya)))
        y=a*x+b
        return(Point(x,y))

ortho=Ortho(Point(0,2),Point(3,2))
print(ortho_projection(Point(1,0),ortho,None))
"""
def path_sequencing(point, path1, path2):
    ortho1, trans1, end_path = path1.ortho, path1.transition, path2.ortho.start
    a=(trans.centre.y-ortho.end.y)/(trans.centre.x-ortho.end.x)
    b=trans.centre.y-trans.centre.x*a
    
    if _SIGN1*sign(point.y-(a*point.x+b))<0:
        _SIGN1=sign(point.y-(a*point.x+b))
"""

def path_sequencing(point, path1, path2):

    ortho1, trans1, ortho2 = path1.ortho, path1.transition, path2.ortho
    xs1, ys1, xe1, ye1 = ortho1.start.x, ortho1.start.y, ortho1.end.x, ortho1.end.y
    xs2, ys2, xe2, ye2 = ortho2.start.x, ortho2.start.y, ortho2.end.x, ortho2.end.y 
    xc, yc = trans1.centre.x, trans1.centre.y
    xwpt, ywpt = _TOWPT.x, _TOWPT.y
    if path1.boolorth==True and path1.booltrans==False:
        
        proj = ortho_projection(point, ortho1, None)
        print(proj)
        x1, y1 = proj.x, proj.y
        if not (((x1>=xs1 and x1<=xe1) or (x1<=xs1 and x1>=xe1)) and ((y1>=ys1 and y1<=ye1) or (y1<=ys1 and y1>=ye1))):
            path1.boolorth=False
            path1.booltrans=True
            
            a=(yc-ywpt)/(xc-xwpt)
            b=yc-a*xc
            _SIGN=sign(point.y-(a*point.x+b))
            
    elif path1.boolorth==False and path1.booltrans==True:
        
        proj = ortho_projection(point, ortho2, None)
        x2, y2 = proj.x, proj.y
        if (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))):
            path1.boolorth=False
            path1.booltrans=False
            
    
path1=Path(Ortho(Point(-3,1),Point(0,1)),Transition(Point(0,0),1,1))
path2=Path(Ortho(Point(1,0),Point(1,-3)),Transition(Point(2,-5),1,1))
print(path1.boolorth)
print(path1.booltrans)
point=Point(0.5,0.5)
path_sequencing(point, path1, path2)
print(path1.boolorth)
print(path1.booltrans)
point2=Point(0.5,-1)
path_sequencing(point2, path1, path2)
print(path1.boolorth)
print(path1.booltrans)

def sequencing_conditions(aircraft, path):
    
    if path1.boolorth==False and path1.booltrans==True:
        if _SIGN!=sign(aircraft.y-(a*aircraft.x+b)):
            activeleg, leglist=activ_leg(_LEGLIST)
            _ACTIVELEG, _LEGLIST= activeleg, leglist
            sendActiveLeg(_ACTIVELEG[1])
            sendLegList(_LEGLIST)
            _TOWPT=Waypoint(_ACTIVELEG[0],_ACTIVELEG[3],_ACTIVELEG[4]) # A CONVERTIR EN NM ? (active_leg[3] et 4 et lat et long)
            








