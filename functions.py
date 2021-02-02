from math import *
from geometryToSEQ import *
#import sys
import numpy as np
import global_variables as g
#import send_msg as s

TEMPS_REP = 3

def xtk(aircraft, path):  #xtk positive si l'avion est a droite et négatif si l'avion est à gauche
    
    #print(aircraft)
    if path.boolorth==True and path.booltrans==False:
        
        #print(path.ortho.start)
        #print(path.ortho.end)
        proj=ortho_projection(aircraft, path.ortho, None)
        #print(proj)
        se=[path.ortho.end.x-path.ortho.start.x,path.ortho.end.y-path.ortho.start.y,0]
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]
        s=np.sign(np.cross(se,ap)[2])
        
        return s*aircraft.distance(proj)
    
    elif path.boolorth==False and path.booltrans==True:
        
        proj=ortho_projection(aircraft, path.ortho, path.transition)
        #print(proj)
        se=[proj.x-path.ortho.end.x,proj.y-path.ortho.end.y,0]
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]
        s=np.sign(np.cross(se,ap)[2])
        
        return s*aircraft.distance(proj)


def tae(aircraft, path1, path2): # Signé en fonction du sens trigo tae : va du heading vers la trajectoire; heading entre nordMag et cap

    
    xs, ys, xe, ye = path1.ortho.start.x, path1.ortho.start.y, path1.ortho.end.x, path1.ortho.end.y
    v1=[xe-xs, ye-ys, 0]
    v2=[0,1,0]
    s=np.sign(np.cross(v1,v2)[2])   
    if path1.boolorth==True and path1.booltrans==False:
        if s==0:
            if ys<ye:
                angle=0
            else:
                angle=-pi
        else:
            angle= -atan((path1.ortho.end.y - path1.ortho.start.y) / (path1.ortho.end.x - path1.ortho.start.x)) + s * pi / 2
        trackangleerror=aircraft.hdg-angle
        
        if trackangleerror>pi:
            trackangleerror+=-2*pi
            print("c est superieur a pi")
        elif trackangleerror<-pi:
            trackangleerror+=2*pi
            print("c est inferieur a -pi")
        return trackangleerror

    elif path1.boolorth==False and path1.booltrans==True:
        xs2, ys2 = path2.ortho.start.x, path2.ortho.start.y
        xc, yc = path1.transition.list_items[0].centre.x, path1.transition.list_items[0].centre.y
        if path1.transition.type== "Flyby":
            v2 = [xs2 - xe, ys2 - ye, 0]
            v3=[0,1,0]
            if xc==aircraft.x:
                s=np.sign(np.cross(v2,v3)[2])
                angle=s*pi/2
            else:
                a=(yc-aircraft.y)/(xc-aircraft.x)    # Attention au cas ou droite verticale ou horizontale
                v1=[1,-1/a,0]
                if np.vdot(v1,v2)<0:
                    v1=[-1,1/a,0]
                s=np.sign(np.cross(v1,v3)[2])
                if a==0:
                    s = np.sign(np.vdot(v2, v3))
                    angle=pi/2-s*pi/2
                else:    
                    angle=-atan(-1/a)+s*pi/2
            trackangleerror=aircraft.hdg-angle
            if trackangleerror>pi:
                trackangleerror+=-2*pi
            elif trackangleerror<-pi:
                trackangleerror+=2*pi
            return trackangleerror
        
        elif path1.transition.type== "Flyover":
            if path1.transition.boolarc1==True:
                return tae(aircraft, Path(path1.ortho, Transition("Flyby", [path1.transition.list_items[0]], False, True)), path2)
            elif path1.transition.boolseg==True:
                return tae(aircraft, Path(path1.transition.list_items[1], Transition("Flyby", [path1.transition.list_items[2]]), True, False), path2)
            elif path1.transition.boolarc2==True:
                return tae(aircraft, Path(path1.transition.list_items[1], Transition("Flyby", [path1.transition.list_items[2]]), False, True), path2)
'''
act=Aircraft(0,0,-pi/4)
path=Path(Ortho(Point(0,0),Point(-60,-60)),Transition("Flyby",[Arc(Point(70,60),10,10)]))
print(tae(act,path,None))
'''


def arc_distance(p1,p2,arc): # Calcul la distance entre deux points sur la transition

    center=arc.centre
    radius=arc.turn_radius
    c=p1.distance(p2)
    alpha=acos((-c**2+2*(radius**2))/(2*(radius**2)))
    
    return alpha*radius

def ortho_distance(p1,ortho): # Calcul la distance entre un point sur le segment et le point end du segment

    return p1.distance(ortho.end)

def transition_distance(p1, p2, transition):
    
    if transition.type=="Flyby":
        return arc_distance(p1,p2,transition.list_items[0])
    
    elif transition.type=="Flyover":
        seg = transition.list_items[1]
        
        if transition.boolarc1==True:
            return arc_distance(p1, seg.start, transition.list_items[0])+ortho_distance(seg.start,seg)+arc_distance(seg.end, p2, transition.list_items[2])
        
        elif transition.boolseg==True:  
            return ortho_distance(p1,seg)+arc_distance(seg.end, p2, transition.list_items[2])
        
        elif transition.boolarc2==True:
            return arc_distance(p1, p2, transition.list_items[2])
        
        
        
def alongpath_distance(aircraft, path1, path2):
    
    if path1.boolorth==True and path1.booltrans==False:
        if path1.transition.type!=None:
            distseg=ortho_distance(ortho_projection(aircraft,path1.ortho,None),path1.ortho)
            disttrans=transition_distance(path1.ortho.end,ortho_projection(g._TOWPT,path1.ortho,path1.transition),path1.transition)
        else:
            distseg = ortho_distance(ortho_projection(aircraft, path1.ortho, None), path1.ortho)
            disttrans=0
        return distseg+disttrans
    
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==True: # Cas ou on est au niveau de la transition et on a pas changé de leg actif
    
        disttrans=transition_distance(ortho_projection(aircraft,path1.ortho,path1.transition),ortho_projection(g._TOWPT,path1.ortho,path1.transition),path1.transition)
        
        return disttrans
    
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==False and path2.transition.type!=None :  # Cas ou on est au niveau de la transition et on a changé de leg actif
    
        disttrans1=transition_distance(ortho_projection(aircraft,path1.ortho,path1.transition),path2.ortho.start,path1.transition)
        distseg=ortho_distance(path2.ortho.start,path2.ortho)
        disttrans2=transition_distance(path2.ortho.end,ortho_projection(g._TOWPT,path2.ortho,path2.transition),path2.transition)
        return disttrans1+distseg+disttrans2

    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==False and path2.transition.type==None :

        disttrans1 = transition_distance(ortho_projection(aircraft, path1.ortho, path1.transition), path2.ortho.start, path1.transition)
        distseg = ortho_distance(path2.ortho.start, path2.ortho)
        return disttrans1+distseg
  


def ortho_projection(point, ortho, transition=None): #Renvoie la projection du point sur un segment (ortho) ou une transition 
    
    if transition==None:
        
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
    
    elif transition.type=="Flyby":       # Problème si l'avion se trouve au centre de la transition (infinité de projections orthogonales sur le cercle !!!!)
        
        if transition.list_items[0].centre.x == point.x:
            x=transition.list_items[0].centre.x
            y=transition.list_items[0].centre.y+transition.list_items[0].turn_radius
            v1=(ortho.end.x-ortho.start.x, ortho.end.y-ortho.start.y)
            v2=(x-ortho.end.x, y-ortho.end.y)
            if v1[0]*v2[0]+v1[1]*v2[1]<0:
                y=transition.list_items[0].centre.y-transition.list_items[0].turn_radius
        else:
            a = (transition.list_items[0].centre.y-point.y)/(transition.list_items[0].centre.x-point.x)
            b = point.y - a*point.x
            distcp=point.distance(transition.list_items[0].centre)
            radius=transition.list_items[0].turn_radius
            xa, ya, xc, yc = point.x, point.y, transition.list_items[0].centre.x, transition.list_items[0].centre.y
            x=(((1/(2*(yc-ya)))*(distcp**2-2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya)))
            y=a*x+b
            v1=(ortho.end.x-ortho.start.x, ortho.end.y-ortho.start.y)
            v2=(x-ortho.end.x, y-ortho.end.y)
            if v1[0]*v2[0]+v1[1]*v2[1]<0:
                 x=(((1/(2*(yc-ya)))*(distcp**2+2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya)))
                 y=a*x+b
        return(Point(x,y))
    
    elif transition.type=="Flyover":
        
        if transition.boolarc1==True:
            return ortho_projection(point, ortho, Transition("Flyby",[transition.list_items[0]]))
        elif transition.boolseg==True:
            return ortho_projection(point ,transition.list_items[1], None)
        elif transition.boolarc2==True:
            return ortho_projection(point, transition.list_items[1], Transition("Flyby",[transition.list_items[2]]))


def path_sequencing(point, path1, path2):
    ortho1, trans1, ortho2 = path1.ortho, path1.transition, path2.ortho
    xs1, ys1, xe1, ye1 = ortho1.start.x, ortho1.start.y, ortho1.end.x, ortho1.end.y
    xs2, ys2, xe2, ye2 = ortho2.start.x, ortho2.start.y, ortho2.end.x, ortho2.end.y 
    xc, yc = trans1.list_items[0].centre.x, trans1.list_items[0].centre.y
    xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
    proj = ortho_projection(point, ortho1, None)
    x1, y1 = proj.x, proj.y

    if path1.boolorth==True and path1.booltrans==False:
        

        if not (((x1>=xs1 and x1<=xe1) or (x1<=xs1 and x1>=xe1)) and ((y1>=ys1 and y1<=ye1) or (y1<=ys1 and y1>=ye1))):
            if proj.distance(ortho1.end)<1:
                path1.boolorth=False
                path1.booltrans=True
                if trans1.type=="Flyby":
                    if xc==xwpt:
                        g._SIGN=np.sign(point.x-xc)
                    else:
                        a=(yc-ywpt)/(xc-xwpt)    # Attention au cas ou droite verticale ou horizontale
                        b=yc-a*xc
                        g._SIGN=np.sign(point.y-(a*point.x+b))
                    
            
    elif path1.boolorth==False and path1.booltrans==True:

        if trans1.type=="Flyby":
            proj = ortho_projection(point, ortho2, None)
            x2, y2 = proj.x, proj.y
            if (((x1 >= xs1 and x1 <= xe1) or (x1 <= xs1 and x1 >= xe1)) and ((y1 >= ys1 and y1 <= ye1) or (y1 <= ys1 and y1 >= ye1))):
                path1.boolorth = True
                path1.booltrans = False

            elif (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))):
                path1.boolorth=False
                path1.booltrans=False
                g._LISTPATHS=g._LISTPATHS[1:]
                g._LISTBANKANGLES=g._LISTBANKANGLES[1:]
                
        elif trans1.type=="Flyover":
            if (((x1 >= xs1 and x1 <= xe1) or (x1 <= xs1 and x1 >= xe1)) and ((y1 >= ys1 and y1 <= ye1) or (y1 <= ys1 and y1 >= ye1))):
                path1.boolorth = True
                path1.booltrans = False
                path1.transition.boolarc1, path1.transition.boolseg, path1.transition.boolarc2=True, False, False
            elif trans1.boolarc1==True:
                proj = ortho_projection(point, trans1.list_items[1], None)
                x2, y2 = proj.x, proj.y
                xs, ys = trans1.list_items[1].start.x, trans1.list_items[1].start.y
                xe, ye = trans1.list_items[1].end.x, trans1.list_items[1].end.y
                
                if (((x2>=xs and x2<=xe) or (x2<=xs and x2>=xe)) and ((y2>=ys and y2<=ye) or (y2<=ys and y2>=ye))):
                    path1.transition.boolarc1, path1.transition.boolseg = False, True
                    
            elif trans1.boolseg==True:
                proj = ortho_projection(point, trans1.list_items[1], None)
                x2, y2 = proj.x, proj.y
                xs, ys = trans1.list_items[1].start.x, trans1.list_items[1].start.y
                xe, ye = trans1.list_items[1].end.x, trans1.list_items[1].end.y
                
                if not (((x2>=xs and x2<=xe) or (x2<=xs and x2>=xe)) and ((y2>=ys and y2<=ye) or (y2<=ys and y2>=ye))):
                    path1.transition.boolseg, path1.transition.boolarc2 = False, True
                    
            elif trans1.boolarc2==True:
                proj = ortho_projection(point, ortho2, None)
                x2, y2 = proj.x, proj.y
        
                if (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))):
                    path1.boolorth=False
                    path1.booltrans=False
                    g._LISTPATHS=g._LISTPATHS[1:]
                    g._LISTBANKANGLES=g._LISTBANKANGLES[1:]
 

def active_leg(legs_list):  # renvoie la leg active et la supprime

    legs_list=legs_list[1:]
    active_leg=legs_list[0]  # active_leg contient le numéro de la leg
    g._NUMSEQ=int(active_leg[0])
    return active_leg,legs_list


            
def sequencing_conditions(aircraft, path):

    if path.boolorth==False and path.booltrans==True:
        if path.transition.type=="Flyover":
            g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)
            g._LISTPOINTS=g._LISTPOINTS[1:]
            g._TOWPT=g._LISTPOINTS[1]
            #g._TOWPT=Waypoint(g._ACTIVELEG[4],g._ACTIVELEG[5]) # A CONVERTIR EN NM ? (active_leg[3] et 4 et lat et long)
            print("ça séquence fort")
            g._LISTPATHS[0].boolactive=False
            return True
        elif path.transition.type=="Flyby" :
            xc, yc = path.transition.list_items[0].centre.x, path.transition.list_items[0].centre.y
            xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
            if xc==xwpt:
                sgn=np.sign(aircraft.x-xc)
            else:
                a=(yc-ywpt)/(xc-xwpt)    # Attention au cas ou droite verticale ou horizontale
                b=yc-a*xc
                sgn=np.sign(aircraft.y-(a*aircraft.x+b))
            
            if g._SIGN!=sgn:
                print(g._LEGLIST)
                g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)
                g._LISTPOINTS=g._LISTPOINTS[1:]
                g._TOWPT=g._LISTPOINTS[1]
                print(g._LEGLIST)
                print(g._LISTPOINTS)
                #g._TOWPT=Waypoint(g._ACTIVELEG[4],g._ACTIVELEG[5]) # A CONVERTIR EN NM ? (active_leg[3] et 4 et lat et long)
                print("ça séquence fort")
                g._LISTPATHS[0].boolactive=False
                
                return True
        
    return False
     

def bank_angle(aircraft, path1, path2):
    
    if path1.boolorth==True and path1.booltrans==False:
        proj=ortho_projection(aircraft, path1.ortho, None)
        return 0

    
    elif path1.boolorth==False and path1.booltrans==True:
        proj=ortho_projection(aircraft, path1.ortho, path1.transition)
        
        if path1.transition.type=="Flyby":
            return g._LISTBANKANGLES[0]
            
        

        
        elif path1.transition.type=="Flyover":
            
            if path1.transition.boolarc1==True:
                return g._LISTBANKANGLES[0][0]
               
            elif path1.transition.boolseg==True:
                return 0
            elif path1.transition.boolarc2==True:
                return g._LISTBANKANGLES[0][1]
           
            
    
    
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

"""
act=Aircraft(0,1,0)
ort=Ortho(Point(0,0),Point(1,1))
print(ortho_projection(act,ort,None))

act=Aircraft(1.5,1.5,0)
trans=Transition("Flyby",[Arc(Point(2,1),1,1)])
print(ortho_projection(act,ort,trans))

act=Aircraft(1.5,1.5,0)
trans=Transition("Flyover",[Arc(Point(-1,0),1,1),Segment(Point(0,0),Point(1,1)),Arc(Point(2,1),1,1)])
print(ortho_projection(act,ort,trans))

"""





















