# Reception des messages


from ivy.std_api import *
import time
from send_msg import *
from geometryToSEQ import *
from functions import *
import global_variables as g

def on_cx(agent, connected):
    pass

def on_die(agent, _id):
    pass


def recepTime(*arg):
    
    g._TIME=float(arg[1])
    print(round(g._TIME,1))
    
def recepLegList(*arg):
    L=L=arg[2].strip().strip("(").strip(")").strip(";").split(";")
    LegList=[l.split() for l in L]                          # On découpe la liste par bloc de leg ID=WPT1 SEQ=0 COURSE=110  LAT= LON=    
    print(LegList)
    print(len(LegList))
    g._LEGLIST=[]
    for i in range(len(LegList)):
            
        g._LEGLIST.append([LegList[i][j].split("=")[1] for j in range(5)])
        
        
    g._TOWPT=Waypoint(g._LEGLIST[0][2], g._LEGLIST[0][3])
    g._ACTIVELEG=g._LEGLIST[0]
    
#FL_LegList Time=1 LegList=ID=WPT2 SEQ=1 LAT=S10101010 LON=W020102010 COURSE=10;ID=WPT1 SEQ=0 LAT=N60201234 LON=E060301234  COURSE=110

def recepGS(*args):
    L=args[1].strip()
    g._GS = eval(L)


def recepBankAngles(*args):
    L=args[1].strip()
    g._LISTBANKANGLES=eval(L)
    

def recepPoints(*arg):    
    L=arg[1].strip()
    g._LISTPOINTS=eval(L)

    
def recepSegments(*arg):
    Liste_Points=g._LISTPOINTS
    L=arg[1].strip()
    g._LISTSEGMENTS=eval(L)

    
def recepOrthos(*arg):
    L=arg[1].strip()
    g._LISTORTHOS=eval(L)
    
    
def recepTransitions(*arg):
    L=arg[1].strip()
    g._LISTTRANSITIONS=eval(L)

        
def recepPaths(*arg):
    Liste_Orthos, Liste_Transitions = g._LISTORTHOS, g.LISTTRANSITIONS
    L=arg[1].strip()
    g._LISTPATHS=eval(L)
    
#GT_TRAJ TRAJ_Paths=[Path(TRAJ_Transitions[0], TRAJ_Segments[0]), Path(TRAJ_Transitions[1], TRAJ_Segments[1])]   

def recepStateVector(*arg):
    x=float(arg[1])/1852            #Metres convertis en NM
    y=float(arg[2])/1852
    hdg=float(arg[6])               #EN RADIANS
    g._AIRCRAFT=Aircraft(x,y,hdg)
    if g._LISTPATHS!=[] and g._TOWPT.id!='':
        path_sequencing(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1]) # Au cas où il ne reste plus qu'un seul path dans la trajectoire
        if sequencing_conditions(g._AIRCRAFT,g._LISTPATHS[0]):
            s.sendActiveLeg(g._ACTIVELEG[1])
            s.sendNewLegList(g._LEGLIST)
            
        xtk, tae, dtwpt, bank_angle_ref = xtk(g._AIRCRAFT, g._LISTPATHS[0]), tae(g._AIRCRAFT, g._LISTPATHS[0]), g.AIRCRAFT.distance(g._TOWPT), bank_angle(g._AIRCRAFT, g._LISTPATHS[0], g._LISTPATHS[1])
        apdist=alongpath_distance(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1])
        sendAlongPathDistance(apdist)
        sendData(xtk, tae, dtwpt, bank_angle_ref)
        
    


#StateVector x=15 y=12 z=0 Vp=2 fpa=4 psi=(.*) phi=(.*)


# fonction eval => run la chaine de caractère







