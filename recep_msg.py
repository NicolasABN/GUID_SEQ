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
    L=arg[2].strip().strip(";").split(";")
    LegList=[l.split() for l in L]                          # On découpe la liste par bloc de leg ID=WPT1 SEQ=0 COURSE=110  LAT= LON=  
    # sendNewLegList(LegList)   
    print(LegList)
    print(len(LegList))
    for i in range(len(LegList)):
            
        g._LEGLIST.append([LegList[i][j].split("=")[1] for j in range(5)])
            #LegList[i][j].split("=")[1]
        
    g._TOWPT=Waypoint(g._LEGLIST[0][0], float(g._LEGLIST[0][3]), float(g._LEGLIST[0][4]))
    g._ACTIVELEG=g._LEGLIST[0]
    print(g._TOWPT)
    print("time ="+arg[1])
    print(g._LEGLIST)
    
#FL_LegList Time=1 LegList=ID=WPT1 SEQ=0 COURSE=110  LAT= LON=20;ID=WPT2 SEQ=1 COURSE=10  LAT=101 LON=201

def recepPoints(*arg):    
    L=arg[1].strip()
    g._LISTPOINTS=eval(L)
    print(g._LISTPOINTS)
    
def recepSegments(*arg):
    Liste_Points=g._LISTPOINTS
    L=arg[1].strip()
    g._LISTSEGMENTS=eval(L)
    print(g._LISTSEGMENTS)
    
def recepOrthos(*arg):
    Liste_Points=g._LISTPOINTS
    L=arg[1].strip()
    g._LISTORTHOS=eval(L)
    
    
def recepTransitions(*arg):
    L=arg[1].strip()
    g._LISTTRANSITIONS=eval(L)
    print(g._LISTTRANSITIONS)
        
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
    path_sequencing(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1])
    sequencing_conditions(g._AIRCRAFT,g._LISTPATHS[0])
    direct_distance=_AIRCRAFT.distance(_WAYPOINT)
    
    print(g._AIRCRAFT)

#StateVector x=15 y=12 z=0 Vp=2 fpa=4 psi=(.*) phi=(.*)


# fonction eval => run la chaine de caractère







