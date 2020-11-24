# Reception des messages


from ivy.std_api import *
import time
from send_msg import *
from geometryToSEQ import *


def on_cx(agent, connected):
    pass

def on_die(agent, _id):
    pass


def recepTime(*arg):
    _TIME=float(arg[1])
    print(round(_TIME,1))
    
def recepLegList(*arg):
    L=arg[2].strip().strip(";").split(";")
    LegList=[l.split() for l in L]                          # On découpe la liste par bloc de leg ID=WPT1 SEQ=0 COURSE=110  LAT= LON=  
    # sendNewLegList(LegList)                         # Puis on redécoupe chaque leg pour avoir la liste de ses paramètres [ID=WPT1,SEQ=0,COURSE=110,LAT=,LON=]
    for i in range(len(LegList)):
        for j in range(5):
            _LEGLIST[i][j]=LegList[i][j].split("=")[1]
    print("time ="+arg[1])
    print(_LEGLIST)
    
#FL_LegList Time=1 LegList=ID=WPT1 SEQ=0 COURSE=110  LAT=10 LON=20;ID=WPT2 SEQ=1 COURSE=10  LAT=101 LON=201

def recepPoints(*arg):    
    L=arg[1].strip()
    _LISTPOINTS=eval(L)
    print(_LISTPOINTS)
    
def recepSegments(*arg):
    L=arg[1].strip()
    _LISTSEGMENTS=eval(L)
    print(_LISTSEGMENTS)
    
def recepTransitions(*arg):
    L=arg[1].strip()
    _LISTTRANSITIONS=eval(L)
    print(_LISTTRANSITIONS)
    
def recepPaths(*arg):
    L=arg[1].strip()
    _LISTPATHS=eval(L)
    print(_LISTPATHS)
    
#GT_TRAJ TRAJ_Paths=[Path(TRAJ_Transitions[0], TRAJ_Segments[0]), Path(TRAJ_Transitions[1], TRAJ_Segments[1])]   

def recepStateVector(*arg):
    x=float(arg[1])
    y=float(arg[2])
    _AIRCRAFT=Point(x,y)
    #direct_distance=_AIRCRAFT.distance(_WAYPOINT)
    
    print(_AIRCRAFT)

#StateVector x=15 y=12 z=0 Vp=2 fpa=4 psi=(.*) phi=(.*)

def recepHeading(*arg):
    hdg=float(arg[2])
    _AIRCRAFT.hdg=hdg
    print(_AIRCRAFT.hdg)
    
# AircraftSetPosition X=(.*) Y=(.*) Altitude-ft=(.*) Roll=(.*) Pitch=(.*) Yaw=(.*) Heading=15 Airspeed=(.*) Groundspeed=(.*)
# fonction eval => run la chaine de caractère







