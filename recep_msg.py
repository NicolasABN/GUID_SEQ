# Reception des messages


from ivy.std_api import *
import time
import send_msg as send
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
    # send.sendNewLegList(LegList)                         # Puis on redécoupe chaque leg pour avoir la liste de ses paramètres [ID=WPT1,SEQ=0,COURSE=110,LAT=,LON=]
    print("time ="+arg[1])
    print(LegList)
    
#FL_LegList Time=1 LegList=ID=WPT1 SEQ=0 COURSE=110  LAT=10 LON=20;ID=WPT2 SEQ=1 COURSE=10  LAT=101 LON=201

def recepPoints(*arg):    
    L=arg[1].strip()
    ListPoints=eval(L)
    print(ListPoints)
    
def recepSegments(*arg):
    L=arg[1].strip()
    ListSegments=eval(L)
    print(ListSegments)
    
def recepTransitions(*arg):
    L=arg[1].strip()
    ListTransitions=eval(L)
    print(ListTransitions)
    
def recepPaths(*arg):
    L=arg[1].strip()
    ListPaths=eval(L)
    print(ListPaths)
    
#GT_TRAJ TRAJ_Paths=[Path(TRAJ_Transitions[0], TRAJ_Segments[0]), Path(TRAJ_Transitions[1], TRAJ_Segments[1])]   

def recepStateVector(*arg):
    x=float(arg[1])
    y=float(arg[2])
    print(x,y)

#StateVector x=15 y=12 z=0 Vp=2 fpa=4 psi=(.*) phi=(.*)


# fonction eval => run la chaine de caractère







