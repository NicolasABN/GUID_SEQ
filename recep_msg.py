# Reception des messages


from ivy.std_api import *
import time
from send_msg import *
from geometryToSEQ import *
from functions import *
import global_variables as g
import matplotlib.pyplot as plt
def on_cx(agent, connected):
    pass

def on_die(agent, _id):
    pass


def recepTime(*arg):
    
    g._TIME=float(arg[1])
    #print(round(g._TIME,1))
    
def recepLegList(*arg):
    L=arg[2].strip().strip("(").strip(")").strip(";").split(";")
    LegList=[l.split() for l in L]                          # On découpe la liste par bloc de leg ID=WPT1 SEQ=0 COURSE=110  LAT= LON=    

    g._LEGLIST=[]
    for i in range(g._NUMSEQ,len(LegList)):
            
        g._LEGLIST.append([LegList[i][j].split("=")[1].strip() for j in range(9)])
        

    #g._TOWPT=Waypoint(g._LEGLIST[0][4], g._LEGLIST[0][5])
    g._ACTIVELEG=g._LEGLIST[0]
    print(g._ACTIVELEG)
    print(g._LEGLIST)
    #print(g._TOWPT)
    
#FL_LegList Time=1 LegList=ID=WPT1 SEQ=0 COURSE=110  LAT=N100000000 LON=E0100000000;ID=WPT2 SEQ=1 COURSE=10  LAT=N600000000 LON=E0700000000

def recepGS(*args):
    L=args[1].strip()
    g._GS = eval(L)


def recepBankAngles(*args):
    L=args[1].strip()
    g._LISTBANKANGLES=eval(L)
    print('bank_angles_reçus')
    

def recepPoints(*arg):    
    L=arg[1].strip()
    g._LISTPOINTS=eval(L)
    g._TOWPT=g._LISTPOINTS[1]
    print(g._TOWPT)
    print(g._LISTPOINTS)

    
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
    Liste_Orthos, Liste_Transitions = g._LISTORTHOS, g._LISTTRANSITIONS
    L=arg[1].strip()
    g._LISTPATHS=eval(L)
    print("Paths reçus")
    
#GT_TRAJ TRAJ_Paths=[Path(TRAJ_Transitions[0], TRAJ_Segments[0]), Path(TRAJ_Transitions[1], TRAJ_Segments[1])]   

def recepStateVector(*arg):
    x=float(arg[1])/1852            #Metres convertis en NM
    y=float(arg[2])/1852
    X=[]
    Y=[]
    X.append(x)
    Y.append(y)
    plt.close()
    plt.plot(X,Y)
    plt.show()
    hdg=float(arg[6])               #EN RADIANS
    g._AIRCRAFT=Aircraft(x,y,hdg)
    if g._LISTPATHS!=[]:
        path_sequencing(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1]) # Au cas où il ne reste plus qu'un seul path dans la trajectoire
        if sequencing_conditions(g._AIRCRAFT,g._LISTPATHS[0]):
            s.sendActiveLeg(g._ACTIVELEG[0])
            s.sendNewLegList(g._LEGLIST)
            
        xtk_, tae_, dtwpt, bank_angle_ref = xtk(g._AIRCRAFT, g._LISTPATHS[0]), tae(g._AIRCRAFT, g._LISTPATHS[0]), g._AIRCRAFT.distance(g._TOWPT), bank_angle(g._AIRCRAFT, g._LISTPATHS[0], g._LISTPATHS[1])
        apdist=alongpath_distance(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1])
        sendData(xtk_, tae_, dtwpt, bank_angle_ref, apdist)
        
    

#StateVector x=15 y=12 z=0 Vp=2 fpa=4 psi=5 phi=10


# fonction eval => run la chaine de caractère







