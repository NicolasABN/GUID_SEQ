# Reception des messages


from send_msg import *
from functions import *
from geometryToSEQ import *
import global_variables as g

def on_cx(agent, connected):
    pass

def on_die(agent, _id):
    pass


def recepTime(*arg):    # Reception du temps 
    g._TIME=float(arg[1])
    
def recepMode(*arg):    # Reception du mode (NAV ou HDG)
    s=arg[3].strip()
    g._MODE=eval(s)

    
def recepLegList(*arg):    # Reception de la liste des legs
    L=arg[2].strip().strip("(").strip(")").strip(";")   # Utilisation de strip pour enlever les éléments a gauche et à droite de la chaîne reçue
    L=L.split(";")                                      # Découpage de la chaîne de caractère en liste de chaîne de caractère de chaque leg
    LegList=[l.split() for l in L]                      # Découpage de chaque leg en liste d'éléments de la leg
    g._LEGLIST=[]
    for i in range(g._NUMSEQ,len(LegList)):
            
        g._LEGLIST.append([LegList[i][j].split("=")[1].strip() for j in range(9)])    # Ajout de chaque élément à la variable globale g._LEGLIST à partir du numéro de sequencement (en gardant que les valeurs)
        
    g._ACTIVELEG=g._LEGLIST[0]  # Initialisation de la leg active à la première leg
    print(g._LEGLIST)   # Contrôle de la reception avec un affichage de la liste des legs
    



def recepBankAngles(*args):   # Reception de la liste des BankAngles
    L=args[1].strip()
    g._LISTBANKANGLES=eval(L)
    print('bank_angles_reçus')   # Contrôle de la reception avec un message
    

def recepPoints(*arg):    # Reception de la liste des points (les waypoints de la trajectoire)
    L=arg[1].strip()
    g._LISTPOINTS=eval(L)
    g._TOWPT=g._LISTPOINTS[1]    # Initialisation du g._TOWPT au premier waypoint 

    
def recepSegments(*arg):    # Reception de la liste des segments
    Liste_Points=g._LISTPOINTS
    L=arg[1].strip()
    g._LISTSEGMENTS=eval(L)

    
def recepOrthos(*arg):    # Reception de la liste des Orthos
    L=arg[1].strip()
    g._LISTORTHOS=eval(L)
    
    
def recepTransitions(*arg):    # Reception de la liste des transitions
    L=arg[1].strip()
    g._LISTTRANSITIONS=eval(L)

        
def recepPaths(*arg):    # Reception de la liste des Paths 
    Liste_Orthos, Liste_Transitions = g._LISTORTHOS, g._LISTTRANSITIONS    # Utilisation de la liste des orthos et des transitions pour créer les paths
    L=arg[1].strip()
    g._LISTPATHS=eval(L)
    print("Paths reçus")     # Contrôle de la reception avec un message 
    


def recepStateVector(*arg):         # Reception du vecteur d'état provenant du modèle avion
    x=float(arg[2])/1852            # Metres convertis en NM
    y=float(arg[1])/1852            
    hdg=float(arg[6])               # Hdg reçu en radians
    g._AIRCRAFT=Aircraft(x,y,hdg)   
    
    if g._LISTPATHS!=[] and len(g._LISTPATHS)>1:                     # Deux cas à traiter : il reste plus d'un path ou il n'y en a plus qu'un (sur le dernier leg)
        path_sequencing(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1]) # Sequencement du path
        
        if sequencing_conditions(g._AIRCRAFT,g._LISTPATHS[0]):       # Sequencement du leg
            sendActiveLeg(g._ACTIVELEG[0])                           # Envoi du numéro de séquencement 
            
        if len(g._LISTPATHS)>1:  # Calcul des différents paramètres dans le cas où il reste plus d'un path (si l'on vient de séquencer)
            xtk_=xtk(g._AIRCRAFT, g._LISTPATHS[0])
            tae_=tae(g._AIRCRAFT, g._LISTPATHS[0],g._LISTPATHS[1])
            dtwpt=g._AIRCRAFT.distance(g._TOWPT)
            bank_angle_ref=bank_angle(g._AIRCRAFT, g._LISTPATHS[0], g._LISTPATHS[1])
            apdist=alongpath_distance(g._AIRCRAFT,g._LISTPATHS[0],g._LISTPATHS[1])

        else:    # Calcul des différents paramètres dans le cas où il reste qu'un path (On vient de séquencer sur le dernier leg)
            xtk_=xtk(g._AIRCRAFT, g._LISTPATHS[0])
            tae_=tae(g._AIRCRAFT, g._LISTPATHS[0],None)
            dtwpt=g._AIRCRAFT.distance(g._TOWPT)
            bank_angle_ref=0
            apdist = alongpath_distance(g._AIRCRAFT, g._LISTPATHS[0], None)
        sendData(xtk_, tae_, dtwpt, bank_angle_ref, apdist)        # Envoi des différents éléments dans le bus Ivy
        
    elif g._LISTPATHS!=[] and len(g._LISTPATHS)==1:     # Calcul des différents paramètres dans le cas où il n'y a plus qu'un path (quand on est sur le dernier leg)
        xtk_=xtk(g._AIRCRAFT, g._LISTPATHS[0])
        tae_=tae(g._AIRCRAFT, g._LISTPATHS[0],None)
        dtwpt=g._AIRCRAFT.distance(g._TOWPT)
        bank_angle_ref=0
        apdist = alongpath_distance(g._AIRCRAFT, g._LISTPATHS[0], None)
        sendData(xtk_, tae_, dtwpt, bank_angle_ref, apdist)   # Envoi des différents éléments dans le bus Ivy








