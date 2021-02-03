# Module des fonctions de calcul

from math import *
import global_variables as g


def xtk(aircraft, path):  #xtk positive si l'avion est a droite et négative si l'avion est à gauche
    
    """ Avion au niveau de l'ortho (True / False) """
    if path.boolorth==True and path.booltrans==False:   
       
        proj=ortho_projection(aircraft, path.ortho, None)               #projete de l'avion sur l'ortho
        se=[path.ortho.end.x-path.ortho.start.x,path.ortho.end.y-path.ortho.start.y,0]  #vecteur ortho.start-->ortho.end
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]                     #vecteur avion-->projete
        s=np.sign(np.cross(se,ap)[2])                                   #Produit vectoriel pour determiner le signe de xtk
        
        return s*aircraft.distance(proj)                                #xtk signée
        
    """ Avion au niveau de la transition (False / True) """
    elif path.boolorth==False and path.booltrans==True:                 #False et True ==> avion au niveau de la transition
        
        proj=ortho_projection(aircraft, path.ortho, path.transition)    #projete de l'avion sur la transition
        se=[proj.x-path.ortho.end.x,proj.y-path.ortho.end.y,0]          #vecteur l'ortho.end-->projete
        ap=[proj.x-aircraft.x, proj.y-aircraft.y,0]                     #vecteur avion-->projete
        s=np.sign(np.cross(se,ap)[2])                                   #Produit vectoriel pour determiner le signe de xtk
        
        return s*aircraft.distance(proj)                                #xtk signée


def tae(aircraft, path1, path2): # Signé en fonction du sens trigo tae : va du heading vers la trajectoire

    
    xs, ys, xe, ye = path1.ortho.start.x, path1.ortho.start.y, path1.ortho.end.x, path1.ortho.end.y
    v1=[xe-xs, ye-ys, 0]                                        # vecteur ortho.start-->vecteur ortho.end
    v2=[0,1,0]                                                  #vecteur nord
    s=np.sign(np.cross(v1,v2)[2])                               #signe de la rotation
    
    if path1.boolorth==True and path1.booltrans==False:         #niveau de l'ortho
        if s==0:                                                #trajectoire et nord parallèles
            if ys<ye:
                angle=0
            else:
                angle=-pi
        else:                                                   # angle = angle signé dans le sens trigo entre la trajectoire et le nord
            angle= -atan((path1.ortho.end.y - path1.ortho.start.y) / (path1.ortho.end.x - path1.ortho.start.x)) + s * pi / 2
        trackangleerror=aircraft.hdg-angle
        
        #angle entre -pi et pi
        if trackangleerror>pi:
            trackangleerror+=-2*pi
        elif trackangleerror<-pi:
            trackangleerror+=2*pi
        return trackangleerror

    elif path1.boolorth==False and path1.booltrans==True:       #niveau de la transition
        xs2, ys2 = path2.ortho.start.x, path2.ortho.start.y
        xc, yc = path1.transition.list_items[0].centre.x, path1.transition.list_items[0].centre.y
        if path1.transition.type== "Flyby":
            v2 = [xs2 - xe, ys2 - ye, 0]                        # vecteur path1.ortho.end-->path2.ortho.start
            v3=[0,1,0]
            if xc==aircraft.x:                                  #trajectoire et nord perpendiculaires
                s=np.sign(np.cross(v2,v3)[2])                   #signe selon le sens de la trajectoire
                angle=s*pi/2                                    #angle entre traj et nord
            else:
                a=(yc-aircraft.y)/(xc-aircraft.x)               #coeff directeur de la droite avion et centre de la transition
                v1=[1,-1/a,0]                                   #vecteur tangeant à la trajetoire 
                if np.vdot(v1,v2)<0:                            #V1 et v2 doivent etre dans le meme sens 
                    v1=[-1,1/a,0]
                s=np.sign(np.cross(v1,v3)[2])                   #signe de la rotation
                if a==0:                                        # si a=0 alors traj et nord parallèles ==> angle = 0 ou pi
                    s = np.sign(np.vdot(v2, v3))
                    angle=pi/2-s*pi/2
                else:       
                    angle=-atan(-1/a)+s*pi/2                    #rotation de pi/2 selon le signe du produit vectoriel
            trackangleerror=aircraft.hdg-angle
            
            #angle entre -pi et pi
            if trackangleerror>pi:
                trackangleerror+=-2*pi
            elif trackangleerror<-pi:
                trackangleerror+=2*pi
            return trackangleerror
        
        """ Fly over traité comme un flyby particulier"""
        elif path1.transition.type== "Flyover":
            #Avion au niveau du premier arc
            if path1.transition.boolarc1==True:
                return tae(aircraft, Path(path1.ortho, Transition("Flyby", [path1.transition.list_items[0]], False, True)), path2)
            #Avion au niveau du segment
            elif path1.transition.boolseg==True:
                return tae(aircraft, Path(path1.transition.list_items[1], Transition("Flyby", [path1.transition.list_items[2]]), True, False), path2)
            #Avion au niveau du deuxieme arc
            elif path1.transition.boolarc2==True:
                return tae(aircraft, Path(path1.transition.list_items[1], Transition("Flyby", [path1.transition.list_items[2]]), False, True), path2)


def arc_distance(p1,p2,arc): # Calcul la distance entre deux points sur un arc

    center=arc.centre
    radius=arc.turn_radius
    c=p1.distance(p2)
    alpha=acos((-c**2+2*(radius**2))/(2*(radius**2)))   #alpha = angle entre deux droites (centre--p1 et centre--p2)
    
    return alpha*radius

def ortho_distance(p1,ortho):       # Calcul la distance entre un point sur le segment et le point end du segment
    return p1.distance(ortho.end)

def transition_distance(p1, p2, transition):    # Calcul la distance entre deux points sur une transition (flyby ou flyover)
    
    if transition.type=="Flyby":
        return arc_distance(p1,p2,transition.list_items[0])
    
    """Flyover traité comme des flyby particuliers"""
    elif transition.type=="Flyover":
        seg = transition.list_items[1]
        #Avion au niveau du premier arc
        if transition.boolarc1==True:
            return arc_distance(p1, seg.start, transition.list_items[0])+ortho_distance(seg.start,seg)+arc_distance(seg.end, p2, transition.list_items[2])
        #Avion au niveau du segment
        elif transition.boolseg==True:  
            return ortho_distance(p1,seg)+arc_distance(seg.end, p2, transition.list_items[2])
        #Avion au niveau du deuxieme arc
        elif transition.boolarc2==True:
            return arc_distance(p1, p2, transition.list_items[2])
        
        
        
def alongpath_distance(aircraft, path1, path2): #Calcul la along path distance entre les projetes de l'avion et du WPT sur la traj
    
    #avion au niveau de l'ortho
    if path1.boolorth==True and path1.booltrans==False:
        # path avec une transition differente de None
        if path1.transition.type!=None:
            distseg=ortho_distance(ortho_projection(aircraft,path1.ortho,None),path1.ortho) #distance sur le segment
            disttrans=transition_distance(path1.ortho.end,ortho_projection(g._TOWPT,path1.ortho,path1.transition),path1.transition) #distance sur la transition
        #path avec une transition None
        else:
            distseg = ortho_distance(ortho_projection(aircraft, path1.ortho, None), path1.ortho)
            disttrans=0
        return distseg+disttrans
    
    #Avion au niveau de la transition et avant le sequencement (boolactive=True)
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==True: 
    
        disttrans=transition_distance(ortho_projection(aircraft,path1.ortho,path1.transition),ortho_projection(g._TOWPT,path1.ortho,path1.transition),path1.transition)
        return disttrans
    
    #Avion au niveau de la transition apres le sequencement (boolactive=False) et la deuxieme transition est differente de none
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==False and path2.transition.type!=None : 
        disttrans1=transition_distance(ortho_projection(aircraft,path1.ortho,path1.transition),path2.ortho.start,path1.transition)
        distseg=ortho_distance(path2.ortho.start,path2.ortho)
        disttrans2=transition_distance(path2.ortho.end,ortho_projection(g._TOWPT,path2.ortho,path2.transition),path2.transition)
        return disttrans1+distseg+disttrans2

    #Avion au niveau de la transition apres le sequencement (boolactive=False) avec une deuxieme transition None
    elif path1.boolorth==False and path1.booltrans==True and path1.boolactive==False and path2.transition.type==None :

        disttrans1 = transition_distance(ortho_projection(aircraft, path1.ortho, path1.transition), path2.ortho.start, path1.transition)
        distseg = ortho_distance(path2.ortho.start, path2.ortho)
        return disttrans1+distseg
  


def ortho_projection(point, ortho, transition=None): #Renvoie la projection du point sur une ortho ou sur une transition 
    #projection sur l'ortho
    if transition==None:
        
        #cas particuliers
        if(ortho.start.x==ortho.end.x):
            return(Point(ortho.start.x,point.y))
        elif(ortho.start.y==ortho.end.y):
            return(Point(point.x,ortho.start.y))
        
        else:
            #calcul de l'equation de l'ortho
            a = (ortho.start.y-ortho.end.y)/(ortho.start.x-ortho.end.x) 
            b = ortho.start.y - a*ortho.start.x    
            #calcul de l'equation de la perpendiculaire à l'ortho
            a_norm = -1/a
            b_norm = point.y-(-1/a)*point.x
            #calcul du point d'intersection
            x = (b_norm-b)/(a-a_norm)
            y = a*x + b 
            
        return (Point(x,y))
    #projection sur la transition
    elif transition.type=="Flyby":       
        
        if transition.list_items[0].centre.x == point.x:    #cas où l'avion et le centre de la transition ont le meme x
            x=transition.list_items[0].centre.x
            y=transition.list_items[0].centre.y+transition.list_items[0].turn_radius
            v1=[ortho.end.x-ortho.start.x, ortho.end.y-ortho.start.y]
            v2=[x-ortho.end.x, y-ortho.end.y]
            if np.sign(np.vdot(v1,v2))<0:
                y=transition.list_items[0].centre.y-transition.list_items[0].turn_radius
        else:
            #calcul de l'equation de la droite centre--avion
            a = (transition.list_items[0].centre.y-point.y)/(transition.list_items[0].centre.x-point.x)
            b = point.y - a*point.x
            distcp=point.distance(transition.list_items[0].centre)
            radius=transition.list_items[0].turn_radius
            xa, ya, xc, yc = point.x, point.y, transition.list_items[0].centre.x, transition.list_items[0].centre.y
            x=(((1/(2*(yc-ya)))*(distcp**2-2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya))) #coordonnes en prenant les distances connues
            y=a*x+b
            v1=[ortho.end.x-ortho.start.x, ortho.end.y-ortho.start.y]
            v2=[x-ortho.end.x, y-ortho.end.y]
            if np.sign(np.vdot(v1,v2))<0:   
                 x=(((1/(2*(yc-ya)))*(distcp**2+2*distcp*radius-ya**2+yc**2-xa**2+xc**2))-b)/(a+((xc-xa)/(yc-ya)))
                 y=a*x+b
        return(Point(x,y))
    
    """flyover traités comme des flyby particuliers"""
    elif transition.type=="Flyover":
        
        if transition.boolarc1==True:
            return ortho_projection(point, ortho, Transition("Flyby",[transition.list_items[0]]))
        elif transition.boolseg==True:
            return ortho_projection(point ,transition.list_items[1], None)
        elif transition.boolarc2==True:
            return ortho_projection(point, transition.list_items[1], Transition("Flyby",[transition.list_items[2]]))

 
 def path_sequencing(point, path1, path2):      # Séquencement du path du leg actif (Permet de savoir si l'avion est au niveau de l'ortho ou de la transition)
    ortho1, trans1, ortho2 = path1.ortho, path1.transition, path2.ortho             # Récupération des données utiles aux calculs et stockage dans des variables locales
    xs1, ys1, xe1, ye1 = ortho1.start.x, ortho1.start.y, ortho1.end.x, ortho1.end.y
    xs2, ys2, xe2, ye2 = ortho2.start.x, ortho2.start.y, ortho2.end.x, ortho2.end.y 
    xc, yc = trans1.list_items[0].centre.x, trans1.list_items[0].centre.y
    xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
    proj = ortho_projection(point, ortho1, None)
    x1, y1 = proj.x, proj.y

    if path1.boolorth==True and path1.booltrans==False:                             # Dans le cas où l'avion est au niveau de l'ortho
        

        if not (((x1>=xs1 and x1<=xe1) or (x1<=xs1 and x1>=xe1)) and ((y1>=ys1 and y1<=ye1) or (y1<=ys1 and y1>=ye1))) and proj.distance(ortho1.end)<1:  
            # Si la projection de l'avion n'est plus dans l'ortho et qu'elle est proche  de la fin ( et donc que l'avion est maintenant au niveau de la transition) :
        
            path1.boolorth=False                                                    # Séquencement du path (ie l'avion est situé au niveau de la transition)
            path1.booltrans=True
            if trans1.type=="Flyby":                                                # Nécessité d'initialiser le signe entre l'avion et la droite passant par le centre de la transition et le waypoint pour un flyby
                if xc==xwpt:                                                        # Si le centre et le waypoint sont alignés (droite verticale)
                    g._SIGN=np.sign(point.x-xc)                                     # Calcul du signe (avion à droite ou à gauche de la droite verticale)
                else:
                    a=(yc-ywpt)/(xc-xwpt)                                           # Calcul de l'équation de la droite
                    b=yc-a*xc
                    g._SIGN=np.sign(point.y-(a*point.x+b))                          # Calcul du signe (avion au dessus ou en dessous de la droite)
                    
            
    elif path1.boolorth==False and path1.booltrans==True:                           # Dans le cas où l'avion est au niveau de la transition

        if trans1.type=="Flyby":                                                    # Pour un flyby
            proj = ortho_projection(point, ortho2, None)
            x2, y2 = proj.x, proj.y
            if (((x1 >= xs1 and x1 <= xe1) or (x1 <= xs1 and x1 >= xe1)) and ((y1 >= ys1 and y1 <= ye1) or (y1 <= ys1 and y1 >= ye1))) and path1.boolactive==True:
                                                                                    # Si l'avion revient dans l'ortho du path qui est encore actif 
                
                path1.boolorth = True                                               # Séquencement dans l'autre sens (l'avion est de nouveau au niveau de l'ortho)
                path1.booltrans = False
                
            elif (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))) and path1.boolactive==False:
                                                                                    # Si l'avion est au niveau de l'ortho du path suivant et le path précedent n'est plus actif
                
                path1.boolorth=False                                                # Séquencement du path (l'avion n'est plus au niveau de l'ortho ni de la transition)
                path1.booltrans=False
                g._LISTPATHS=g._LISTPATHS[1:]                                       # Suppression du path précedent
                g._LISTBANKANGLES=g._LISTBANKANGLES[1:]                             # Suppresion du bank angle correspondant au path venant d'être séquencé
                
        elif trans1.type=="Flyover":                                                # Pour un flyover 
            if trans1.boolarc1==True:                                               # Si l'avion est au niveau du premier arc
                proj = ortho_projection(point, trans1.list_items[1], None)          # Récupération des données utiles aux calculs et stockage dans des variables locales
                x2, y2 = proj.x, proj.y
                xs, ys = trans1.list_items[1].start.x, trans1.list_items[1].start.y
                xe, ye = trans1.list_items[1].end.x, trans1.list_items[1].end.y
                
                if (((x2>=xs and x2<=xe) or (x2<=xs and x2>=xe)) and ((y2>=ys and y2<=ye) or (y2<=ys and y2>=ye))):
                                                                                    # Si la projection de l'avion est dans le segment de la transition
                    
                    path1.transition.boolarc1, path1.transition.boolseg = False, True   # Séquencement du premier arc (l'avion est au niveau du segment)
                    
            elif trans1.boolseg==True:                                              # Si l'avion est au niveau du segment
                proj = ortho_projection(point, trans1.list_items[1], None)          # Récupération des données utiles aux calculs et stockage dans des variables locales
                x2, y2 = proj.x, proj.y
                xs, ys = trans1.list_items[1].start.x, trans1.list_items[1].start.y
                xe, ye = trans1.list_items[1].end.x, trans1.list_items[1].end.y
                
                if not (((x2>=xs and x2<=xe) or (x2<=xs and x2>=xe)) and ((y2>=ys and y2<=ye) or (y2<=ys and y2>=ye))):
                                                                                    # Si la projection de l'avion n'est plus dans le segment de la transition
                    path1.transition.boolseg, path1.transition.boolarc2 = False, True # Séquencement du segment (l'avion est au niveau du second arc)
                    
            elif trans1.boolarc2==True:                                             # Si l'avion est au niveau du second arc
                proj = ortho_projection(point, ortho2, None)                        # Récupération des données utiles aux calculs et stockage dans des variables locales
                x2, y2 = proj.x, proj.y
        
                if (((x2>=xs2 and x2<=xe2) or (x2<=xs2 and x2>=xe2)) and ((y2>=ys2 and y2<=ye2) or (y2<=ys2 and y2>=ye2))):
                                                                                    # Si l'avion est au niveau de l'ortho du path suivant
                    
                    path1.boolorth=False                                            # Séquencement du path (l'avion n'est plus au niveau de l'ortho ni de la transition)
                    path1.booltrans=False
                    g._LISTPATHS=g._LISTPATHS[1:]                                   # Suppression du path précedent
                    g._LISTBANKANGLES=g._LISTBANKANGLES[1:]                         # Suppresion du bank angle correspondant au path venant d'être séquencé
 

def active_leg(legs_list):     # Renvoie du leg actif et suppression du leg précédent
    legs_list=legs_list[1:]                                                         # Suppression du leg précédent
    active_leg=legs_list[0]                                                         # active_leg contient le numéro de la leg
    g._NUMSEQ=int(active_leg[0])                                                    # Stockage du numéro de séquencement
    return active_leg,legs_list                                                     # Renvoie la leg active et la nouvelle liste de legs


            
def sequencing_conditions(aircraft, path):          # Séquencement du leg actif
    
    if path.boolorth==False and path.booltrans==True and path.boolactive==True:     # Si l'avion est au niveau de la transition et que le leg n'a pas encore était sequencé :
        
        if path.transition.type=="Flyover":                                         # Pour un flyover
            if g._MODE=='NAV':                                                      # En mode NAV     
                g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)                     # Séquencement en envoyant le numéro du leg actif
                g._LISTPOINTS=g._LISTPOINTS[1:]                                     # Suppression du waypoint qu'on a dépassé de la liste
                g._TOWPT=g._LISTPOINTS[1]                                           # Modification du TOWPT au nouveau waypoint à atteindre
                print("ça séquence !")                                              # Contrôle du séquencement avec un message
                g._LISTPATHS[0].boolactive=False                                    # Le path du leg venant d'être séquencé, desactivation du path correspondant
                return True                                                         # Renvoie du booléen correspondant au sequencement
            
            elif g._MODE=='HDG':                                                    # En mode HDG
                hdg=aircraft.hdg
                v1=[-sin(hdg), cos(hdg)]                                            # Vecteur directeur de l'avion
                xs, ys, xe, ye = path.ortho.start.x, path.ortho.start.y, path.ortho.end.x, path.ortho.end.y
                v2=[xe-xs, ye-ys]                                                   # Vecteur directeur de l'ortho
                if np.sign(np.vdot(v1,v2))>0 and aircraft.distance(path.ortho.end)<5:  # L'avion doit être dans le même sens que la trajectoire et assez proche du waypoint (<5NM) 
                    g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)                 # Séquencement de la même manière que pour un flyover
                    g._LISTPOINTS=g._LISTPOINTS[1:]
                    g._TOWPT=g._LISTPOINTS[1]
                    print("ça séquence en mode hdg !")
                    g._LISTPATHS[0].boolactive=False
                    return True 
                
        elif path.transition.type=="Flyby" :                                        # Pour un flyby
            xc, yc = path.transition.list_items[0].centre.x, path.transition.list_items[0].centre.y
            xwpt, ywpt = g._TOWPT.x, g._TOWPT.y
            if xc==xwpt:                                                            # Cas où le waypoint est aligné avec le centre de la transition (droite verticale)       
                sgn=np.sign(aircraft.x-xc)                                          # Calcul du signe (avion à droite ou à gauche de la droite verticale)
            else:
                a=(yc-ywpt)/(xc-xwpt)                                               # Calcul de l'équation de la droite passant par le centre et par le waypoint
                b=yc-a*xc
                sgn=np.sign(aircraft.y-(a*aircraft.x+b))                            # Calcul du signe entre l'avion et la droite (avion au dessus ou en dessous de la droite)
            
            if g._SIGN!=sgn:                                                        # Si le signe est différent du signe initialisé
                if g._MODE=='NAV':                                                  # En mode NAV
                    g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)                 # Séquencement de la même manière qu'au dessus
                    g._LISTPOINTS=g._LISTPOINTS[1:]
                    g._TOWPT=g._LISTPOINTS[1]
                    print("ça séquence !")
                    g._LISTPATHS[0].boolactive=False
                    return True
                
                elif g._MODE=='HDG':                                                # En mode HDG                        
                    hdg=aircraft.hdg
                    proj=ortho_projection(aircraft, path.ortho, path.transition)
                    v1=[-sin(hdg), cos(hdg)]                                        # Vecteur directeur de l'avion
                    xe, ye = path.ortho.end.x, path.ortho.end.y1                    
                    v2=[proj.x-xe, proj.y-ye]                                       # Vecteur entre la fin de l'ortho et le projeté de l'avion sur la transition
                    if np.sign(np.vdot(v1,v2))>0 and aircraft.distance(path.ortho.end)<5:   # L'avion doit être dans le même sens que la trajectoire et assez proche du waypoint (<5NM) 
                        g._ACTIVELEG, g._LEGLIST=active_leg(g._LEGLIST)             # Séquencement de la même manière qu'au dessus
                        g._LISTPOINTS=g._LISTPOINTS[1:]
                        g._TOWPT=g._LISTPOINTS[1]
                        print("ça séquence en mode hdg !")
                        g._LISTPATHS[0].boolactive=False
                        return True 
        
    return False                                                                    # Renvoie de False quand il n'y a pas de séquencement
     

def bank_angle(aircraft, path1, path2):     # Calcul du bank angle
    
    if path1.boolorth==True and path1.booltrans==False:                             # Pas d'envoi de bank angle quand l'avion est sur l'orhto (ligne droite)
        return 0

    
    elif path1.boolorth==False and path1.booltrans==True:                           # Cas où l'avion est sur une transition
        
        if path1.transition.type=="Flyby":    
            return g._LISTBANKANGLES[0]                                             # Renvoie la valeur de la liste pour un Flyby
            
        elif path1.transition.type=="Flyover":                                      # Pour un Flyover :         
            
            if path1.transition.boolarc1==True:             
                return g._LISTBANKANGLES[0][0]                                      # Renvoie la valeur correspondant au premier arc si l'avion est sur cet arc
               
            elif path1.transition.boolseg==True:
                return 0                                                            # Renvoie 0 si l'avion est sur le segment de la transition (ligne droite)
                                
            elif path1.transition.boolarc2==True:
                return g._LISTBANKANGLES[0][1]                                      # Renvoie la valeur correspondant au second arc si l'avion est sur cet arc
   
    
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





















