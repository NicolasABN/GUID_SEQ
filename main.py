# -*- coding: utf-8 -*-
"""
Created on Wed Nov 18 15:43:09 2020

@author: aubin
"""
from recep_msg import *
import time


if __name__=='__main__':
    
    #Connection au bus IVY
    app_name="GUID_SEQ_APP"
    ivy_bus=""
    bus=IvyInit(app_name,"GUID_SEQ is Ready",0,on_cx,on_die)
    IvyStart(ivy_bus)
    time.sleep(1)
    
    #Abonnement aux messages
    
    IvyBindMsg(recepTime,'^Time t=(.*)')                                # Reception du temps
    IvyBindMsg(recepMode, 'GC_AP Time=(.*) AP_State=(.*) AP_Mode=(.*)') # Reception du mode de l'AP (NAV ou HDG)

    IvyBindMsg(recepLegList,'^FL_LegList Time=(.*) LegList=(.*)')       # Reception de la liste des legs
    IvyBindMsg(recepBankAngles,'^GT Liste_BankAngles=(.*)')             # Reception des bankangles de la trajectoire

    # Reception de la trajectoire
    IvyBindMsg(recepPoints,'^GT Liste_Points=(.*)')                     # Reception des points
    IvyBindMsg(recepSegments,'^GT Liste_Segments=(.*)')                 # Reception des segments
    IvyBindMsg(recepOrthos, '^GT Liste_Orthos=(.*)')                    # Reception des orthos
    IvyBindMsg(recepTransitions,'^GT Liste_Transitions=(.*)')           # Reception des transitions
    IvyBindMsg(recepPaths,'^GT Liste_Paths=(.*)')                       # Reception des paths
    
    # Reception du vecteur d'etat (position de l'avion et heading)
    IvyBindMsg(recepStateVector,'^StateVector x=(.*) y=(.*) z=(.*) Vp=(.*) fpa=(.*) psi=(.*) phi=(.*)') # Reception du vecteur d'état 
    
    
    IvyMainLoop()
    
