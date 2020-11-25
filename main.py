# -*- coding: utf-8 -*-
"""
Created on Wed Nov 18 15:43:09 2020

@author: aubin
"""
from recep_msg import *
from send_msg import *
import time


if __name__=='__main__':
    app_name="GUID_SEQ_APP"
    ivy_bus="127.0.0.1:2010"
    bus=IvyInit(app_name,"GUID_SEQ is Ready",0,on_cx,on_die)
    IvyStart(ivy_bus)
    time.sleep(1)
    
    #sendData(11,25,35,1)
    IvyBindMsg(recepTime,'^Time t=(.*)')
    
    IvyBindMsg(recepLegList,'^FL_LegList Time=(.*) LegList=(.*)') # Reception de la liste des legs
    
    IvyBindMsg(recepPoints,'^GT_TRAJ Liste_Points=(.*)') # Reception des points
    IvyBindMsg(recepSegments,'^GT_TRAJ Liste_Segments=(.*)') # Reception des segments
    IvyBindMsg(recepOrthos, '^GT_TRAJ Liste_Orthos=(.*)')
    IvyBindMsg(recepTransitions,'^GT_TRAJ Liste_Transitions=(.*)') # Reception des transitions
    IvyBindMsg(recepPaths,'^GT_TRAJ Liste_Paths=(.*)') # Reception des paths
    IvyBindMsg(recepStateVector,'^StateVector x=(.*) y=(.*) z=(.*) Vp=(.*) fpa=(.*) psi=(.*) phi=(.*)') # Reception du vecteur d'état 
    IvyMainLoop()
    