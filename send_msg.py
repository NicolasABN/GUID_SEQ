# Envoi des messages

from ivy.std_api import *
import global_variables as g


def sendActiveLeg(numSeq):          #Envoie le numéro de séquencencement du leg actif sur le bus IVY
    IvySendMsg("GS_AL Time="+str(g._TIME)+" NumSeqActiveLeg="+str(numSeq))

   
def sendData(xtk, tae, dtwpt, bank_angle, alongpath_distance):      #Envoie les infos calculées sur le bus IVY
    IvySendMsg("GS_Data Time="+str(g._TIME)+" XTK="+str(xtk)+" TAE="+str(tae)+" DTWPT="+str(dtwpt)+" BANK_ANGLE_REF="+str(bank_angle)+" ALDTWPT="+str(alongpath_distance))
    
    
