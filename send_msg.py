# Envoi des messages

from ivy.std_api import *
import time
from functions import *

_TIME=0


def sendActiveLeg(numSeq):
    IvySendMsg("GS_AL Time="+str(_TIME)+" NumSeqActiveLeg="+str(numSeq))
    
def sendNewLegList(LegList):  #format de leg liste : "ID=WPT1 SEQ=0 COURSE=110  LAT= LON= ;ID=WPT2 SEQ=1 COURSE=150 LAT= LON=  ;............"
    C=""
    for i in range (len(LegList)):
        C+="ID="
        C+=LegList[i][0]
        C+=" "
        C+="SEQ="
        C+=LegList[i][1]
        C+=" "
        C+="COURSE="
        C+=LegList[i][2]
        C+=" "
        C+="LAT="
        C+=LegList[i][3]
        C+=" "
        C+="LON="
        C+=LegList[i][4]
        C+=" "
        C+=";"
    IvySendMsg("GS_AL Time="+str(_TIME)+" LegList="+C)
    
def sendAlongPathDistance(alongpath_distance):
    IvySendMsg("GS_AL Time="+str(_TIME)+" AlongPathDistance="+str(alongpath_distance))
    
def sendData(xtk, tae, dtwpt, bank_angle):
    IvySendMsg("GS_Data Time="+str(_TIME)+" XTK="+str(xtk)+" TAE="+str(tae)+" DTWPT="+str(dtwpt)+" BANK_ANGLE_REF="+str(bank_angle))
    
    
#def sendBankAngle(bankangle):
 #   IvySendMsg("")