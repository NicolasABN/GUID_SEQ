"""Geometry classes and utilities."""
import numpy as np

class Point(object):
    """Nm coordinates, with attributes x, y: int"""

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "({0.x}, {0.y})".format(self)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __rmul__(self, k):
        return Point(k * self.x, k * self.y)

    def __abs__(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def sca(self, other):
        """sca(Point) return float
        returns the scalar product between self and other"""
        return self.x * other.x + self.y * other.y

    def det(self, other):
        """det(Point) return float
        returns the determinant between self and other"""
        return self.x * other.y - self.y * other.x

    def egal(self, other):
        return self.x == other.x and self.y == other.y

    def distance(self, other):
        return abs(self - other)

    def multiplie(self, scalaire):
        """multiplie les coordonnees du point par un scalaire"""
        return Point(self.x * scalaire, self.y * scalaire)

    def milieu(self, other):
        """trouve le milieu de deux points et renvoie ce nouveau point"""
        new_x = (self.x + other.x) / 2
        new_y = (self.y + other.y) / 2
        return Point(new_x, new_y)

    def seg_dist(self, a, b):
        """Distance from point to segment
        @param a,b @e Point: Description of segment
        @return Distance from Point self to segment [a, b]"""
        ab, ap, bp = b - a, self - a, self - b
        if ab.sca(ap) <= 0:
            return abs(ap)
        elif ab.sca(bp) >= 0:
            return abs(bp)
        else:
            return abs(ab.det(ap)) / abs(ab)

class Segment(object):
    """Définit une classe Segment qui correspond à la représentation graphique
    d'un LEG"""
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def norm(self):
        return ((self.end.x - self.start.x)**2+(self.end.y - self.start.y)**2)**0.5

    def affix(self):
        return self.end.x-self.start.x,self.end.y-self.start.y

    def scal(self, other):
        return self.affix()[0]*other.affix()[0] + self.affix()[1]*other.affix()[1]

    def det(self,other):
        return self.affix()[0]*other.affix()[1] - self.affix()[1]*other.affix()[0]


class Ortho(Segment):
    """Définit une classe qui hérite de Segement mais qui correspond à la portion
    de segment comprise dans la trajectoire de référence"""
    def __init__(self, start, end):
        super().__init__(start, end)
        
class Arc(object):
    def __init__(self, centre, turn_radius, lead_distance):
        self.centre = centre
        self.turn_radius = turn_radius
        self.lead_distance = lead_distance


class Transition(object):
    def __init__(self, type, list_items):
        self.type = type
        self.list_items = list_items
        self.boolarc1=True
        self.boolseg=False
        self.boolarc2=False

class Path(object):
    def __init__(self, ortho, transition, bool1=True, bool2=False):
        self.ortho = ortho
        self.transition = transition
        self.boolorth = bool1  
        self.booltrans = bool2
        self.boolactive = True
        
    def __repr__(self):
        return(self.boolorth, self.booltrans)

class Trajectoire(object):
    def __init__(self):
        self.path_list = []  # liste des paths

    def __repr__(self):
        return str(self.path_list)
    
#FL_LegList Time=1 LegList=(ID=LFMN SEQ=0 LAT=N43400000 LON=E007130000 COURSE=0 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=OKTET SEQ=1 LAT=N44290600 LON=E006341000 COURSE=320 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=BULOL SEQ=2 LAT=N46024500 LON=E005053100 COURSE=317 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=PIBAT SEQ=3 LAT=N46482100 LON=E004153300 COURSE=315 FLY=fly_over FLmin=100 FLmax=400 SPEEDmax=350; ID=OKRIX SEQ=4 LAT=N47575800 LON=E003340300 COURSE=335 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=AMODO SEQ=5 LAT=N48251000 LON=E002584900 COURSE=304 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=RESMI SEQ=6 LAT=N48340700 LON=E002113100 COURSE=283 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=LGL SEQ=7 LAT=N48472600 LON=E000314900 COURSE=280 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350; ID=LFRG SEQ=8 LAT=N49214800 LON=E000093600 COURSE=340 FLY=fly_by FLmin=100 FLmax=400 SPEEDmax=350)
#GT Liste_Points=[Point(433.7773534331124, 2923.327298318111),Point(394.87430364484646, 2991.796113659577),Point(306.06515264751283, 3125.10641933703),Point(256.0087821474177, 3191.3814894610828),Point(214.43427829643812, 3294.394590662351),Point(179.13769149283542, 3335.2706779083874),Point(131.7527750554538, 3348.800332717158),Point(31.873786285751027, 3369.005123462615),Point(9.617234625768775, 3421.5647089876284)]
#GT Liste_Segments=[Segment(Liste_Points[0], Liste_Points[1]),Segment(Liste_Points[1], Liste_Points[2]),Segment(Liste_Points[2], Liste_Points[3]),Segment(Liste_Points[3], Liste_Points[4]),Segment(Liste_Points[4], Liste_Points[5]),Segment(Liste_Points[5], Liste_Points[6]),Segment(Liste_Points[6], Liste_Points[7]),Segment(Liste_Points[7], Liste_Points[8])]'
#GT Liste_Transitions=[Transition("fly_by",[Arc(Point(353.4401736908345, 2966.2679928628163), 48.63631380275426, 1.7265735257898764)]), Transition("fly_by",[Arc(Point(266.38673721710717, 3096.942817430543), 48.63631380275426, 1.440174675145205)]), Transition("fly_over",[Arc(Point(271.2712787487191, 3202.9089791069055), 19.126599803396417, 4.2551260934304205), Segment(Point(252.56113250197532, 3198.9394391505953), Point(253.0038218680359, 3196.8528540873285)), Arc(Point(205.42649220645356, 3186.7588597050594), 48.63631380271498, 4.2551260934304205)]), Transition("fly_by",[Arc(Point(192.23338777970744, 3280.846054834948), 25.658064321598626, 4.2551260934304205)]), Transition("fly_by",[Arc(Point(171.13394066681548, 3322.737442782298), 14.249067055069359, 4.255126093430421)])]
#GT Liste_Orthos=[Ortho(Point(433.7773534331124, 2923.327298318111), Point(395.727252410505, 2990.294935940877)), Ortho(Point(393.917053486969, 2993.233029897637), Point(306.86361701324165, 3123.907854465364)), Ortho(Point(305.19716780825433, 3126.255638817263), Point(256.0087821474177, 3191.3814894610828)), Ortho(Point(250.52821159493377, 3204.9612202255785), Point(216.0267784913966, 3290.4487015103286)), Ortho(Point(211.65329103129065, 3297.615181897215), Point(181.9186787579829, 3332.0500866735238)), Ortho(Point(175.0460830808096, 3336.43894100959), Point(133.58996788507065, 3348.2757652667815)), Ortho(Point(129.88009343192775, 3349.1791625471583), Point(36.59074809257889, 3368.05091650242)), Ortho(Point(29.997224284542316, 3373.43668691873), Point(9.617234625768775, 3421.5647089876284))]'
#GT Liste_Paths=[Path(Liste_Orthos[0], Liste_Transitions[0]), Path(Liste_Orthos[1], Liste_Transitions[1]), Path(Liste_Orthos[2], Liste_Transitions[2]), Path(Liste_Orthos[3], Liste_Transitions[3]), Path(Liste_Orthos[4], Liste_Transitions[4]), Path(Liste_Orthos[5], Transition(None, None))]
#FL_LegList Time=0.0 LegList=(ID=LFMN SEQ=0 LAT=N43395546 LONE007125394 COURSE=000 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=OKTET SEQ=1 LAT=N44290600 LONE006341000 COURSE=327 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=GIPNO SEQ=2 LAT=N45333600 LONE005314500 COURSE=325 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=BULOL SEQ=3 LAT=N46024500 LONE005053100 COURSE=345 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MOMIL SEQ=4 LAT=N46324600 LONE004324800 COURSE=301 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=ATN SEQ=5 LAT=N46482140 LONE004153290 COURSE=323 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=AVLON SEQ=6 LAT=N47333600 LONE003484800 COURSE=332 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=OKRIX SEQ=7 LAT=N47575800 LONE003340300 COURSE=338 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=TELBO SEQ=8 LAT=N48252700 LONE002515300 COURSE=321 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MLN SEQ=9 LAT=N48272080 LONE002484780 COURSE=313 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=AGOGO SEQ=10 LAT=N48311200 LONE002423800 COURSE=313 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=LFRG SEQ=11 LAT=N49214822 LONE000093599 COURSE=301 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=LFMN SEQ=0 LAT=N43395546 LONE007125394 COURSE=000 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=OKTET SEQ=1 LAT=N44290600 LONE006341000 COURSE=327 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=GIPNO SEQ=2 LAT=N45333600 LONE005314500 COURSE=325 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=BULOL SEQ=3 LAT=N46024500 LONE005053100 COURSE=345 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MOMIL SEQ=4 LAT=N46324600 LONE004324800 COURSE=301 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=ATN SEQ=5 LAT=N46482140 LONE004153290 COURSE=323 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=AVLON SEQ=6 LAT=N47333600 LONE003484800 COURSE=332 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=OKRIX SEQ=7 LAT=N47575800 LONE003340300 COURSE=338 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=TELBO SEQ=8 LAT=N48252700 LONE002515300 COURSE=321 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MLN SEQ=9 LAT=N48272080 LONE002484780 COURSE=313 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=AGOGO SEQ=10 LAT=N48311200 LONE002423800 COURSE=313 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=LFRG SEQ=11 LAT=N49214822 LONE000093599 COURSE=301 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=LFRG SEQ=11 LAT=N49214822 LONE000093599 COURSE=301 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000)
#FL_LegList Time=279.0 LegList=(ID=LFMN SEQ=0 LAT=N43395546 LON=E007125394 COURSE=000 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=OKTET SEQ=1 LAT=N44290600 LON=E006341000 COURSE=327 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=GIPNO SEQ=2 LAT=N45333600 LON=E005314500 COURSE=325 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=BULOL SEQ=3 LAT=N46024500 LON=E005053100 COURSE=345 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MOMIL SEQ=4 LAT=N46324600 LON=E004324800 COURSE=301 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=ATN SEQ=5 LAT=N46482140 LON=E004153290 COURSE=323 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=AVLON SEQ=6 LAT=N47333600 LON=E003484800 COURSE=332 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=OKRIX SEQ=7 LAT=N47575800 LON=E003340300 COURSE=338 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=TELBO SEQ=8 LAT=N48252700 LON=E002515300 COURSE=321 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=MLN SEQ=9 LAT=N48272080 LON=E002484780 COURSE=313 FLY=Flyby FLmin=FL195 FLmax=FL460 SPEEDmax = 000; ID=AGOGO SEQ=10 LAT=N48311200 LON=E002423800 COURSE=313 FLY=Flyby FLmin=FL000 FLmax=FL460 SPEEDmax = 000; ID=LFRG SEQ=11 LAT=N49214822 LON=E000093599 COURSE=301 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000; ID=LFRG SEQ=11 LAT=N49214822 LON=E000093599 COURSE=301 FLY=Flyby FLmin=FL000 FLmax=FL195 SPEEDmax = 000)


class Waypoint(Point):    # Prend en paramètre lat et lon en degrés (quand on rentre des str) ou en NM (quand on rentre des entiers)
    """Définit une classe qui hérite de Point avec un id pour identifer le waypoint"""
    def __init__(self,lat,lon):      
        strlat, strlon = lat, lon
        if type(strlat)==str:
            lat=float(strlat[1:3])
            lat+=float(strlat[3:5])/60
            lat+=float(strlat[5:7])/3600
            lat+=float(strlat[7:9])/36000
            lon=float(strlon[1:4])
            lon+=float(strlon[4:6])/60
            lon+=float(strlon[6:8])/3600
            lon+=float(strlon[8:10])/36000
            if strlat[0]!="N":
                lat*=-1   
            if strlon[0]!="E":
                lon*=-1
        
            lon=lon*np.pi/180
            lat=lat*np.pi/180
            R = 6371007 # Rayon de la Terre
            a = 6378137 # Demi grand axe de l'ellipsoide de reference WGS-84 (m)
            x = a*lon/1852
            y = a*np.log(np.tan(np.pi/4+lat/2))/1852
        else:
            x=lon
            y=lat
        super().__init__(x,y)



class Aircraft(Point):
    
    def __init__(self,x,y,hdg):
        super().__init__(x,y)
        self.hdg=hdg
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        