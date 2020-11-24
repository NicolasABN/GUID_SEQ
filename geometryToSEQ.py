"""Geometry classes and utilities."""


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

class Transition(object):
    def __init__(self, centre, turnRadius, leadDistance):
        self.centre = centre
        self.turn_radius, self.lead_distance = turnRadius, leadDistance

class Path(object):
    def __init__(self, ortho, transition):
        self.ortho = ortho
        self.transition = transition
        self.boolorth = True  
        self.booltrans = False
        
    def __repr__(self):
        return(self.boolorth, self.booltrans)

class Trajectoire(object):
    def __init__(self):
        self.path_list = []  # liste des paths

    def __repr__(self):
        return str(self.path_list)

class Waypoint(Point):
    """Définit une classe qui hérite de Point avec un id pour identifer le waypoint"""
    def __init__(self,id,x,y):
        super().__init__(x,y)
        self.id=id

        
class Aircraft(Point):
    
    def __init__(self,x,y,hdg):
        super().__init__(x,y)
        self.hdg=hdg
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        