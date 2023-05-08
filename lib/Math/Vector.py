import math

class Vector2:
    def __init__(self, x = None, y = None):
        if type(x) in [list, tuple] and len(x) == 2:
            self.x, self.y = x
        elif type(x) == Vector2:
            self.x, self.y = float(x.x), float(x.y)
        elif x == None and y == None:
            self.x, self.y = (0.0, 0.0)
        elif y == None:
            self.x, self.y = (x, x)
        else:
            self.x, self.y = (x, y)
            
    # Other constructors
    @staticmethod
    def unit_from_angle(angle):
        """
        Retourne un vecteur unité tq self.angle_to_horizon() == angle
        """
        return Vector2(
            math.cos(angle),
            math.sin(angle)
        )
    @staticmethod
    def unit_from_angle_deg(angle):
        """
        Retourne un vecteur unité tq self.angle_to_horizon_deg() == angle
        """
        angle = math.radians(angle)
        return Vector2(
            math.cos(angle),
            math.sin(angle)
        )
    @staticmethod
    def from_polar(mag, angle):
        return Vector2.unit_from_angle(angle) * mag
    @staticmethod
    def from_polar_deg(mag, angle):
        return Vector2.unit_from_angle_deg(angle) * mag


    # Additions / Substractions
    def add(self, vec):
        return Vector2(
            self.x + vec.x,
            self.y + vec.y
        )
    def add_ip(self, vec):
        self.x += vec.x
        self.y += vec.y
    def __add__(self, vec):
        return self.add(vec)
    def __neg__(self):
        return Vector2(-self.x, -self.y)
    def __sub__(self, vec):
        return self + (-vec)

    # Multiplications
    def __mul__(self, other):
        if type(other) == Vector2:
            return Vector2(self.x * other.x, self.y * other.y)
            # return self.dot(other) # Dot product or cross product (3D...) ?

        else:
            return Vector2(
                self.x * other,
                self.y * other
            )
    def __truediv__(self, other):
        if type(other) == Vector2:
            return Vector2(self.x / other.x, self.y / other.y)
        else:
            return self * (1/other)
    
    
    def dot_product(self, other):
        return self.x * other.x + self.y * other.y
    dot = dot_product


    # Operations on length
    def mag_sqr(self):
        return self.x ** 2 + self.y ** 2
    def mag(self):
        return math.sqrt(self.mag_sqr())
    mag_squared = mag_sqr
    norm_squared = mag_sqr
    norm = mag
    
    def __abs__(self):
        return self.mag()
    def normalize(self):
        mag = self.mag()
        return Vector2(
            self.x / mag,
            self.y / mag
        )
    def to_mag(self, new_mag):
        return self * (new_mag/self.mag())
    
    def rotate_by_angle(self, theta_rad):
        c = math.cos(theta_rad)
        s = math.sin(theta_rad)

        x = self.x
        y = self.y

        return Vector2(
            c * x - s * y,
            s * x + c * y
        )
    
    
    # Angular operations (all clockwise)
    def angle_to_horizon(self):
        if self.x != 0:
            if self.x >= 0 and self.y >= 0:
                return math.atan(self.y / self.x)
            elif self.x >= 0 and self.y <= 0:
                return math.atan(self.y / self.x) + (2 * math.pi)
            else: # self.x <= 0
                return math.atan(self.y / self.x) + (math.pi)
        else:
            return math.pi / 2
    def angle_to_horizon_deg(self):
        return math.degrees(self.angle_to_horizon())
    
    def to_pygame(self):
        """
        returns a tuple containing the x and y coordinates as integers
        """
    
        return int(self.x), int(self.y)
    def to_int(self):
        return Vector2(
            int(self.x),
            int(self.y)   
        )
    

    def equals(self, obj):
        return obj.x == self.x and obj.y == self.y
    def __str__(self):
        return f'<Vector2>({self.x}, {self.y})'
    def __repr__(self):
        return str(self)

if __name__ == '__main__':
    Vector = Vector2

    v1 = Vector(2, 4)
    v2 = Vector(3,4)

    print(v1 == v2)



    
