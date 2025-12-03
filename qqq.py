class Quaternion:
    w: float
    x: float
    y: float
    z: float

    # Конструктор
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    # Сложение
    def __add__(self, other):
        return Quaternion(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)
    
    # Умножение
    def __mul__(self, other):
        if isinstance(other, float):
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other)
        else:
            w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
            x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y  
            y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
            z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)
    
    # Сопряжение
    def sopryazh(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    


    
