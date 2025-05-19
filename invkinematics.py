from math import *

class ThreeRPSInvKinematicsSolver():
    def __init__(self, d, e, f, g):
        self.d = d # Distance from center of base to any corner
        self.e = e # Distance from center of platform to any corner
        self.f = f # Link 1 Length
        self.g = g # Link 2 Length

    def theta(self, leg, hz, nx, ny):
        # Unit normal vector
        nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1)
        nx = nx / nmag
        ny = ny / nmag
        nz = 1 / nmag

        # Angle of Leg A, B, or C
        match leg:
            case 1: # Leg A
                y = self.d + (self.e / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))))
                z = hz + self.e * ny
                mag = sqrt(pow(y, 2) + pow(z, 2))
                angle = acos(y / mag) + acos((pow(mag, 2) + pow(self.f, 2) - pow(self.g, 2)) / (2 * mag * self.f))
            case 2: # Leg B
                x = (sqrt(3) / 2) * (self.e * (1 - (pow(nx, 2) + sqrt(3) * nx * ny) / (nz + 1)) - self.d)
                y = x / sqrt(3)
                z = hz - (self.e / 2) * (sqrt(3) * nx + ny)
                mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
                angle = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(self.f, 2) - pow(self.g, 2)) / (2 * mag * self.f))
            case 3: # Leg C
                x = (sqrt(3) / 2) * (self.d - self.e * (1 - (pow(nx, 2) - sqrt(3) * nx * ny) / (nz + 1)))
                y = -x / sqrt(3)
                z = hz + (self.e / 2) * (sqrt(3) * nx - ny)
                mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
                angle = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(self.f, 2) - pow(self.g, 2)) / (2 * mag * self.f))

        return 180 - (angle * (180 / pi))
    