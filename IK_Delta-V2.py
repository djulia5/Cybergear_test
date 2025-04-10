import math

# Dimensions du robot
e = 214.95  # Rayon de la base mobile (mm)
f = 424.7  # Rayon de la base fixe (mm)
re = 538  # Longueur des bras passifs (mm)
rf = 233  # Longueur des bras actifs (mm)

# Constantes trigonométriques
sqrt3 = math.sqrt(3.0)
pi = math.pi
tan30 = 1 / sqrt3


# Fonction auxiliaire pour calculer l'angle dans le plan YZ
def delta_calc_angle_yz(x0, y0, z0):
    # Coordonnées dans le plan YZ (compensation de la base mobile)
    y1 = -0.5 * tan30 * f  # Décalage de la base fixe
    y0 -= 0.5 * tan30 * e  # Décalage de la base mobile

    # Résolution quadratique pour l'angle
    a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
    b = (y1 - y0) / z0
    d = -(a + b * y1) * (a + b * y1) + rf * (1 + b**2) * rf

    if d < 0:
        return None  # Position non réalisable

    yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
    zj = a + b * yj
    theta = math.degrees(math.atan2(-zj, y1 - yj))  # Calcul de l'angle
    return theta


# Fonction principale pour la cinématique inverse
def delta_calc_inverse(x0, y0, z0):
    # Angle pour BLDC0
    theta1 = delta_calc_angle_yz(x0, y0, z0)
    if theta1 is None:
        return None

    # Rotation des coordonnées pour BLDC1
    x1 = x0 * math.cos(2 * pi / 3) + y0 * math.sin(2 * pi / 3)
    y1 = y0 * math.cos(2 * pi / 3) - x0 * math.sin(2 * pi / 3)
    theta2 = delta_calc_angle_yz(x1, y1, z0)
    if theta2 is None:
        return None

    # Rotation des coordonnées pour BLDC2
    x2 = x0 * math.cos(4 * pi / 3) + y0 * math.sin(4 * pi / 3)
    y2 = y0 * math.cos(4 * pi / 3) - x0 * math.sin(4 * pi / 3)
    theta3 = delta_calc_angle_yz(x2, y2, z0)
    if theta3 is None:
        return None

    return theta1, theta2, theta3


# Interface utilisateur
if __name__ == "__main__":
    print("Entrez les coordonnées de la position cible :")
    x = float(input("X (mm) : "))
    y = float(input("Y (mm) : "))
    z = float(input("Z (mm) : "))

    print("\nDimensions du robot :")
    print(f"  rf = {rf} mm (longueur des bras actifs)")
    print(f"  re = {re} mm (longueur des bras passifs)")
    print(f"  f  = {f} mm (coté base fixe)")
    print(f"  e  = {e} mm (coté triangle base mobile)\n")

    angles = delta_calc_inverse(x, y, z)
    if angles is None:
        print("Position non réalisable avec la géométrie actuelle.")
    else:
        theta1, theta2, theta3 = angles
        print(f"Angles calculés :")
        print(f"  Servo0 : {theta1:.2f}°")
        print(f"  Servo1 : {theta2:.2f}°")
        print(f"  Servo2 : {theta3:.2f}°")
