import pybullet as pb
import pybullet_data
import time
import numpy as np

class Env():
    def __init__(self):
        super(Env, self).__init__()
        
        # Pripojenie k GUI PyBullet simulátoru
        pb.connect(pb.GUI)
        # Nastavenie gravitácie (práve zjednodušené na zemskej gravitácii)
        pb.setGravity(0, 0, -9.8)

    def robot_model(self):
        # Nastavenie cesty pre prednastavené objekty (napr. plane.urdf pre podlahu)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  
        
        # Načítanie podlahy do simulácie
        plane_id = pb.loadURDF("plane.urdf")  # Tento objekt sa načíta v každej simulácii ako podlaha
        
        # Načítanie robota (Franka Panda robot) na začiatok na pozícii (0, 0, 0)
        # `useFixedBase=True` znamená, že robot bude pevne umiestnený na podlahe
        self.robot_id = pb.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True) 
        
        # Nastavenie počiatočných pozícií všetkých kĺbov robota na hodnotu 0
        # Týmto sa zabezpečí, že robot bude mať všetky kĺby v počiatočnej pozícii (0, 0, 0)
        for joint_number in range(pb.getNumJoints(self.robot_id)):
            pb.resetJointState(self.robot_id, joint_number, 0)

    def move_joint(self, robot_id, joint_index, target_angle):
        # Funkcia pre pohyb kĺbu robota do požadovaného uhla (cieľsový uhol)
        pb.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=pb.POSITION_CONTROL,  # Móde riadenia: riadenie pozície kĺbu
            targetPosition=target_angle  # Požadovaný cieľový uhol
        )

    def inverse_kinematics(self, target_position):
        # Vypočíta inverznú kinematiku pre danú cieľovú pozíciu v priestore
        # Tu sa používajú kĺby robota na to, aby sa dostali do požadovanej pozície.
        # Index 7 odkazuje na koncový efektor (robotova ruka)
        joint_angles = pb.calculateInverseKinematics(self.robot_id, 7, target_position)
        return joint_angles  # Vráti zoznam kĺbových uhlov potrebných na dosiahnutie cieľovej pozície

    def visualize_circle_and_points(self, center, radius, num_points):
        # Vytvára vizualizáciu bodov a kruhu v 3D priestore
        # Vytvorí guľu (vizuálny tvar) pre každý bod na kruhu, ktoré budú modré
        sphere_visual_shape = pb.createVisualShape(shapeType=pb.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1]) 
        
        # Vytvárame zoznam uhlov pre rozdelenie bodov na kruhu
        angles = np.linspace(0, 2 * np.pi, num_points)
        points = []  # Zoznam na uloženie bodov
        for angle in angles:
            # Pre každý uhol vypočítame pozíciu bodu na kruhu
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]  # Udržiavame výšku rovnakú pre všetky body
            print(f"Creating point at: ({x}, {y}, {z})")  # Logovanie pozície bodu
            # Vytvorenie bodu (gule) na danej pozícii
            pb.createMultiBody(baseMass=0, 
                               baseVisualShapeIndex=sphere_visual_shape,
                               basePosition=[x, y, z])
            points.append([x, y, z])  # Uložíme bod do zoznamu

        # Vytvorenie čiar medzi bodmi (na zobrazenie spojenia bodov na kruhu)
        for i in range(num_points):
            # Pre každý bod vytvoríme čiaru medzi týmto bodom a ďalším bodom (cirkulárne)
            x1, y1, z1 = points[i]
            x2, y2, z2 = points[(i + 1) % num_points]
            pb.addUserDebugLine([x1, y1, z1], [x2, y2, z2], lineColorRGB=[1, 0, 0], lineWidth=2)  # Červená farba
        return points  # Vrátime zoznam bodov

    def move_along_circle(self, points, speed=0.1):
        # Funkcia na pohyb robota podél kruhu (prejdeme cez všetky body na kruhu)
        i = 0  # Začneme na prvom bode
        while True:
            # Cieľová pozícia efektora je bod na kruhu
            target_position = points[i]
            joint_angles = self.inverse_kinematics(target_position)  # Vypočíta kĺbové uhly pre daný bod
            #print(f"Joint angles for position {target_position}: {joint_angles}")
            
            # Ak sú uhly správne, nastavíme pozíciu všetkých kĺbov robota
            if len(joint_angles) > 0:
                for joint_index in range(len(joint_angles)):
                    self.move_joint(self.robot_id, joint_index, joint_angles[joint_index])
            
            # Prejdeme na ďalší bod na kruhu
            i = (i + 1) % len(points)  # Ak sme dosiahli posledný bod, vrátime sa na prvý bod
            
            pb.stepSimulation()  # Krok simulácie (robot sa pohybuje)
            time.sleep(speed)  # Časový interval medzi pohybmi (rýchlosť pohybu robota)

# Vytvorenie inštancie simulácie
env = Env()

if __name__ == "__main__":
    # Načítanie modelu robota
    env.robot_model()
    
    # Parametre pre kruh (polomer a počet bodov)
    radius = 0.2        # Polomer kruhu
    num_points = 11     # Počet bodov na kruhu (vytvorí sa 11 bodov)
    
    # Vytvorenie bodov na kruhu (bude sa vytvárať kruh okolo bodu center)
    center = [0.75, 0, 0.75]  # Stred kruhu (x, y, z)
    points = env.visualize_circle_and_points(center, radius, num_points)
    
    # Presun robota po bodoch na kruhu
    env.move_along_circle(points, speed=0.05)  # Rýchlosť pohybu robota, zníženie hodnoty speed pre plynulý pohyb

    # Odpojenie od PyBullet po skončení simulácie
    pb.disconnect()
