Tento projekt simuluje pohyb robota Franka Emika Panda pomocou PyBullet. Robot sleduje kruhovú trajektóriu definovanú množinou bodov v 3D priestore. Simulácia obsahuje vizualizáciu trajektórie a plynulý pohyb koncového efektora robota po tejto dráhe.
📦 Požiadavky

    Python 3.7+

    pybullet

    numpy

Inštalácia požiadaviek

pip install pybullet numpy

▶️ Spustenie

Spustite skript priamo v termináli:

python main.py

Poznámka: Simulácia sa otvorí v GUI PyBullet, kde môžete pozorovať robota a jeho pohyb po trajektórii.
🧠 Čo skript robí?

    Inicializuje prostredie a pripojí sa k PyBullet GUI.

    Načíta rovinu (plane.urdf) a model robota Franka Panda (panda.urdf).

    Vizualizuje kruh v 3D priestore pomocou modrých bodov a červených čiar.

    Vypočíta inverznú kinematiku pre každý bod na kruhu.

    Pohybuje robotom tak, aby koncový efektor prešiel cez všetky body na kruhu.

🔍 Prehľad funkcií
robot_model()

Načíta podlahu a robota do simulácie. Všetky kĺby robota sa resetujú do východzej pozície.
move_joint(robot_id, joint_index, target_angle)

Pohybuje zvoleným kĺbom do požadovaného uhla.
inverse_kinematics(target_position)

Vypočíta inverznú kinematiku pre cieľovú pozíciu efektora.
visualize_circle_and_points(center, radius, num_points)

Vizualizuje kruh pomocou sfér a čiar v simulácii a vracia zoznam bodov na kruhu.
move_along_circle(points, speed)

Pohybuje robotom cez všetky body v zozname, čím vytvára kruhový pohyb koncového efektora.
🧪 Príklad

center = [0.75, 0, 0.75]
radius = 0.2
num_points = 11

points = env.visualize_circle_and_points(center, radius, num_points)
env.move_along_circle(points, speed=0.05)

❌ Ukončenie simulácie

Na konci simulácie sa volá pb.disconnect(), čo korektne ukončí spojenie so simulátorom.
