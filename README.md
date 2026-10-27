# Simulation de bras robotique : empilement de cubes

Simulation ROS 2 (Jazzy) + Gazebo d'un bras **Franka Panda** posé devant une table sur laquelle se trouvent trois cubes de couleur (rouge, vert, bleu). Une caméra déportée observe la table : le bras repère les cubes par leur couleur, les attrape avec sa pince et les empile dans l'ordre choisi.

## Ce que fait le bras

1. **Détection** : le nœud `cube_detector` analyse l'image couleur et l'image de profondeur de la caméra déportée, isole chaque cube par sa teinte (HSV) et publie sa position dans le repère monde (`/cube_detector/red_cube`, `/cube_detector/green_cube`, `/cube_detector/blue_cube`).
2. **Planification** : le nœud `main_controller` est une machine à états qui enchaîne les actions pour chaque cube, dans l'ordre d'empilement demandé :
   - approche au-dessus du cube, descente, fermeture de la pince ;
   - remontée, déplacement jusqu'à la zone de dépôt (x = 0.8, y = 0), descente à la hauteur de la pile, ouverture de la pince ;
   - passage au cube suivant, jusqu'à ce que la pile soit complète.
3. **Gestion des obstacles** : si le cube à prendre a un autre cube posé dessus, le bras dépile d'abord ce cube gênant et le pose sur une zone libre de la table (à portée du bras, hors de la zone de dépôt et à l'écart des autres cubes), puis reprend le cube voulu.
4. **Mouvement** : `arm_controller` reçoit les positions cibles via l'action `/arm_controller/move_to_position`, calcule la cinématique inverse et génère une trajectoire articulaire à vitesse limitée.
5. **Préhension** : `gripper_controller` ouvre et ferme les doigts (`/gripper/open`, `/gripper/close`) et attache le cube à la pince lorsqu'un contact est détecté.

## Lancement

Compiler puis sourcer le workspace :

```bash
./build.sh
source install/setup.bash
```

Lancer la simulation (Gazebo, RViz, bridge, `arm_controller`, `gripper_controller`, `cube_detector`) :

```bash
ros2 launch robotic_arm_sim sim.launch.py
```

Dans un second terminal, lancer le contrôleur principal qui exécute l'empilement :

```bash
ros2 run robotic_arm_sim main_controller
```

## Choisir l'ordre d'empilement

L'ordre se règle avec le paramètre `cube_order`. Le premier cube de la liste est posé en bas de la pile, le dernier en haut :

```bash
ros2 run robotic_arm_sim main_controller --ros-args -p cube_order:="[blue, red, green]"
```

La liste doit contenir exactement les trois couleurs `red`, `green` et `blue`, chacune une seule fois. Sans ce paramètre, l'ordre par défaut est `[green, blue, red]`.

## Réinitialiser la simulation

Le service de reset remet la scène dans un état de départ sans relancer Gazebo :

```bash
ros2 service call /arm_controller/reset std_srvs/srv/Trigger
```

Il :

- réinitialise la pince (ouverture et détachement de tout cube tenu) ;
- ramène le bras dans sa position initiale ;
- repose les cubes à des positions aléatoires sur la table, à portée du bras et hors de la zone de dépôt. Certains cubes peuvent être posés les uns sur les autres, ce qui permet de tester la gestion des obstacles.

Les paramètres du tirage aléatoire (zone, portée, probabilité d'empilement, etc.) se trouvent dans [`src/robotic_arm_sim/config/arm_params.yaml`](src/robotic_arm_sim/config/arm_params.yaml).

Après un reset, relancer `main_controller` pour effectuer un nouvel empilement.
