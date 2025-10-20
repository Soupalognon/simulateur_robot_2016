## Conversion SolidWorks vers urdf
### Installation
https://wiki.ros.org/sw_urdf_exporter

### Tuto
https://www.youtube.com/watch?v=H6YPkXmkdPg

Après avoir récupéré le dossier avec l'urdf et les mesh
Ranger l'urdf dans simulateur_robot_2016/resource/urdf
Ranger les mesh dans simulateur_robot_2016/resource/meshes/visual



## Conversion urdf vers proto (webots)
### Installation
https://github.com/cyberbotics/urdf2webots/tree/master

### Tuto
Déjà il y a un git qui utilise webots et qui est indispensable
https://github.com/cyberbotics/webots_ros2

J'ai beaucoup utilisé turtlebot, e-puck et husarion

La ligne que j'utilise dans mon projet est:
python3 -m urdf2webots.importer --input=Assemblage_Carcasse.urdf --output=../proto/Assemblage_Carcasse.proto



## Créer un stl simplifié
### Installation
Installer meshlab

### Tuto
Voici un tuto pour comprendre et résoudre une partie du problème
https://adamconkey.medium.com/collision-meshes-for-simulating-robots-in-gazebo-ccc647d8b87d

Ranger tous les stl simplifié créés dans le dossier 
    simulateur_robot_2016/resource/meshes/collision









## Spécificité pour ce projet

### URDF
Après avoir converti le Solidworks en urdf il faut maintenant modifier l'urdf
Voici la liste des changements:
- Modifier les chemins des stl visuel, mettre 
<visual>
    <geometry>
        <mesh filename="package://simulateur_robot_2016/resource/meshes/visual/[mesh_name].stl" />
    </geometry>
</visual>

- Modifier les chemins des stl collision, mettre
<collision>
    <geometry>
        <mesh filename="package://simulateur_robot_2016/resource/meshes/collision/[mesh_name].stl" />
    </geometry>
</collision>

- Créer un stl simplifié pour chaque objet qui ne touche pas le sol
    - référer vous à la section "créer un stl simplifié"

- Modifier toutes les géométries des objets qui sont en contact avec le sol par des géométries simple. Exemple:
    - tous les caster (roulements à bille)
        - Dans le link/visual/geometry et link/collision/geometry la géométrie de l'objet doit être une sphère simple
<link name="caster_link">
    <visual>
        <geometry>
        <sphere radius="0.005"/>
        </geometry>
    </visual>
    <collision>
        <geometry>
        <sphere radius="0.005"/>
        </geometry>
    </collision>
</link>

    - les roues sont des cylindres, modifier comme pour les caster
<link name="wheel_link">
    <visual>
        <origin
            xyz="0 0 0"
            rpy="1.57 0 0" />
        <geometry>
        <cylinder length="0.030" radius="0.0325"/>
        </geometry>
    </visual>
    <collision>
        <origin
            xyz="0 0 0"
            rpy="1.57 0 0" />
        <geometry>
        <cylinder length="0.030" radius="0.0325"/>
        </geometry>
    </collision>
</link>

- Si vous voulez un imu --> Mettre à la fin du fichier 
<gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <topicName>imu</topicName>
        <gaussianNoise>0.0</gaussianNoise>
    </plugin>
</gazebo>

### PROTO
L'imu se créer tout seul mais pas les distance mètre...
Il faut donc les rajouter à la main une fois le fichier généré:
- Au début du fichier, rajouter l'import de l'objet
EXTERNPROTO "distance_sensor.proto"

ça ressemble en gros à ça à la fin:
#Extracted ...
EXTERNPROTO "distance_sensor.proto"
PROTO Assemblage_Carcasse
...

- Dans chaque Solid nommé "distance_sensorX_link" (il y aura un numéro à la place du X), ajouter au dessus du Shape l'objet:
    DEF EPUCK_PS0 distance_sensor {
    translation 0 0 0
    rotation 0 0 0 0
    name "range_meter_1"
    numberOfRays 1
    }
ça ressemblera à ça:
Solid {
    ...
    children [
        DEF EPUCK_PS0 distance_sensor {
        translation 0 0 0
        rotation 0 0 0 0
        name "range_meter_1"
        numberOfRays 1
        }
        Shape ...
    ]
    name ...
}

