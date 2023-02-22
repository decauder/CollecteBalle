# Tennis Ball Collector

[![collecte-balle-ros-foxy](https://github.com/FLEDJ-Inge/CollecteBalle/actions/workflows/collecte-balle-ros-foxy.yml/badge.svg)](https://github.com/FLEDJ-Inge/CollecteBalle/actions/workflows/collecte-balle-ros-foxy.yml)

## Lancer la simulation

### Dépendences


### Démarrer la simulation
Dans la racine du workspace (ws), lancer:
```bash
    colcon build
    source ./install/setup.sh
    ros2 launch tennis_court tennis_court.launch.py

```


## Groupe

### Membres

DECAUDAVEINE Ermance
HODEK Jakub
POP Danut
JERRAM Florian
DROUDUN Laurent


### Gestion de projet

Lien du projet Taiga
[https://tree.taiga.io/project/xqubics-fledj-inge/timeline](https://tree.taiga.io/project/xqubics-fledj-inge/timeline)



## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.
Le dossier ws/src contient trois packages : tennis_court, top_camera et robot_control

### Package `tennis_court`

Le dossier `ws/src/tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](ws/src/tennis_court/README.md) du package pour plus d'informations.

### Package `top_camera`

Le dossier `ws/src/top_camera` est un package ROS contenant le processing des données de la `zenith_camera` - la caméra placée au dessus du terrain de tennis.
Les codes qu'elles contient permettent donc, en s'abonnant à la caméra, de détecter le robot et les balles, et ainsi de detreminer le chemin à suivre.

#### Subscribers:
- **`zenith_camera/image_raw`**: raw camera image qui viens de package `tennis_court` (`sensor_msgs.msg.Image`)

#### Publishers:
- **`robot/target_position`**: position destinée du robot  (`geometry_msgs.msg.Pose2D`)
- **`robot/position`**: position vrai du robot (detecté par camera) (`geometry_msgs.msg.Pose2D`)

### Package 'robot_control'

Le dossier 'ws/src/robot_control' est un package ROS contenant les codes de control du robot.

#### Subscribers:
- **`robot/target_position`**: position destinée du robot  (`geometry_msgs.msg.Pose2D`)
- **`robot/position`**: position vrai du robot (detecté par camera) (`geometry_msgs.msg.Pose2D`)

#### Publishers:
- **`demo/cmd_vel`**: commande de vitesse (linéaire et rotationnel) du robot (`geometry_msgs.msg.Twist`) 
- *(dummy_target_position, qui publie vers `robot/target_position` pour tester le robot_control)*


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.

_________

### Installer
Pour le package 'top_camera':
```sh
    sudo apt install ros-foxy-vision-opencv ros-foxy-cv-bridge
    rosdep install -i --from-path src --rosdistro foxy -y -r
```

<!-- sudo apt install ros-humble-ament-pycodestyle -->


<!-- sudo apt install ros-humble-ament-pycodestyle -->

### Build
```sh
    colcon build
```

### Tester
```sh
    make test
```
*Note: You should be in the root folder, where the Makefile is.*


#### Code style
Nous utilisons [autopep8](https://pypi.org/project/autopep8/) pour vérifier et réparer erreurs de code style.

En cas tu veux vérifier le code style d'un fichier, tu peux utiliser:
```sh
    autopep8 <path-to-file> --select=E,W --max-line-length=120 --diff
```

En cas tu veux réparer le code style d'un fichier, tu peux utiliser:
```sh
    autopep8 <path-to-file> --max-line-length=120 --select=E,W 
```

Pour VSCode, le fichier `.vscode/settings.json` est configuré pour utiliser autopep8 à chaque sauvegarde.


### Run
```sh
    source ./install/setup.sh
    [insert command here]
```
