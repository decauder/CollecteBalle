# Tennis Ball Collector

## Lancer la simulation

### Dépendences


### Démarrer la simulation

###### A compléter avec la/les commande(s) à lancer.
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

### Package `tennis_court`

Le dossier `ws/src/tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.

### Package `top_camera`

Le dossier `ws/src/top_camera` est un package ROS contenant le processing des données de la `zenith_camera` - la caméra placée au dessus du terrain de tennis.
Ce package provide des positions de balle et de robot.

### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.

_________

### Installs
for `top_camera` pkg:
```sh
    sudo apt install ros-foxy-vision-opencv ros-foxy-cv-bridge
    rosdep install -i --from-path src --rosdistro foxy -y -r
```

<!-- sudo apt install ros-humble-ament-pycodestyle -->

