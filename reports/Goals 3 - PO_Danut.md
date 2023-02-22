# Objectifs du 13/01/2023

PO: Danut Pop

## Fonctionnalités attendues
- Identifier de manière unique les balles en leur associant des scores.
- Avancer la conception du robot.
- En se plaçant dans une terrain, être capable de calculer un chemin pour ramener les balles.

## Tâches à réaliser

### Conception:
- Etre capable d'envoyer des commandes au robot pour le faire avancer.
- Ajouter la pelle pour ramasser les balles.
- Ajouter les portiques pour ramasser les balles.
- Donner un nom au robot: Djokovic
- Garde au sol: 2.5 cm

### Caméra:
- Identifier les balles avec un ID qui ne change pas au courant du temps: object tracking.
- Association d'un temps écoulé à chaque balle depuis le début le spawn.

### Stratégie
- Avoir une première version fonctionnelle d'une fonction de coût qui servira à déterminer la stratégie de ramassage du robot.
- Se placer d'abord que dans le terrain gauche/droit pour ramasser les balles puis changer de terrain.


## Challenges techniques
- Comment faire du object tracking.
- Maitrise de l'urdf.
- Définir la structure du code (Est-ce que c'est la classe A qui a une atribut de classe B ou l'inverse? etc.)


