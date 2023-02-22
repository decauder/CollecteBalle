# Objectifs du 12/01/2023

PO: Laurent Droudun


## Fonctionnalités attendues

- Le robot doit pouvoir se déplacer aux différents points correspondant aux balles détectées en évitant le filet dès qu'il doit changer de terrain
- La vitesse ne doit pas excéder une certaine valeur sinon il se retournera
- Les roues ne doivent pas être détectées comme des balles
- Pouvoir suivre la trajectoire plannifiée en l'actualisant lors de la récupération des balles


## Tâches à réaliser

- Mettre en commun le travail déjà réalisé
- Finalisation de la conception sur Gazebo
- Bien documenté les codes
- Réaliser des noeuds ROS pour communiquer les données traitées grâce à la caméra et le robot pour qu'il sache où il doit aller


## Challenges techniques

- Améliorer la plannification de trajectoire (régler le problème de retour à la base lorsqu'une balle est détectée sur un terrain au départ mais qu'elle change de terrain en roulant)
- Pouvoir lancer tout ce dont on a besoin en 1 seul launch
