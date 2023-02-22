# Debrief du 20/01/2023 - matin

PO: Jakub

## Bilan

Pourcentage de tâches réalisées: 75%

*Note: Nous avions des imprévus dans l'équipage - pas tous membres présent.*

### Ce qui a fonctionné

- Controle la mouvement du robot dans Gazebo
- Amelioration de path planning algorithme


### Ce qui n'a pas fonctionné

Nous ne sommes pas arrivés à vérifier si le robot satisfait les contraintes données et ajoute automatique des balles détectées. C'est à faire dans le prochain sprint. (Regardez Backlog dans Taiga)


### Retour d'expérience du PO

C'était difficile de préparer des tâches pour ce sprint vite pour ne bloquer personne. Cela serait mieux de préparer les tâches d'avance. Peut-être je suggesterais de passer la rôle de PO directement aprés le sprint est fini et laisse le prochain PO préparer les tâches pour le prochain sprint.


### Conseils pour le prochain PO

Préparer les tâches d'avance et bien pense à la difficulté (temps de réalisation) de chaque tâche.


## Nouvelles mesures

### Tâches à réaliser:
- Améliorer la vitesse du robot ( Ajouter un bouton "vitesse boost" ? )
- Fixer path planning algo - demi plans bug
- Détection de balles: Gérer disparassion de balles - Qu'est-ce qu'on fait quand on ne voit plus la balle ?
- Proposer un nouveu nom pour le robot
- Quelle est la valeur de *Garde au sol* du notre robot ?

### Nouvelles demandes de client:
- **Code style uniforme** (Linting) <span class="color: red">[important]</span>
- CI en utilisant **Github Actions** ... PO peut faire ça <span class="color: red">[important]</span>

- À implementer plus tard (pas nécessaire dans le sprint 5):
    - Ajouter des tests unitires
        - tip: Gtest (Google Test), PyTest

