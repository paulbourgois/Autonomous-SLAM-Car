# car_ros2_ws

Ce workspace ROS2 est dédié à la conception d'un véhicule autonome utilisant la localisation et la cartographie simultanées (SLAM).

Il est en développement.

## Objectif

Développer et intégrer des algorithmes de SLAM pour permettre au véhicule de se localiser et de cartographier son environnement de manière autonome.

## Structure du projet

- `src/` : Contient les packages ROS2 pour le contrôle du véhicule, la perception, et le SLAM.

## Installation

```bash
cd ~/car_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Utilisation

Lancer le SLAM autonome :

```bash
ros2 launch <nom_du_package> <fichier_launch>.py
```

## Documentation

Consultez la documentation de chaque package dans le dossier `src/`.

## Contribuer

Les contributions sont les bienvenues ! Veuillez ouvrir une issue ou une pull request.

## Licence

Ce projet est sous licence MIT.