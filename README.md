
# Ingescape ROS2 Interface

Ce projet vise à établir une interface entre le framework **ROS 2**, largement utilisé en robotique, et le logiciel **Ingescape**. L'objectif est de combiner la puissance et la flexibilité de ROS 2 avec la simplicité d'utilisation offerte par Ingescape, en particulier pour les utilisateurs ne souhaitant pas installer ROS sur leur machine.

Le cadre du projet est celui de la navigation d'un robot mobile. Le projet inclut un robot simulé évoluant dans un environnement qu'il a préalablement cartographié. Grâce à l'interface visuelle **RViz** (le visualisateur de ROS), il est possible de définir un point d'intérêt sur la carte que le robot atteindra.

Avec l'interface développée, il devient désormais possible de :

- Visualiser le retour vidéo en direct depuis le robot dans le tableau blanc d'Ingescape.
- Suivre en temps réel la position du robot sur la carte.
- Contrôler le robot avec un joystick.
- Commander le robot pour qu'il se rende à des positions préenregistrées.

## Intérêt du projet

Ce projet simplifie la prise en main des robots fonctionnant sous ROS. Il permet à des utilisateurs, même sans installation préalable de ROS, de bénéficier des capacités avancées de ce framework. En combinant ROS avec Ingescape, nous proposons une solution intuitive et accessible pour des applications robotiques, tout en conservant les fonctionnalités avancées et modulaires de ROS.

## Méthodes d'utilisation

Nous proposons deux méthodes pour démarrer avec ce projet :

1. **Utilisation d'une image Docker préconfigurée** : Cette méthode permet de commencer rapidement, sans configuration complexe.
2. **Installation classique de ROS Humble** : Une méthode plus traditionnelle pour les utilisateurs souhaitant une configuration ROS native sur leur machine.

## Méthode 1 : Lancer la simulation ROS avec Docker

### Étape 1 : Cloner le projet

Commencez par cloner le dépôt Git contenant le projet et accéder au répertoire :  

```bash
git clone https://github.com/Axelado/projet_ros_ingescape.git
cd projet_ros_ingescape
```

### Étape 2 : Construire l'image Docker

Dans le répertoire du projet, construisez l'image Docker :  

```bash
docker build -t perso_humble .
```

> Cette étape peut prendre un certain temps, en fonction de votre connexion Internet. Une fois terminée, l'image sera prête à être utilisée.

### Étape 3 : Préparer l'accès graphique pour Docker

Pour permettre au groupe Docker d'accéder à l'interface graphique, exécutez la commande suivante :  

```bash
xhost +local:docker
```

### Étape 4 : Démarrer un conteneur Docker

Lancez un conteneur avec l'image Docker précédemment créée :  

```bash
docker run -it --name ros_container --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY perso_humble
```

> Cette commande :

> - Démarre un conteneur nommé `ros_container` à partir de l'image `perso_humble`.
> - Configure l’accès réseau (via `--network=host`) et l’affichage graphique (via `--env=DISPLAY`).

Vous devriez maintenant avoir un terminal ouvert dans le conteneur. **Ne le fermez pas**, car vous l’utiliserez pour les prochaines étapes.  

Si le terminal est fermé par erreur, vous pouvez redémarrer le conteneur avec :  

```bash
docker start -i ros_container
```

Pour ouvrir un nouveau terminal dans le conteneur, utilisez :  

```bash
docker exec -it ros_container /bin/bash
```

---

## Étape 5 : Configurer et connecter Ingescape

Sur l'ordinateur hôte ou un autre ordinateur du réseau : 

1. Lancez **Ingescape Circle**.  
2. Chargez le système `ros_interface.igssystem` et connectez-le.  
3. Lancez et connectez le **whiteboard**.

> Vous devriez voir six agents (y compris le whiteboard) apparaître dans Circle.

Les agents suivants seront démarrés automatiquement par des nœuds ROS 2 :  

- **image_on_whiteboard**  
- **joystick_ros**  
- **room_number_ros**

Les agents **joystick** et **go_to_room** peuvent être démarrés sur n’importe quel ordinateur du réseau.

---

## Étape 6 : Démarrer la simulation ROS

Dans le terminal du conteneur Docker, exécutez la commande suivante (en remplaçant `network_device` et `port` par les valeurs appropriées pour votre réseau) :  

```bash
ros2 launch ingescape_ros2_interface ingescape_ros_interface.launch.py network_device:=wlo1 port:=5670
```

Cela active les agents **image_on_whiteboard**, **joystick_ros**, et **room_number_ros** dans Circle. Vous devriez maintenant voir :  

- Le retour vidéo du robot.  
- La carte et la position du robot sur le **whiteboard**.

---

## Étape 7 : Lancer des agents supplémentaires

### Agent `go_to_room`  

Cet agent permet de demander au robot d'aller à une position préenregistrée sur la carte.  

1. Ouvrez un terminal dans le répertoire `ingescape_client`.  
2. Lancez le script Python en modifiant les paramètres comme suit :  

   ```bash
   python3 go_to_room.py post wlo1 5670
   ```

3. Une interface s’affiche. Sélectionnez une salle dans la liste déroulante, puis cliquez sur le bouton **GO TO** pour que le robot se déplace.

---

### Agent `joystick`

Cet agent permet de contrôler le robot à l’aide d’une manette.  

1. Branchez une manette compatible à votre PC.  
2. Ouvrez un terminal dans le répertoire `ingescape_client`.  
3. Lancez le script Python en modifiant les paramètres comme suit : 

   ```bash
   python3 joystick.py joystick wlo1 5670
   ```

4. Pour déplacer le robot, maintenez la touche **R1** (ou **L1**) et utilisez le joystick gauche.

---

**Note** : Assurez-vous que les paramètres réseau (comme `wlo1` et `5670`) correspondent à votre configuration pour chaque commande. 

## Méthode 2 : Installer ROS Humble sur Ubuntu

### Prérequis

Ubuntu 22.04 est recommandé pour cette version de ROS.  

### Étape 1 : Installer ROS Humble

Suivez les instructions officielles pour installer ROS Humble :  
[Guide officiel d'installation ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  

À la fin de l'installation, ajoutez la commande suivante à votre fichier `.bashrc` pour éviter de devoir la taper manuellement à chaque nouveau terminal :  

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Étape 2 : Créer un workspace ROS 2

Créez un espace de travail ROS 2 :  

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clonez le projet dans le workspace :  

```bash
git clone https://github.com/Axelado/projet_ros_ingescape.git
```

### Étape 3 : Installer les dépendances

#### Automatique

Exécutez la commande suivante pour installer les dépendances requises :  

```bash
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

#### Manuelle

Installez les dépendances manquantes via `apt` :  

```bash
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    joystick \
    jstest-gtk \
    evtest \
    ros-humble-twist-mux \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-opencv
```

### Étape 4 : Compiler le projet

Compilez le projet à partir du répertoire `ros2_ws` :  

```bash
cd ~/ros2_ws
colcon build
```

### Étape 5 : Configurer et connecter Ingescape

Sur l'ordinateur hôte ou un autre ordinateur du réseau :  
1. Lancez **Ingescape Circle**.  
2. Chargez le système `ros_interface.igssystem` et connectez-le.  
3. Lancez et connectez le **whiteboard**.  

Vous devriez voir six agents (y compris le whiteboard) apparaître dans Circle.

Les agents suivants seront démarrés automatiquement par des nœuds ROS 2 :  
- **image_on_whiteboard**  
- **joystick_ros**  
- **room_number_ros**

Les agents **joystick** et **go_to_room** peuvent être démarrés sur n’importe quel ordinateur du réseau.

---

### Étape 6 : Démarrer la simulation ROS

Lancez la simulation en remplaçant `network_device` et `port` par les valeurs appropriées pour votre réseau :  

```bash
ros2 launch ingescape_ros2_interface ingescape_ros_interface.launch.py network_device:=wlo1 port:=5670
```

Les agents suivants seront activés dans Circle :  

- **image_on_whiteboard** : Affiche le retour vidéo du robot.  
- **joystick_ros** : Permet de contrôler le robot avec une manette.  
- **room_number_ros** : Affiche la position et les informations sur la carte.  

Le retour caméra et la carte sont maintenant visibles sur le **whiteboard**.

---

### Étape 7 : Lancer des agents supplémentaires

#### Agent `go_to_room`

Cet agent permet de demander au robot d'aller à une position préenregistrée sur la carte.  

1. Ouvrez un terminal dans le répertoire `ingescape_client`.  
2. Lancez le script Python en modifiant les paramètres comme suit :

   ```bash
   python3 go_to_room.py post wlo1 5670
   ```

3. Une interface s’affiche. Sélectionnez une salle dans la liste déroulante, puis cliquez sur le bouton **GO TO** pour que le robot se déplace.

---

#### Agent `joystick`

Cet agent permet de contrôler le robot à l’aide d’une manette.  

1. Branchez une manette compatible à votre PC.  
2. Ouvrez un terminal dans le répertoire `ingescape_client`.  
3. Lancez le script Python en modifiant les paramètres comme suit :  

   ```bash
   python3 joystick.py joystick wlo1 5670
   ```

4. Maintenez la touche **R1** (ou **L1**) et utilisez le joystick gauche pour déplacer le robot.

---

**Note** : Assurez-vous que les paramètres réseau (comme `wlo1` et `5670`) correspondent à votre configuration pour chaque commande.




## Script Validation and Verification

### Localisation des scripts de test
Les scripts de test sont situés dans le répertoire `ingescape_VandV`.

### Remarques générales
- L'agent **image_on_whiteboard** n’a pas de script de test, car :
  - Il n’a ni entrées, ni sorties, ni services.
  - Il ne fait qu’appeler des services du whiteboard.  
- Les quatre autres agents ont soit des entrées, soit des sorties, mais aucun service.

---
