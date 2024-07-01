# StageLAAS2024
![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)

## Commands
### MATE
Pour utiliser le fichier 'MATE_command.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- move2x3axes || Permet de faire bouger 2 MATEs en même temps à l'aide de 2 listes de valeurs par moteurs en degrés

- Changez le MAIN selon les positions que vous souhaitez lui donner

Le programme envoie 0N de couple à la fin pour éviter des collisions

### SCARA
Pour utiliser le fichier 'MATE_command.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- SO/SR || Selon le SCARA utilisé, mettez SO pour le SCARA Orange et SR pour le SCARA Rouge (ligne 54)
- move4axes || Permet de faire bouger le SCARA  à l'aide d'une liste de valeurs par moteurs en degrés pour les angles q1, q2, q4 et en centimètres pour le moteur q3
- pickandplace || Permet de faire un mouvement de pick ou de place sans gérer la pince à l'aide des valeurs finales du robot pret à poser/déposer

- Changez le MAIN selon les positions que vous souhaitez lui donner

Le programme envoie 0N de couple à la fin pour éviter des collisions

## Trajectory
Les deux programmes 'create_trajectory.py' et 'read_trajectory.py' travaillent avec un fichier 'trajectory.csv' créé automatiquement et qui contient toutes les données pour faire bouger un robot SCARA à partir d'une simulation
### Create
Pour utiliser le fichier 'create_trajectory.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- moveMotor || Permet de faire bouger le SCARA  à l'aide d'une liste de valeurs par moteurs en degrés pour les angles q1, q2, q4 et en centimètres pour le moteur q3, il prend en compte aussi l'état du gripper en paramètre id 2 (0 ouvert / 1 fermé)

- Changez le MAIN selon les positions que vous souhaitez lui donner

Le programme écrit ensuite les valeurs prises depuis la simulation dans un fichier csv : 'trajectory.csv'

### Read
Pour utiliser le fichier 'MATE_command.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- SO/SR || Selon le SCARA utilisé, mettez SO pour le SCARA Orange et SR pour le SCARA Rouge (ligne 54)

Le programme prends en compte le fichier 'trajectory.csv' créé avec 'create_trajectory.py' pour calculer automatiquement les positions du SCARA en réel et le faire bouger comme il est prévu

## Pos_0torque
Pour utiliser le fichier 'pos_0torque.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- SO/SR || Selon le SCARA utilisé, mettez SO pour le SCARA Orange et SR pour le SCARA Rouge (ligne 54)
- tf = x || Change le temps où le programme va calculer les valeurs actuelles

Le programme envoie 0N de couple pour ensuite, durant le temps défini, récupérer les valeurs de position des angles su SCARA pour ensuite les afficher grâce à MatPlotLib

## Robotics Place Code
Pour utiliser le fichier 'RoboticsPlaceCode.py' :
- hz = x || Permet de changer la valeur en hertz du programme
- SO/SR || Selon le SCARA utilisé, mettez SO pour le SCARA Orange et SR pour le SCARA Rouge (ligne 54)
- nb_tours = x || Change le nombre d'itération où le programme va faire les mouvements prévu

Tout ce fait automatiquement;

On commence par lancer le programme et choisir entre :

(le programme gère 1 SCARA et 2 MATE)
### 1 learn new positions
On vient choisir le nombre de positions que l'on souhaite, on valide chaque positions une après l'autre et le programme écrit dans un fichier csv 'trajectory_rpd.py' les positions ou le programme va aller

### 2 movement 'pick and place'
On donne au programme les positions du gripper (ouvert ou fermé) pour chaque positions du fichier enregistrés avant, puis le programme va lire les positions et les reproduire sous forme de 'pick and place' pour le SCARA
