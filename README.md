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
- move4axes || Permet de faire bouger le SCARA  à l'aide d'une liste de valeurs par moteurs en degrés pour les angles q1, q2, q4 et en milimètres pour le moteur q3
- pickandplace || Permet de faire un mouvement de pick ou de place sans gérer la pince à l'aide des valeurs finales du robot pret à poser/déposer

- Changez le MAIN selon les positions que vous souhaitez lui donner

Le programme envoie 0N de couple à la fin pour éviter des collisions