# wm_lifting_column_hardware_interface

Package pour le hardware interface des moteurs lifting_column de sara.
Ce package agit comme un plugin pour sara_control. Par conséquent, aucun launch direct n'est néssésaire.
### Instalation
```sh
sudo apt-get install ros-kinetic-ros-control
cd <your workspace>/src
git clone https://github.com/WalkingMachine/sara_launch.git
git clone https://github.com/WalkingMachine/sara_control.git
git clone https://github.com/WalkingMachine/sara_description.git
git clone https://github.com/WalkingMachine/lifting_column_140_description.git
cd <your workspace>
catkin_make
```
### Pour les tests manuels
```
roslaunch sara_description sara_description
roslaunch sara_control sara_control
roslaunch sara_launch sara_controllers
```  
