# wm_lifting_column_hardware_interface

Package pour le hardware interface du lifting_column de sara.
Ce package agit comme un plugin pour sara_control. Par conséquent, aucun launch direct n'est néssésaire.
## Instalation
```sh
sudo apt-get install ros-kinetic-ros-control
cd <your workspace>/src
git clone https://github.com/WalkingMachine/wm_lifting-column_hardware_interface.git
cd <your workspace>
catkin_make
``` 
## Topics
* Publisher
    * /column/cmd (std_msgs/Int32)
        * command can range from -255 (down) to 255 (up).
    * /column/set_position (std_msgs/Int32)
        * Used to calibrate the column
* Subscriber
    * /column/position (std_msgs/Int32)
        * relative position from the hall sensor
        
## Exemple de configuration
```yaml
base_actuator_joint:
 type: wm_lifting_column_hardware_interface/WMLiftingColumnHardwareInterface
 joints:
  - base_actuator_joint
 cmd_max: 255
 speed_max_up: 0.0279503106
 speed_min_up: 0.02
 speed_max_down: 0.02958988
 speed_min_down: 0.02
 max_height: 0.69
 resolution: 2387
 cmd_topic: "column/cmd"
 position_topic: "column/position"
 set_position_topic: "column/set_position"
```
