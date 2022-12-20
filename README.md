# Tareas Básicas en Robots de Servicio Doméstico
Escuela de Invierno de Robótica, 2023.


## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/EIR-2023-AtHomeBasicTasks
* $ cd EIR-2023-AtHomeBasicTasks
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source EIR-2023-AtHomeBasicTasks/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/EIR-2023-AtHomeBasicTasks/blob/main/Media/rviz.png" alt="RViz" width="639"/>

Un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/EIR-2023-AtHomeBasicTasks/blob/main/Media/gazebo.png" alt="Gazebo" width="631"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/EIR-2023-AtHomeBasicTasks/blob/main/Media/gui.png" alt="GUI" width="328"/>

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
marco.negrete@ingenieria.unam.edu<br>
