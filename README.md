# Tilt-rotor_Simulator

This repository is outdated. The newest version of this software is in https://github.com/Guiraffo/ProVANT-Simulator

Este simulador está sendo desenvolvido dentro das atividades do projeto de pesquisa Provant UFMG/UFSC. Para tal utiliza-se o ambiente de simulação Gazebo sobre a plataforma ROS(Robotic Operating System). 

O código nesse repositório consistitui atualmente do desenvolvimento de bibliotecas dinâmics, descrição do modelo de simulação e arquivos CAD.

Pacotes ROS
- aerodinamica -> constitui no código fonte para simulação de forças aerodinâmicas (Empuxo e Drag)
- hector_gazebo_plugins -> constitui num conjunto de código fonte encontrados em (https://github.com/tu-darmstadt-ros-pkg/hector_gazebo acesso dia 24/12/2015) que simulam sensores utilizados no Projeto Tilt-rotor
- hector_sensors_description -> consitui na descrição do modelo CAD do sensor ultrassom utilizado na simulação
- servo_motor-> constitui no código fonte para simulação dos servos-motores
- tilt -> é o pacote que possue diversos arquivos para a descrição do modelo mecânico do Tiltrotor obtido do SolidWorks (http://wiki.ros.org/sw_urdf_exporter acesso dia 24/12/2015)
- sw_urdf_exporter -> funcionalidade adicional do SOlidwoks para exportar modelos CADs para modelos URDF


