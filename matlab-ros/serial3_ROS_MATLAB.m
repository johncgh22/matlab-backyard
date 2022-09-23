%% serial3_ROS_MATLAB.m  
% Script Ejemplo para conexión entre MATLAB y ROS

% Este script es un ejemplo para intercambiar datos entre una instancia
% de MATLAB con ROS usando un Robot Serial de 3 GDL.
% El paquete relacionado al robot lo puedes encontrar en este link:
% https://github.com/johncgh22/serial3_robot.git

%% Chequeo de los topicos de ROS

% Con esta función, checamos los topicos existentes.
% En este punto, el paquete del robot ya debe de estar ejecutandose dentro
% de ROS para poder ver los topicos existentes.

rostopic list;

%% Subscrición a los tópicos

% Aquí vamos a subscribirnos a los tópicos referentes al controloador
% definido en el paquete del robot serial. Esto nos servirá para poder
% controlar la posición de sus actuadores desde MATLAB.

e1Sub = rossubscriber('/serial3/joint_e1_position_controller/command');
e2Sub = rossubscriber('/serial3/joint_e2_position_controller/command');
e3Sub = rossubscriber('/serial3/joint_e3_position_controller/command');

%% Publicación a los tópicos seleccionados

% Aqui vamos a generar la publicación de los comandos de control para los
% actuadores del robot.
% El tipo de comando es importante para enviar el tipo de dato correcto
% Para un controlador del tipo position_controller, usaremos comandos de
% mensajes estandar del tipo Float64
% Mas info en este enlace:
% http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Float64.html

e1Pub = rospublisher('/serial3/joint_e1_position_controller/command','std_msgs/Float64');
e2Pub = rospublisher('/serial3/joint_e2_position_controller/command','std_msgs/Float64');
e3Pub = rospublisher('/serial3/joint_e3_position_controller/command','std_msgs/Float64');

%% Creación de los mensajes

% Aquí vamos a crear los mensajes en si, los cuales necesitan que el
% publicador definido anteriormente.

msg1 = rosmessage(e1Pub);
msg2 = rosmessage(e2Pub);
msg3 = rosmessage(e3Pub);

% Aqui insertamos el valor correspondiente al tipo definido por el commando
% del publicador para que sea el valor mandado al robot.

msg1.Data = 0.0;
msg2.Data = 0.5;
msg3.Data = 0.5;

%% Envío de los Mensajes

% Aquí hacemos el envío de los mensajes hacia ROS mediante el publicador
% generado anteriormente.
% Al ejecutar esta sección, se mandan los valores al comando de control del
% controlador para mover los actuadores hacia el valor defindo.

send(e1Pub,msg1)
send(e2Pub,msg1)
send(e3Pub,msg1)
