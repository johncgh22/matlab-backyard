%% conectarROS.m - Script Ejemplo para conectar MATLAB con ROS

% El siguiente código sirve para crear los nodos de conexión entre una
% instancia de MATLAB y ROS (ya sea en Máquina Virtual o en Físico)
% Es necesario tener instalado el ROS Toolbox para su correcta ejecución
% Este script se ha probado tanto en Melodic (Ubuntu 18.04)
% como en Noetic (Ubuntu 20.04)

% Para mayor información, checa el siguiente link:
% https://la.mathworks.com/help/ros/ug/connect-to-a-ros-network.html

%% Cerrar conexiones existentes de ROS

% Primero cerramos cualquier conexión existente entre los programas.

rosshutdown

%% Definir la IP de MATLAB

% Aquí definimos la IP de la instalación de MATLAB
% En Windows, puedes checarla con el comando ipconfig
% Si usas Ubuntu (Linux), puedes checarla con el comando ip -a

 matlabIP = 'Dirección IP de MATLAB'; % Aquí escribes la IP

%% Definir la IP de ROS (Maquina Virtual)

% Aquí definimos la IP de tu instalación de ROS
% La variable de entorno que define este valor es el ROS_MASTER_URI
% Checa aquí para mas info: 
% http://wiki.ros.org/ROS/EnvironmentVariables

 rosIP = 'Dirección IP de ROS'; % Aqui pones la IP de ROS

%% Generar la variables de conexión

% Aquí definimos las variables de entorno para crear la conexión
% El puerto de comunicación siempre debe de ser el 11311

setenv('ROS_IP',matlabIP);
setenv('ROS_MASTER_URI',['http://' rosIP ':11311'])

% Con esta variable, inicializamos el nodo de MATLAB con ROS

rosinit(rosIP);

% Para verificar que se haya creado la conexión, ejecuta un roscore
% y ejecuta un rosnode list y tienes que ver el nodo de la siguiente forma:
% /matlab_global_node_XXXXX