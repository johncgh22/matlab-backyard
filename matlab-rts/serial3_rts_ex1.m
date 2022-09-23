%% serial3_rts_ex1-m 
% Script Ejemplo del uso de MATLAB Robotics Toolbox para un 
% Manipulador Serial 3 GDL
clear 
clc

% Es necesario tener instalado el Robotics System Toolbox de MATLAB para el
% correcto funcionamiento de este script

%% Cargar el robot

% Cargamos el archivo que define al robot de trabajo para el RTS.
% El archivo URDF necesario para correr este ejemplo, se puede descargar
% del siguiente repositorio:
% https://github.com/johncgh22/serial3_robot.git

addpath(genpath(strcat(pwd,'\meshes\visual')));  % Utilizamos los stl que definen al robot
serial3 = importrobot('serial3_robot.urdf','MeshPath',...
    {'Cambiar por ubicación del Repositorio\serial3_robot\meshes\visual'});
axes = show(serial3);  % Mostramos al robot en una figura de MATLAB
axes.CameraPositionMode = 'auto';

% Agregamos la referencia para el efector final
efinal = robotics.RigidBody('end_effector'); % Nombre del cuerpo rígido
setFixedTransform(efinal.Joint,trvec2tform([0.18 0 0])); % Ubicacion del efector final
addBody(serial3,efinal,'e3_link'); % Define el predecesor para referencia.
show(serial3);
axis([-0.6 0.6 -0.6 0.6 0 0.6])

%% Definir configuracion inicial del Robot

% Aquí definimos la configuración incial que mostrará al cargar el robot
% utilizando el RTS. Esta se toma de la posición definida al exportar el
% URDF
homeConfig = homeConfiguration(serial3);
randomConfig = randomConfiguration(serial3); % Configuracion Aleatoria

%% Probar otra configuracion del Robot

% Aqui vamos a variar la configuración del robot variando los valores de la
% posición de cada junta.
config = homeConfiguration(serial3);

config(1).JointPosition = 0;
config(2).JointPosition = -pi/4;
config(3).JointPosition = 0;

% Mostramos nuevamente la nueva configuración
show(serial3,config);
axis([-0.6 0.6 -0.6 0.6 0 0.6])

%% Obtener Matrices de Transformación

% El RTS permite mostrae la Matriz de Transformación Homogénea de cada
% eslabon que conforma al robot.

transform1 = getTransform(serial3,config,'e1_link');
transform2 = getTransform(serial3,config,'e2_link');
transform3 = getTransform(serial3,config,'e3_link');
transform4 = getTransform(serial3,config,'end_effector');

%% Jacobiano

% Tambien es posible obtener el Jacobiano de cada eslabón. Es necesario
% definir la configuración del robot en la cual se quiera obtener esa
% matriz.
geoJacob1 = geometricJacobian(serial3,config,'e1_link');
geoJacob2 = geometricJacobian(serial3,config,'e2_link');
geoJacob3 = geometricJacobian(serial3,config,'e3_link');
geoJacob4 = geometricJacobian(serial3,config,'end_effector');

%% Centro de Masa

% Para obtener centros de masa, es necesario cambiar la definición de los
% datos de las juntas del robot.
serial3.DataFormat = 'row';
q = [config(1).JointPosition,config(2).JointPosition,config(3).JointPosition];
com = centerOfMass(serial3,q);

%% Dinamica Directa

% Primero es necesario definir el vector de gravedad para empezar a trabajar
% con la dinamica del robot
serial3.Gravity = [0 0 -9.81]; % Asignamos Gravedad (Eje Z)

wrench = [0 0 0.5 0 0 0.3]; % Asignamos vector de Fuerzas y Torques
fext = externalForce(serial3,'end_effector',wrench,q);
qddot = forwardDynamics(serial3,q,[],[],fext);

%% Torque Gravitacional
gtau = gravityTorque(serial3,q); % 

qdot = [0 0 0.2];
tau = -velocityProduct(serial3,[],qdot);

%% Matriz de Masa
H = massMatrix(serial3,q);

%% Dinámica Inversa
tau2 = inverseDynamics(serial3,q); 
