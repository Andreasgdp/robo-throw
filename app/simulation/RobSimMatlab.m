clear
close all

% uncomment if you need the documentation
doc loadrobot

% Loads a robot
ur = loadrobot("universalUR5",'DataFormat','row');

% Show details about the system
showdetails(ur);
disp(ur);

% Sets to configs
randConfig = ur.randomConfiguration;
tform = getTransform(ur,randConfig,'tool0','base');

% Input a config to simulate
simConfig = [0 0 0 0 0 0];
simTform = getTransform(ur,simConfig,'tool0','base');

% Calc a jacobian for a specific config
geoJacob = geometricJacobian(ur,simConfig,'tool0');
disp(geoJacob);

% Calc the inverse kinematics
ik = inverseKinematics('RigidBodyTree',ur);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = ur.homeConfiguration;
[configSoln,solnInfo] = ik('tool0',tform,weights,initialguess);

% Shows the two robot configs
show(ur, randConfig);
figure();
show(ur,configSoln);