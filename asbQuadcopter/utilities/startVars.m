%% startVars.m - Initialize variables
% This script initializes variables and buses required for the model to
% work.

% Copyright 2013-2024 The MathWorks, Inc.

% Register variables in the workspace before the project is loaded
initVars = who;

% Variants Conditions
VSS_COMMAND = 3;       % 0: Signal Editor, 1: Joystick, 2: Pre-saved data, 3: Pre-saved data in a Spreadsheet
VSS_SENSORS = 1;       % 0: Feedthrough, 1: Dynamics
VSS_ENVIRONMENT = 0;   % 0: Constant, 1: Variable
VSS_VISUALIZATION = 3; % 0: Scopes, 1: Send values to workspace, 2: FlightGear, 3: AirportScene. 4: AppleHillScene.
VSS_VEHICLE = 1;       % 0: Linear Airframe, 1: Nonlinear Airframe.

% Bus definitions
asbBusDefinitionCommand; 
asbBusDefinitionSensors;
asbBusDefinitionEnvironment;
asbBusDefinitionStates;

% Enum definitions
asbEnumDefinition;

% Sampling rate
Ts= 0.005;   % Flight Control System sample rate
VTs = 40*Ts; % Image processing sampling rate

% Simulation time
TFinal = 30;

% Geometric properties
thrustArm = 0.10795;

% Initial conditions
% Initial position of quadcopter vary based on the visualization option and scene
init.date = [2017 1 1 0 0 0];
init.posLLA = [42.299886 -71.350447 71.3232];
init.posNED = [57 95 -0.046];
init.vb = [0 0 0];
init.euler = [0 0 0];
init.angRates = [0 0 0];
init.posAirport = [3900, -1, -0.2]; % initial position of quadcopter in Airport scene
init.posAirportActor = [3900, 1, 0]; % initial position of the Simulation 3D Actor (red box) in Airport scene
init.posAppleHill = [-50 70 -0.2]; % initial position of quadcopter in Apple Hill scene
init.posAppleHillActor = [-50 72 0.197]; % initial position of the Simulation 3D Actor (red box) in Apple Hill scene

% Initialize States:
States = Simulink.Bus.createMATLABStruct('StatesBus');
States.V_body = init.vb';
States.Omega_body = init.angRates';
States.Euler = init.euler';
States.X_ned = init.posNED';
States.LLA = init.posLLA;
States.DCM_be = angle2dcm(init.euler(3),init.euler(2),init.euler(1));

% Environment
rho = 1.184;
g = 9.81;

% Variables
% Load MAT file with model for persistence
load('modelParrot');
% Obtain vehicle variables
vehicleVars;
% Obtain sensor variables
sensorsVars;
% Obtain controller variables
controllerVars;
% Obtain command variables
commandVars;
% Obtain estimator variables
estimatorVars;
% Obtain visualization variables
visualizationFlightGearVars;

% Simulation Settings
takeOffDuration = 1;
enableLanding = true;
landingAltitude = -0.6;
measurementTolerance = 0.01;

%% Custom Variables
% Add your variables here:
% myvariable = 0;

% Default zero external torque for the disturbance From Workspace block.
if ~exist('tau_ext_ts', 'var')
    tau_ext_ts = timeseries(zeros(2,3), [0; max(TFinal, 1000)]);
    tau_ext_ts.Name = 'tau_ext_ts';
end

% Register variables after the project is loaded and store the variables in
% initVars so they can be cleared later on the project shutdown.
endVars = who;
initVars = setdiff(endVars,initVars);
clear endVars;
