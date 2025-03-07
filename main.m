
% Initialize ZMQ-based Remote API Client
client = RemoteAPIClient();
sim = client.require('sim');

% Enable stepping mode
sim.setStepping(true);

% Start the simulation
sim.startSimulation();

% Check for successful connection
if isempty(sim)
    disp('Failed to connect MATLAB to CoppeliaSim using ZMQ')
    return;
else
    fprintf('Connection to CoppeliaSim established using ZMQ.\n');
end
cameraHandle = sim.getObjectHandle('XYZCameraProxy');
disp(cameraHandle)
% Run simulation loop for 3 seconds
while true
    fprintf('Hi\n');
    t = sim.getSimulationTime();
    if t >= 6
        break;
    end
    fprintf('Simulation time: %.2f [s]\n', t);
    sim.step();
end

% Stop the simulation
sim.stopSimulation();
