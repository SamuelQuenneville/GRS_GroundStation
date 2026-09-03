addpath('core');
addpath('config');
addpath('apps');

config = config();
payloadPath  = traj.generatePayloadPath(config);
[aircraftPath, takeoffPath] = traj.generateAircraftPath(config, payloadPath);
tethersForces = dyn.getTethersForces(config, payloadPath, aircraftPath);
controls = dyn.getAircraftControls(config, aircraftPath, takeoffPath, tethersForces);

% Dump to plain CSV files for cross-checking against a C++ port.
csvwrite('/tmp/oct_payload_pos.csv', payloadPath.pos);
csvwrite('/tmp/oct_payload_vel.csv', payloadPath.vel);
csvwrite('/tmp/oct_payload_acc.csv', payloadPath.acc);
csvwrite('/tmp/oct_payload_t.csv', payloadPath.t);

for k = 1:numel(aircraftPath)
    csvwrite(sprintf('/tmp/oct_ac%d_pos.csv', k), aircraftPath{k}.inertialFrame.pos);
    csvwrite(sprintf('/tmp/oct_ac%d_vel.csv', k), aircraftPath{k}.inertialFrame.vel);
    csvwrite(sprintf('/tmp/oct_ac%d_acc.csv', k), aircraftPath{k}.inertialFrame.acc);
    csvwrite(sprintf('/tmp/oct_ac%d_pos_payloadFrame.csv', k), aircraftPath{k}.payloadFrame.pos);
    csvwrite(sprintf('/tmp/oct_takeoff%d_psi.csv', k), takeoffPath{k}.psi);
    csvwrite(sprintf('/tmp/oct_takeoff%d_gamma.csv', k), takeoffPath{k}.gamma);
    csvwrite(sprintf('/tmp/oct_takeoff%d_pos.csv', k), takeoffPath{k}.inertialFrame.pos);
    csvwrite(sprintf('/tmp/oct_takeoff%d_vel.csv', k), takeoffPath{k}.inertialFrame.vel);
    csvwrite(sprintf('/tmp/oct_takeoff%d_acc.csv', k), takeoffPath{k}.inertialFrame.acc);
    csvwrite(sprintf('/tmp/oct_controls%d.csv', k), controls{k});
end

F1 = cell2mat(tethersForces.forces(:,1));
F2 = cell2mat(tethersForces.forces(:,2));
csvwrite('/tmp/oct_tetherF1.csv', F1);
csvwrite('/tmp/oct_tetherF2.csv', F2);

disp('OK');
disp(size(payloadPath.pos));
disp(size(controls{1}));
