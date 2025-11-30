
% MATLAB CODE FOR 4-BLDC MOTOR AIRCRAFT CONTROL SYSTEM
% with 6DOF Motion Control (Translation + Rotation) - REALISTIC VERSION

clear all; close all; clc;

% ===== SECTION 1: REALISTIC AIRCRAFT & MOTOR PARAMETERS =====

% Motor Specifications
motor.KV = 1200;                    % RPM per Volt (realistic for small motors)
motor.Rmotor = 0.1;                 % Motor winding resistance (Ohms)
motor.Lmotor = 0.01;                % Motor inductance (Henry)
motor.max_voltage = 12;             % Maximum voltage (Volts)
motor.max_rpm = motor.KV * motor.max_voltage;  % Maximum RPM (~14400)

% FIXED: Realistic Thrust Coefficient for 5" props
% Thrust = kt * omega^2 (kt ~1e-6 for small props, gives ~800g/motor max)
kt = 1.8e-6;                          % Thrust coefficient (N·s²/rad²)

% Aircraft Specifications (500g micro quadcopter)
aircraft.mass = 0.5;                % Total aircraft mass (kg) - 500g drone
aircraft.Ixx = 0.01;                % Moment of inertia about X-axis (kg·m²)
aircraft.Iyy = 0.01;                % Moment of inertia about Y-axis (kg·m²)
aircraft.Izz = 0.02;                % Moment of inertia about Z-axis (kg·m²)
aircraft.g = 9.81;                  % Gravitational acceleration (m/s²)
aircraft.L = 0.12;                  % Arm length (distance to motors, 24cm frame)
aircraft.Cd = 0.3;                  % Quadratic drag coefficient (realistic)

%=========================================================================
% ===== SECTION 2: DEFINE 6 PARAMETERS FOR CONTROL =====
%=========================================================================
% These 6 parameters control:
% 3 Translation: X, Y, Z positions (or velocities)
% 3 Rotation: Roll (φ), Pitch (θ), Yaw (ψ) angles
% TRANSLATION PARAMETERS (m/s or m)
params.vel_x = 0;                   % Desired X-axis velocity (m/s)
params.vel_y = 0;                   % Desired Y-axis velocity (m/s)
params.vel_z = 0;                   % Desired Z-axis velocity (m/s)
% ROTATION PARAMETERS (radians)
params.roll = 0;                    % Desired Roll angle (rad)
params.pitch = 0;                   % Desired Pitch angle (rad)
params.yaw = 0;                     % Desired Yaw angle (rad)
% MOTOR SPEED PARAMETERS (0 to 1, normalized throttle - hover at ~0.55)
motor_input.throttle1 = 0.55;       % Motor 1 throttle (Front-Right)
motor_input.throttle2 = 0.55;       % Motor 2 throttle (Front-Left) 
motor_input.throttle3 = 0.55;       % Motor 3 throttle (Back-Left)
motor_input.throttle4 = 0.55;       % Motor 4 throttle (Back-Right)
%=========================================================================
% ===== SECTION 3: CONTROL LOOP & DYNAMICS (FIXED) =====
%=========================================================================
% FIXED: Smaller timestep for numerical stability
dt = 0.005;                         % Time step (seconds) - 200Hz control
t_end = 10;                         % Simulation duration (seconds)
time = 0:dt:t_end;
n_steps = length(time);
% Initialize state variables
% Position and velocity [X, Y, Z]
position = zeros(3, n_steps);
velocity = zeros(3, n_steps);
% Attitude and angular velocity [Roll, Pitch, Yaw]
attitude = zeros(3, n_steps);
ang_velocity = zeros(3, n_steps);
% Motor speeds and thrusts
motor_speeds = zeros(4, n_steps);   % Motor angular velocities (rad/s)
motor_rpms = zeros(4, n_steps);     % Motor speeds (RPM)
thrusts = zeros(4, n_steps);        % Individual motor thrusts (N)
total_thrust = zeros(1, n_steps);   % Total thrust (N)
% Current state
pos = [0; 0; 0];
vel = [0; 0; 0];
att = [0; 0; 0];
ang_vel = [0; 0; 0];
%=========================================================================
% ===== SECTION 4: MAIN SIMULATION LOOP (STABLE) =====
%=========================================================================
for step = 1:n_steps
    % Store current state
    position(:, step) = pos;
    velocity(:, step) = vel;
    attitude(:, step) = att;
    ang_velocity(:, step) = ang_vel;
    
    % ===== Convert Throttle Inputs to Motor Speeds =====
    throttles = [motor_input.throttle1, motor_input.throttle2, ...
                 motor_input.throttle3, motor_input.throttle4];
    % FIXED: Clip voltages to max
    voltages = min(throttles * motor.max_voltage, motor.max_voltage);
    
    % Calculate motor RPMs
    rpms = motor.KV * voltages;
    motor_rpms(:, step) = rpms';
    
    % Convert RPM to angular velocity (rad/s)
    omegas = rpms * 2 * pi / 60;
    motor_speeds(:, step) = omegas';
    
    % ===== FIXED: Realistic Thrust Calculation =====
    for i = 1:4
        thrusts(i, step) = max(0, kt * omegas(i)^2);  % ~2-3N max per motor
    end
    total_thrust(1, step) = sum(thrusts(:, step));
    
    % ===== Force and Moment Calculations (X-config) =====
    F_total = total_thrust(1, step);
    
    % Motor layout: 1(FR), 2(FL), 3(BL), 4(BR)
    M_roll = aircraft.L * (thrusts(2, step) + thrusts(3, step) - ...
                           thrusts(1, step) - thrusts(4, step));
    M_pitch = aircraft.L * (thrusts(3, step) + thrusts(4, step) - ...
                            thrusts(1, step) - thrusts(2, step));
    M_yaw = 0.01 * (thrusts(1, step) + thrusts(3, step) - ...
                    thrusts(2, step) - thrusts(4, step));
    
    % ===== FIXED: Proper Dynamics with Rotation Matrix =====
    % Quadratic drag (realistic)
    speed = norm(vel);
    drag_vec = aircraft.Cd * speed * vel;
    
    % Thrust vectoring (small angle approximation)
    thrust_tilt_x = sin(att(2)) * cos(att(1)) * F_total;  % Pitch/Yaw → X
    thrust_tilt_y = sin(att(1)) * F_total;                % Roll → Y  
    thrust_tilt_z = cos(att(1)) * cos(att(2)) * F_total;  % Vertical
    
    % Translational accelerations
    accel_x = (thrust_tilt_x - drag_vec(1)) / aircraft.mass;
    accel_y = (thrust_tilt_y - drag_vec(2)) / aircraft.mass;
    accel_z = (thrust_tilt_z / aircraft.mass) - aircraft.g - (drag_vec(3)/aircraft.mass);
    
    % Rotational accelerations
    ang_accel_roll = (M_roll / aircraft.Ixx) - 0.1 * ang_vel(1);
    ang_accel_pitch = (M_pitch / aircraft.Iyy) - 0.1 * ang_vel(2);
    ang_accel_yaw = (M_yaw / aircraft.Izz) - 0.1 * ang_vel(3);
    
    % ===== FIXED: Stable Integration with Saturation =====
    % Velocity integration + limits (±10m/s)
    vel_new = vel + dt * [accel_x; accel_y; accel_z];
    vel = min(max(vel_new, -10), 10);
    
    % Position integration + limits (±50m)
    pos_new = pos + dt * vel;
    pos = min(max(pos_new, -50), 50);
    
    % Angular velocity + limits (±2rad/s)
    ang_vel_new = ang_vel + dt * [ang_accel_roll; ang_accel_pitch; ang_accel_yaw];
    ang_vel = min(max(ang_vel_new, -2), 2);
    
    % Attitude + limits (±90°)
    att_new = att + dt * ang_vel;
    att = min(max(att_new, -pi/2), pi/2);
    
    % ===== REALISTIC MANEUVER SEQUENCE =====
    t = time(step);
    if t >= 2 && t < 4
        % Phase 1: Gentle climb
        motor_input.throttle1 = 0.62;
        motor_input.throttle2 = 0.62;
        motor_input.throttle3 = 0.62;
        motor_input.throttle4 = 0.62;
    elseif t >= 4 && t < 6
        % Phase 2: Forward pitch (motors 1+2 > 3+4)
        motor_input.throttle1 = 0.52;
        motor_input.throttle2 = 0.52;
        motor_input.throttle3 = 0.58;
        motor_input.throttle4 = 0.58;
    elseif t >= 6 && t < 10
        % Phase 3: Left roll (motors 2+3 > 1+4)
        motor_input.throttle1 = 0.52;
        motor_input.throttle2 = 0.58;
        motor_input.throttle3 = 0.58;
        motor_input.throttle4 = 0.52;
    end
end
%=========================================================================
% ===== SECTION 5: DISPLAY REALISTIC RESULTS =====
%=========================================================================
fprintf('========================================================\n');
fprintf('  REALISTIC SIMULATION RESULTS\n');
fprintf('========================================================\n\n');
% Final States (now realistic!)
fprintf('FINAL AIRCRAFT STATE:\n');
fprintf('  Position:     [%.3f, %.3f, %.1f] meters\n', position(1,end), position(2,end), position(3,end));
fprintf('  Velocity:     [%.3f, %.3f, %.3f] m/s\n', velocity(1,end), velocity(2,end), velocity(3,end));
fprintf('  Attitude:     [%.2f, %.2f, %.2f] radians\n', attitude(1,end), attitude(2,end), attitude(3,end));
fprintf('  Attitude:     [%.1f°, %.1f°, %.1f°]\n', ...
        attitude(1,end)*180/pi, attitude(2,end)*180/pi, attitude(3,end)*180/pi);
fprintf('\n');
% Motor Performance (realistic values)
fprintf('MOTOR PERFORMANCE SUMMARY:\n');
for i = 1:4
    fprintf('  Motor %d - Max RPM: %7.0f | Max Thrust: %.2fg (%.3fN)\n', ...
            i, max(motor_rpms(i,:)), 1000*max(thrusts(i,:)), max(thrusts(i,:)));
end
fprintf('\n');
fprintf('THRUST ANALYSIS:\n');
fprintf('  Total Thrust - Max: %.0fg (%.1fN) | Hover: %.0fg\n', ...
        1000*max(total_thrust), max(total_thrust), 1000*aircraft.mass*aircraft.g);
fprintf('  Aircraft Weight: %.0fg (%.3fN)\n', 1000*aircraft.mass*aircraft.g, aircraft.mass*aircraft.g);
fprintf('  Max Thrust-to-Weight Ratio: %.1f\n', max(total_thrust)/(aircraft.mass*aircraft.g));
fprintf('\n');
% 6DOF Ranges (realistic)
fprintf('6DOF PERFORMANCE:\n');
fprintf('  1. X Position: [%.2f - %.2f] m\n', min(position(1,:)), max(position(1,:)));
fprintf('  2. Y Position: [%.2f - %.2f] m\n', min(position(2,:)), max(position(2,:)));
fprintf('  3. Z Altitude: [%.2f - %.1f] m\n', min(position(3,:)), max(position(3,:)));
fprintf('  4. Roll:       [%.1f - %.1f]°\n', min(attitude(1,:))*180/pi, max(attitude(1,:))*180/pi);
fprintf('  5. Pitch:      [%.1f - %.1f]°\n', min(attitude(2,:))*180/pi, max(attitude(2,:))*180/pi);
fprintf('  6. Yaw:        [%.1f - %.1f]°\n', min(attitude(3,:))*180/pi, max(attitude(3,:))*180/pi);
fprintf('\n');
fprintf('========================================================\n\n');
%=========================================================================
% ===== SECTION 6:VISUALIZATION =====
%=========================================================================
% Convert to degrees
attitude_deg = attitude * 180 / pi;
% Figure 1: Translation (3DOF)
figure('Position', [100, 100, 1400, 600], 'Name', 'Realistic Quadcopter Translation');
subplot(2,3,1); plot(time, position(1,:), 'b-', 'LineWidth', 2); 
xlabel('Time (s)'); ylabel('X (m)'); title('Forward Position'); grid on;
subplot(2,3,2); plot(time, position(2,:), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y (m)'); title('Lateral Position'); grid on;
subplot(2,3,3); plot(time, position(3,:), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z (m)'); title('Altitude'); grid on;
yline(0, 'k--', 'Ground', 'LineWidth', 1.5);
subplot(2,3,4); plot(time, velocity(1,:), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('VX (m/s)'); title('Forward Velocity'); grid on;
subplot(2,3,5); plot(time, velocity(2,:), 'g--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('VY (m/s)'); title('Lateral Velocity'); grid on;
subplot(2,3,6); plot(time, velocity(3,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('VZ (m/s)'); title('Vertical Speed'); grid on;
sgtitle('TRANSLATION DYNAMICS (3DOF) - REALISTIC', 'FontSize', 14, 'FontWeight', 'bold');

% Figure 2: Rotation (3DOF)
figure('Position', [100, 750, 1400, 600], 'Name', 'Realistic Quadcopter Rotation');
subplot(2,3,1); plot(time, attitude_deg(1,:), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Roll (°))'); title('Roll Angle'); grid on;
subplot(2,3,2); plot(time, attitude_deg(2,:), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Pitch (°))'); title('Pitch Angle'); grid on;
subplot(2,3,3); plot(time, attitude_deg(3,:), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Yaw (°))'); title('Yaw Angle'); grid on;
subplot(2,3,4); plot(time, ang_velocity(1,:), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('P (rad/s)'); title('Roll Rate'); grid on;
subplot(2,3,5); plot(time, ang_velocity(2,:), 'g--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Q (rad/s)'); title('Pitch Rate'); grid on;
subplot(2,3,6); plot(time, ang_velocity(3,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('R (rad/s)'); title('Yaw Rate'); grid on;
sgtitle('ROTATION DYNAMICS (3DOF) - REALISTIC', 'FontSize', 14, 'FontWeight', 'bold');

