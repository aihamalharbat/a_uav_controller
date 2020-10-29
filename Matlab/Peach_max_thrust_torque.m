%   From peach.sdf
x_position = 0.2966;            % Absolute displacement of the 4 propellers in x-axis (hex-x configuration)
y_position = 0.1712;            % Absolute displacement of the 4 propellers in y-axis (hex-x configuration)
arm = 0.3425;                   % Euclidean displacement of the propellers from COM
motorConstant = 8.54858e-06;    %   From iris.sdf
momentConstant = 0.06;          %   From iris.sdf
maxRotVelocity = 1500;          % [rad/s] From iris.sdf
max_force = motorConstant * (maxRotVelocity^2)
max_x_torque = (2 * max_force * y_position ) + (max_force * arm)
max_y_torque = 2 * max_force * x_position
max_thrust = 6 * max_force
max_yaw_torque = 6 * momentConstant * max_force % not correct