%   From iris.sdf
x_position = 0.13;      %   Absolute displacement of the 4 propellers in x-axis (quad-x configuration)
y_position = 0.22;      %   Absolute displacement of the 4 propellers in y-axis (quad-x configuration)
motorConstant = 5.84e-06;     %   From iris.sdf
momentConstant = 0.06;        %   From iris.sdf
maxRotVelocity = 1100;        % [rad/s] From iris.sdf
max_force = motorConstant * (maxRotVelocity^2)
max_x_torque = 2 * max_force * x_position
max_y_torque = 2 * max_force * y_position
max_thrust = 4 * max_force
max_yaw_torque = 4 * momentConstant * max_force  % not correct