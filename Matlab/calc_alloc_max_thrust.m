% find the allocated max_thrust
mass = 3.506;                   % from peach.sdf
g = 9.81;
weight = mass * g;
hover_throttle = 0.7545;        % Found empirically
max_alloc_thrust = weight/hover_throttle