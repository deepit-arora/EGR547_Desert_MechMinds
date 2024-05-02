%% TRAPEZOIDAL VELOCITY PROFILE
function [position, velocity, acceleration, time] = trapezoidal_trajectory(initial_position, desired_position, T)

    a_max = 3;

    % Calculate the total distance vector and its magnitude
    delta_p = desired_position - initial_position;
    distance = norm(delta_p);  % Euclidean distance
    
    % Compute the unit vector for direction
    direction = delta_p / distance;

    % Calculate the time to accelerate and decelerate using maximum acceleration
    T_a = sqrt(distance / a_max);  % Time to accelerate from 0 to v_max assuming constant acceleration
    v_max = a_max * T_a;

    % Adjust time to accelerate/decelerate if the full profile cannot be achieved within the total time
    if T < 2 * T_a
        T_a = T / 2;
        v_max = a_max * T_a;
    end

    % Time vector
    dt = 0.01;  % time step for better resolution
    time = 0:dt:T;
    position = zeros(length(time), 3);
    velocity = zeros(length(time), 3);
    acceleration = zeros(length(time), 3);

    % Calculate position, velocity, acceleration for each component
    for i = 1:length(time)
        t = time(i);
        if t < T_a
            % Acceleration phase
            position(i, :) = initial_position + 0.5 * a_max * t^2 * direction;
            velocity(i, :) = a_max * t * direction;
            acceleration(i, :) = a_max * direction;
        elseif t >= T_a && t <= (T - T_a)
            % Constant velocity phase
            position(i, :) = initial_position + v_max * (t - T_a) * direction + 0.5 * a_max * T_a^2 * direction;
            velocity(i, :) = v_max * direction;
            acceleration(i, :) = [0 0 0];
        elseif t > (T - T_a) && t <= T
            % Deceleration phase
            position(i, :) = desired_position - 0.5 * a_max * (T - t)^2 * direction;
            velocity(i, :) = a_max * (T - t) * direction;
            acceleration(i, :) = -a_max * direction;
        else
            % Post trajectory
            position(i, :) = desired_position;
            velocity(i, :) = [0 0 0];
            acceleration(i, :) = [0 0 0];
        end
    end

end