%% Function - Jacobian from DH parameters
function J_a = analytical_Ja(thetas, alphas, a, d)
    % This function accepts DH parameters
    % and returns the analytical jacobian.

    % USAGE: 
    %        input thetas as radians, i.e thetas = [pi/4 0 0]
    %        input alphas as radians, i.e alphas = [0 pi/2 pi/4]
    %        input a as numbers, i.e a = [1 0 0]
    %        input d as numbers, i.e d = [0 1.5 2.5]
    %     to return the reduced analytical Jacobian matrix J_a
    
    number_joints = length(thetas);
    
    % Getting T0H and T0H_symbolic
    [T0H, T0H_sym] = f_kinematics(thetas, alphas, a, d);

    % q_list
    q_list = cell(1, number_joints);

    % getting symbolic joint variables from input
    for i = 1:number_joints
        q_list{i} = sym(sprintf('q%d', i));
    end

    % assigning J_a as a 3xn matrix - it is NOT 6xn because we do not
    % care about the orientation
    J_a = sym('J_a', [3, number_joints]);
    
    % looping thru the number of joints and taking the partial derivative
    % wrt to each 'q' for each column.
    for i=1:number_joints
        J_a(:, i) = diff(T0H_sym(1:3, i), q_list{i});
    end
    
    J_a = simplify(J_a);
end
