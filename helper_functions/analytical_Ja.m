%% Function - Jacobian from DH parameters
function J_a = analytical_Ja(thetas, alphas, a, d)
    % This function accepts DH parameters
    % and returns the analytical jacobian.

    % USAGE: 
    %        input thetas as radians or variables as a cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e thetas = {'_theta1' 0 0}, or  
    %        input alphas as radians as cell matrix, i.e alphas = {0 pi/2 pi/4}
    %        input a as numbers as cell matrix, i.e a = {a1 0 0}
    %        input d as numbers or variables as cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e d = {0 'd2' '_d3'}
    % TO RUN THE CODE, DO:
    % J_a = analytical_Ja(thetas, alphas, a, d)
    
    number_joints = length(thetas);
    
    % Getting T0H and T0H_symbolic
    [T0H, ~, joint_variables, ~] = f_kinematics(thetas, alphas, a, d);


    % assigning J_a as a 3xn matrix - it is NOT 6xn because we do not
    % care about the orientation
    J_a = sym('J_a', [3, number_joints]);
    
    % looping thru the number of joints and taking the partial derivative
    % wrt to each 'q' for each column.
    for i=1:number_joints
        J_a(:, i) = diff(T0H{number_joints}(1:3, 4), joint_variables{i});
    end
    
    J_a = simplify(J_a);
end
