%% Derive C matrix based on user inputs
function c = c_matrix(thetas, alphas, a, d, jointTypes)
    % This function accepts DH parameters and jointTypes,
    % and returns the c matrix.

    % USAGE:
    %        input thetas as radians or variables as cell matrix, i.e thetas = {'theta1' 0 0}
    %        input alphas as radians as cell matrix, i.e alphas = {0 -pi/2 0}
    %        input a as numbers as cell matrix, i.e a = {0 0 0}
    %        input d as numbers or variables as cell matrix, i.e d = {0 'd2' 'd3'}
    %        input jointTypes as letters, i.e jointTypes = ['R', 'P', 'P']
    %     to return the reduced c matrix, do:
    %     c = c_matrix(thetas, alphas, a, d, jointTypes)
    
    [~, ~, joint_variables] = f_kinematics(thetas, alphas, a, d);

    [B_sum, ~, ~, ~, ~] = inertia_matrix(thetas, alphas, a, d, jointTypes);

    number_joints = length(thetas);
    % setting up and getting c matrix
    c = sym(zeros(number_joints));

    for i=1:number_joints
        for j=1:number_joints
            for k=1:number_joints
                c(i, j) = simplify(1/2*(diff(B_sum(i, j), joint_variables{k}) + diff(B_sum(i, k), joint_variables{j}) - diff(B_sum(j, k), joint_variables{i})));
            end
        end
    end

end
