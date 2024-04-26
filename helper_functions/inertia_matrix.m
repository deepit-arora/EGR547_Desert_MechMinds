%% Derive Intertia Matrix based on DH inputs
function [B_sum, Jp_l, Jo_l, Jp_m, Jo_m] = inertia_matrix(thetas, alphas, a, d, jointTypes)
    % This function calculates the intertia matrix for a given set
    % of dh parameters, motor masses, and link masses, and returns it
    % as a 3xn (number of joints) matrix.

    % USAGE:
    %        input thetas as radians or variables as cell matrix, i.e thetas = {'theta1' 0 0}
    %        input alphas as radians as cell matrix, i.e alphas = {0 pi/2 pi/4}
    %        input a as numbers as cell matrix, i.e a = {a1 0 0}
    %        input d as numbers or variables as cell matrix, i.e d = {0 'd2' 'd3'}
    %        input jointTypes as letters, i.e jointTypes = ['R', 'P', 'P']
    %    to return intertia matrix B, do:
    %    B = inertia_matrix(thetas, alphas, a, d, jointTypes)
    %    B will be the 3xn intertia matrix corresponding to the DH params


    % Getting and checking number of joints
    number_joints = length(thetas);
    if (length(alphas) ~= number_joints) || (length(a) ~= number_joints) || (length(d) ~= number_joints)
        error('The input vectors must have the same length as the number of joints.');
    end
    
    % using forward kinematics function to get T0H and T0H syms matricies
    [T0H, T0H_sym, ~] = f_kinematics(thetas, alphas, a, d);

    % Setting motor and link symbolic masses
    for i = 1:number_joints
        ml_list{i} = sym(sprintf('ml_%d', i));
        mm_list{i} = sym(sprintf('mm_%d', i));
    end
    
    % Setting intertia symbolic variables
    for i = 1:number_joints
        Il_list{i} = sym(sprintf('Il_%d', i));
        Im_list{i} = sym(sprintf('Im_%d', i));
    end

    % Computation of Kinetic Energy:
    % Jacobians for Links:
    Jp_l = {};
    Jo_l = {};
    for i=1:number_joints
        Jp_l{i} = sym(zeros(3, number_joints));
        Jo_l{i} = sym(zeros(3, number_joints));

        % prismatic joints
        if jointTypes(i) == 'P' || jointTypes(i) == 'p'
            for j=1:number_joints
                if j <= i 
                    Jp_l{i}(:, j) = T0H_sym{j}(1:3, 3);
                end
            end
        
        % revolute joints
        % origin frame T0H(0) = T0H{1}
        elseif jointTypes(i) == 'R' || jointTypes(i) == 'r'
            for j=1:number_joints
                if j == 1                                 
                    Jp_l{i}(:, j) = cross(T0H_sym{j}(1:3, 3), T0H_sym{i}(1:3, 4));
                    Jo_l{i}(:, j) = T0H_sym{j}(1:3, 3);
                elseif (j ~= 1) && (j <= i)   % both edited from j to j-1
                    Jp_l{i}(:, j) = cross(T0H_sym{j-1}(1:3, 3), T0H_sym{i}(1:3, 4)-T0H_sym{j-1}(1:3, 4));
                    Jo_l{i}(:, j) = T0H_sym{j-1}(1:3, 3);
                end
            end
        end
    end
    
    % Jacobians for Motors:
    Jp_m = {};
    Jo_m = {};

    for i=1:number_joints
        Jp_m{i} = sym(zeros(3, number_joints));
        
        % prismatic joints
        if jointTypes(i) == 'P' || jointTypes(i) == 'p'
            for j=1:number_joints
                if j > i-1
                    continue
                else
                    Jp_m{i}(:, j) = T0H_sym{j}(1:3, 3);
                end
            end
        
        % revolute joints
        elseif jointTypes(i) == 'R' || jointTypes(i) == 'r'
            for j=1:number_joints
                if j > i-1
                    continue
                else
                    if j == 1 
                        Jp_m{i}(:, j) = cross(T0H_sym{j}(1:3, 3), T0H_sym{i}(1:3, 4));
                    else
                        Jp_m{i}(:, j) = cross(T0H_sym{j-1}(1:3, 3), T0H_sym{i}(1:3, 4) - T0H_sym{j-1}(1:3, 4));
                
                    end
                end
            end
        end
    end

    for i = 1:number_joints
        Jo_m{i} = sym(zeros(3, number_joints));
        krm_i = sym(sprintf('krm%d', i));
        for j=1:number_joints
            if j < i
                Jo_m{i}(:, j) = Jo_l{i}(:, j);
            elseif j==i
                Jo_m{i}(:, j) = krm_i*T0H{i}(1:3, 3);
            end
        end
    end

    % Finding the B(q) matrix from the sum equation:
    for i=1:number_joints
        R_i_l = T0H_sym{i}(1:3, 1:3);
        if i==1
            R_i_m = [[1 0 0]' [0 1 0]' [0 0 1]'];
        else
            R_i_m = T0H_sym{i-1}(1:3, 1:3);
        end
        B{i} = ml_list{i}*transpose(Jp_l{i})*Jp_l{i} + ...
                 transpose(Jo_l{i})*R_i_l*Il_list{i}*transpose(R_i_l)*Jo_l{i} + ...
                 mm_list{i}*transpose(Jp_m{i})*Jp_m{i} + ...
                 transpose(Jo_m{i})*R_i_m*Im_list{i}*transpose(R_i_m)*Jo_m{i};
    end

    B_sum = zeros(size(B{1}, 1), size(B{1}, 2));
    for i=1:number_joints
        B_sum = simplify(B_sum + B{i});
    end
    
end

