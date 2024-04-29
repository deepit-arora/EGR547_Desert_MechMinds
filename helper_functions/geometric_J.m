%% Generates the Geometric Jacobian
function J = geometric_J(thetas, alphas, a, d, jointTypes)
    % This function generates the geometric Jacobian based 
    % on T0H. This T0H is a 1xn cell matrix, which stores each T0n matrix.
    % It also requires a jointTypes list, with
    % 'P' corresponding to prismatic and 'R' corresponding to revolute.
    [T0H, ~, ~, ~] = f_kinematics(thetas, alphas, a, d);

    number_joints = length(T0H);

    %Creating symbolic Jacobian matrix
    J = sym(cell(6,length(thetas)));
    Pe = T0H{end}(1:3,4);

    for i = 1:number_joints
        % Prismatic Joints
        if(jointTypes(i) == 'P' || jointTypes(i) == 'p')
            if(i == 1)
                Zcomp = [0;0;1];
            else
                Zcomp = T0H{i-1}(1:3,3);
            end
            J(:,i) = [Zcomp;0;0;0];

        %Revolute Joints
        elseif(jointTypes(i) == 'R' || jointTypes(i) == 'r')
            %Find the P and the Z components
            if(i == 1)
                P = [0;0;0];
                Zcomp = [0;0;1]; 
            else
                P = T0H{i-1}(1:3,4);
                Zcomp = T0H{i-1}(1:3,3);  
            end
            %Find the Jacobian component
            Jcomp = cross(Zcomp,(Pe - P));
            J(:,i) = [Jcomp;Zcomp];
        end
    end
    J = simplify(J);
end
