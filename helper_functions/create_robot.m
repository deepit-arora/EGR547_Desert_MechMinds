%% Function that creates and returns robot
function my_robot = create_robot(jointTypes, control_thetas, control_alphas, control_as, control_ds, Il_list_numeric, Im_list_numeric, ml_list_numeric, kr_list_numeric)
    global my_robot
    dh = [control_thetas' control_ds' control_as' control_alphas'];

    for i=1:length(control_thetas)
        if jointTypes(i) == 'R' || jointTypes(i) == 'r'
            L(i) = Link('revolute', 'd', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4), 'I', [Il_list_numeric(i) 0 0], 'Jm', [Im_list_numeric(i) 0 0], 'G', kr_list_numeric(i), 'm', ml_list_numeric(i));
        elseif jointTypes(i) == 'P' || jointTypes(i) == 'p'
            L(i) = Link('prismatic', 'theta', dh(i,1), 'a', dh(i,3), 'alpha', dh(i,4), 'I', [Il_list_numeric(i) 0 0], 'Jm', [Im_list_numeric(i) 0 0], 'G', kr_list_numeric(i), 'm', ml_list_numeric(i));
        end
    end
    
    my_robot = SerialLink(L);
end
