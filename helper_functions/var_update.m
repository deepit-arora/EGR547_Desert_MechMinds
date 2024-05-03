function var_update(exportedData,controlsexportedData)

        % this function is designed to extract values from thje gui and
        % format them

        % for dynamics enter as var.. for controls as numbers 
variableNames = {'thetas', 'alphas', 'as', 'ds', 'joint_types', 'link_masses', 'motor_mass', 'I_motors', 'I_links', 'trans_ratios', 'friction_coeffs'};
for k = 1:length(variableNames)
    currentData = exportedData{:, k};
    if isnumeric(currentData)
        % Remove NaNs and ensure it is a column vector if it is numeric
        currentData = currentData(~isnan(currentData));
        if size(currentData, 1) == 1 && size(currentData, 2) > 1
            currentData = currentData'; % Transpose if single row
        end
    else
        % For non-numeric data, just ensure it is in column format
        currentData = currentData(~cellfun(@isempty, currentData));
        if size(currentData, 1) == 1 && size(currentData, 2) > 1
            currentData = currentData';
        end
    end
    % Handle cases where the entire column is empty or non-applicable data
    if isempty(currentData)
        currentData = NaN; 
    end
    assignin('base', variableNames{k}, currentData');
end

% CONTROL TABLE
contVariableNames = { 'initial_euler_angles', 'intial_ef_xyz','desired_xyz', 'num_theta', 'num_alpha', 'num_a', 'num_d', 'gravity_mat', 'des_time','compliance_gains', 'impedance_gains','desired_pos_coeff'};
for k = 1:length(contVariableNames)
    controlscurrentData = str2num(cell2mat(controlsexportedData{1, k}));
  controlscurrentData = controlscurrentData';
    if isempty(controlscurrentData)
        controlscurrentData = NaN; 
    end
    assignin('base', contVariableNames{k}, controlscurrentData');
end

% extract and label gains for compliance and impedance
compliance_gains = evalin('base', 'compliance_gains');
impedance_gains = evalin('base', 'impedance_gains');
if exist('compliance_gains', 'var')==1 && all(~isnan(compliance_gains   ), 'all')
    compliance_kp = compliance_gains(1,:);
    compliance_kd = compliance_gains(2,:);
    compliance_k = compliance_gains(3,:);
    assignin('base', 'compliance_kd', compliance_kd);
    assignin('base', 'compliance_kp', compliance_kp);
    assignin('base', 'compliance_k', compliance_k);
end
if exist('impedance_gains', 'var') == 1 && all(~isnan(impedance_gains), 'all')
    impedance_kp = impedance_gains(1,:);
    impedance_ke = impedance_gains(2,:);
    impedance_kd = impedance_gains(3,:);
    impedance_kde = impedance_gains(4,:);
    impedance_md = impedance_gains(5,:);
    impedance_mde = impedance_gains(6,:);

    assignin('base', 'impedance_kp', impedance_kp);
    assignin('base', 'impedance_ke', impedance_ke);
    assignin('base', 'impedance_kd', impedance_kd);
    assignin('base', 'impedance_kde', impedance_kde);
    assignin('base', 'impedance_md', impedance_md);
    assignin('base', 'impedance_mde', impedance_mde);

end

% to turn joint types to list
joint_types = evalin('base', 'joint_types');
if exist('joint_types', 'var') == 1  
    max_length = max(cellfun(@length, joint_types));
    joint_types_list = char(cellfun(@(x) [x repmat(' ', 1, max_length - length(x))], joint_types, 'UniformOutput', false));
    joint_types_list = joint_types_list';
    assignin('base', 'joint_types_list', joint_types_list);
end


% FORMAT FIX
fixvars = {'link_masses', 'motor_mass', 'I_motors', 'I_links', 'trans_ratios', 'friction_coeffs'};
for k = 1:length(fixvars)
    value = evalin('base', fixvars{k});
    converted_value = str2double(value);
    assignin('base', fixvars{k}, converted_value);
end


end


