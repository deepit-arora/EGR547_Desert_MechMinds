function var_update(exportedData)


variableNames = {'thetas', 'alphas', 'as', 'ds', 'link_masses', 'motor_mass', 'I_motors', 'I_links'};
for k = 1:length(variableNames)
    numericData = str2double(exportedData{:, k}); % Convert string to double
    cleanData = numericData(~isnan(numericData)); % Remove NaN values
    assignin('base', variableNames{k}, cleanData'); % Assign clean data to base workspace
end

thetas = evalin('base', 'thetas');
alphas = evalin('base', 'alphas');
as = evalin('base', 'as');
ds = evalin('base', 'ds');
link_masses =evalin('base', 'link_masses');
motor_mass = evalin('base', 'motor_mass');
I_motors = evalin('base', 'I_motors');
I_links = evalin('base', 'I_links');


% eqns 
[T0H, T0H_sym] = f_kinematics(thetas, alphas, as, ds);

ja = analytical_Ja(thetas, alphas, as, ds);

anal_jac = analytical_jacobian(thetas, alphas, a, d, Q);



assignin('base', "T0H_sym", T0H_sym )



end


