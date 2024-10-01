%%% CLIK : Closed Loop Inverse Kinematics %%%
function u = clik(robot_obj, q, error, ee_des_vel, gain) 

    % Compute Jacobian
    J = robot_obj.jacob0(q);

    % Compute CLIK law
    u = J\(ee_des_vel + gain*eye(length(error))*error);

end