% CLIK : Closed Loop Inverse Kinematics
function u = clik(robot_obj, q, error, ee_des_vel, gain) 
    % Closed Loop Inverse Kinematics Implementation:
    % - robot_obj : Peter Corke's object that contains 
    % the kinematic informations of the manipulator
    % - error : Error in terms of ee's pose
    % - ee_des_vel: End-Effector Desired Velocity twist,
    % i.e., linear velocity and angular velocity
    % - gain: Controller Gain

    %% Compute Jacobian
    J = Rob.jacob0(q);
    %% Compute CLIK law
    u = J\(ee_des_vel + gain*eye(length(error))*error);

    % Notes:
    % -> here the gain is a scalar number (1x1), multiplied to 
    % an identity matrix.
    % -> I used backslash operator, that is much more better
    % than pinv or inv.
end