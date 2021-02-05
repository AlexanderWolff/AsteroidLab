classdef Simulation < handle
    properties (GetAccess='public', SetAccess='private')
       
       robot_class
       parameters
       
    end
    
    properties (Constant)
       
    % UR3 Parameters
    % theta (rad), a (m), d(m), alpha (rad)
    UR3_parameters_kinematics = [0,        0, 0.15190, pi/2; ...
                                 0, -0.24365,       0,    0; ...
                                 0, -0.21325,       0,    0; ...
                                 0,        0, 0.11235, pi/2; ...
                                 0,        0, 0.08535,-pi/2; ...
                                 0,        0, 0.08190,    0];

    % mass (kg), centre of mass (m)
    UR3_parameters_dynamics = { 2.00, [   0, -0.02,      0]; ...
                                3.42, [0.13,     0, 0.1157]; ...
                                1.26, [0.05,     0, 0.0238]; ...
                                0.80, [   0,     0,   0.01]; ...
                                0.80, [   0,     0,   0.01]; ...
                                0.35, [   0,     0,  -0.02]};


    % UR5 Parameters
    % theta (rad), a (m), d(m), alpha (rad)
    UR5_parameters_kinematics = [0,       0, 0.089159, pi/2; ...
                                 0,-0.42500,        0,    0; ...
                                 0,-0.39225,        0,    0; ...
                                 0,       0, 0.109150, pi/2; ...
                                 0,       0, 0.094650,-pi/2; ...
                                 0,       0, 0.082300,    0];

    % mass (kg), centre of mass (m)
    UR5_parameters_dynamics = { 3.7000, [      0,-0.02561, 0.001930]; ...
                                8.3930, [0.21255,       0, 0.113360]; ...
                                2.3300, [0.15000,       0, 0.026500]; ...
                                1.2190, [      0,-0.00180, 0.016340]; ...
                                1.2190, [      0, 0.00180, 0.016340]; ...
                                0.1879, [      0,       0,-0.001159]};
        
    end
    
    methods (Access = 'public')
        function this = Simulation(robot_class)
            this.robot_class = robot_class;
            
            switch robot_class
               
                case 'UR3'
                    this.parameters.kinematics = UR3_parameters_kinematics;
                    this.parameters.dynamics = UR3_parameters_dynamics;
                case 'UR5'
                    this.parameters.kinematics = UR5_parameters_kinematics;
                    this.parameters.dynamics = UR5_parameters_dynamics;
                otherwise
                    disp("Robot Parameters not Configured");
            end
            
        end
        
        
    end
    
end