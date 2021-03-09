classdef Simulation < handle
    properties (GetAccess='public', SetAccess='private')
       
       robot_class
       parameters
       
       transform
    end
    
    properties (Constant, GetAccess='private')
       
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

    % Calculated Using the findWorkspace method in this class
    UR3_workspace = [-0.575163609153450,   0.575163609153450;...
                     -0.575163609153450,   0.575163609153450;...
                     -0.727063609153450   0.423263609153450;...
                     0,   0.735696925034515]
                            
                            
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
                    this.parameters.kinematics = Simulation.UR3_parameters_kinematics;
                    this.parameters.dynamics = Simulation.UR3_parameters_dynamics;
                    this.parameters.workspace = Simulation.UR3_workspace;
                case 'UR5'
                    this.parameters.kinematics = Simulation.UR5_parameters_kinematics;
                    this.parameters.dynamics = Simulation.UR5_parameters_dynamics;
                otherwise
                    disp("Robot Parameters not Configured");
            end
            
            this.transform.base_offset = Homogeneous.Empty;
            this.transform.tool_offset = Homogeneous.Empty;
            this.transform.chain = cell(7,1);
            this.transform.local = cell(7,1);
            
            this.Transform([0,0,0,0,0,0]);
        end
        
        function Set_Base_Offset(this,offset)
           this.transform.base_offset = offset; 
        end
        
        function Set_Tool_Offset(this,offset)
           this.transform.tool_offset = offset; 
        end
        
        function Frames = Transform(this, joint_angles)
            %% First Transform : Base -> First Joint (ShoulderA)
            p = [0,0,0,0];
            T_F0tF1 = Homogeneous.fromDH_F(p);
            
            R1 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(1)]).R);
            
            % F1 from F1
            F1_F1 = Homogeneous.Empty;
            F1_F1 = F1_F1.transform(R1);
            
            % F1 from F0
            F1_F0 = F1_F1.transform(T_F0tF1);
            
            
            %% Second Transform : First Joint -> Second Joint (ShoulderB)
            p = [0,this.parameters.kinematics(1,2:4)];
            T_F2tF1 = Homogeneous.fromDH_F(p);
            
            R2 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(2)]).R);
            
            % F2 from F2
            F2_F2 = Homogeneous.Empty;
            F2_F2 = F2_F2.transform(R2);
            
            % F2 from F1
            F2_F1 = F2_F2.transform(T_F2tF1);
            
            % F2 from F0
            F2_F0 = F2_F1.transform(F1_F0);
            
            %% Third Transform : Second Joint -> Third Joint (Elbow)
            p = [0,this.parameters.kinematics(2,2:4)];
            T_F3tF2 = Homogeneous.fromDH_F(p);
            
            R3 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(3)]).R);
            
            % F3 from F3
            F3_F3 = Homogeneous.Empty;
            F3_F3 = F3_F3.transform(R3);
             
            % F3 from F2
            F3_F2 = F3_F3.transform(T_F3tF2);
            
            % F3 from F0
            F3_F0 = F3_F2.transform(F2_F0);

            %% Fourth Transform : Third Joint -> Fourth Joint (WristA)
            p = [0,this.parameters.kinematics(3,2:4)];
            T_F4tF3 = Homogeneous.fromDH_F(p);
            
            R4 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(4)]).R);
            
            % F4 from F4
            F4_F4 = Homogeneous.Empty;
            F4_F4 = F4_F4.transform(R4);
            
            % F4 from F3
            F4_F3 = F4_F4.transform(T_F4tF3);
            
            % F4 from F0
            F4_F0 = F4_F3.transform(F3_F0);
            
            %% Fifth Transform : Fourth Joint -> Fifth Joint (WristB)
            p = [0,this.parameters.kinematics(4,2:4)];
            T_F5tF4 = Homogeneous.fromDH_F(p);
            
            R5 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(5)]).R);
            
            % F5 from F5
            F5_F5 = Homogeneous.Empty;
            F5_F5 = F5_F5.transform(R5);
            
            % F5 from F4
            F5_F4 = F5_F5.transform(T_F5tF4);
            
            % F5 from F0
            F5_F0 = F5_F4.transform(F4_F0);
            
            %% Sixth Transform : Fifth Joint -> Sixth Joint (WristC)
            p = [0,this.parameters.kinematics(5,2:4)];
            T_F6tF5 = Homogeneous.fromDH_F(p);
            
            R6 = Homogeneous.fromR(RMatrix.fromXYZ([0,0,joint_angles(6)]).R);
            
            % F6 from F6
            F6_F6 = Homogeneous.Empty;
            F6_F6 = F6_F6.transform(R6);
            
            % F6 from F5
            F6_F5 = F6_F6.transform(T_F6tF5);
            
            % F6 from F0
            F6_F0 = F6_F5.transform(F5_F0);
            
            %% Seventh Transform : Sixth Joint -> Tooltip
            p = [0,this.parameters.kinematics(6,2:4)];
            T_F7tF6 = Homogeneous.fromDH_F(p);
            
            % F7 from F7
            F7_F7 = Homogeneous.Empty;
            
            % F7 from F6
            F7_F6 = F7_F7.transform(T_F7tF6);
            
            % F7 from F0
            F7_F0 = F7_F6.transform(F6_F0);
            
            %% Update Internal Information
            this.transform.chain{1} = F1_F0;
            this.transform.chain{2} = F2_F1;
            this.transform.chain{3} = F3_F2;
            this.transform.chain{4} = F4_F3;
            this.transform.chain{5} = F5_F4;
            this.transform.chain{6} = F6_F5;
            this.transform.chain{7} = F7_F6; 
            
            this.transform.local{1} = F1_F0;
            this.transform.local{2} = F2_F0;
            this.transform.local{3} = F3_F0;
            this.transform.local{4} = F4_F0;
            this.transform.local{5} = F5_F0;
            this.transform.local{6} = F6_F0;
            this.transform.local{7} = F7_F0;
            
            Frames = cell(7,1);
            Frames{1} = F1_F0;
            Frames{2} = F2_F0;
            Frames{3} = F3_F0;
            Frames{4} = F4_F0;
            Frames{5} = F5_F0;
            Frames{6} = F6_F0;
            Frames{7} = F7_F0;
        end
        
        function gDisplay(this)
            % Global Display (+Base Offset)
            
            F1_F0 = this.transform.chain{1};
            F2_F1 = this.transform.chain{2};
            F3_F2 = this.transform.chain{3};
            F4_F3 = this.transform.chain{4};
            F5_F4 = this.transform.chain{5};
            F6_F5 = this.transform.chain{6};
            F7_F6 = this.transform.chain{7};
            
            F0_Foffset = this.transform.base_offset;
            F1_Foffset = F1_F0.transform(F0_Foffset);
            F2_Foffset = F2_F1.transform(F1_Foffset);
            F3_Foffset = F3_F2.transform(F2_Foffset);
            F4_Foffset = F4_F3.transform(F3_Foffset);
            F5_Foffset = F5_F4.transform(F4_Foffset);
            F6_Foffset = F6_F5.transform(F5_Foffset);
            F7_Foffset = F7_F6.transform(F6_Foffset);
            
            Ftool_F7 = this.transform.tool_offset;
            Ftool_Foffset = Ftool_F7.transform(F7_Foffset);
            
            F0 = F0_Foffset;
            F1 = F1_Foffset;
            F2 = F2_Foffset;
            F3 = F3_Foffset;
            F4 = F4_Foffset;
            F5 = F5_Foffset;
            F6 = F6_Foffset;
            F7 = F7_Foffset;
            
            Ftool = Ftool_Foffset;
            
            F0.plot
            text(F0.T(1),F0.T(2),F0.T(3),'    B')
            
            F1.plot
            text(F1.T(1),F1.T(2),F1.T(3),'J1')
            
            F2.plot
            text(F2.T(1),F2.T(2),F2.T(3),'J2')
            
            F3.plot
            text(F3.T(1),F3.T(2),F3.T(3),'J3')
            
            F4.plot
            text(F4.T(1),F4.T(2),F4.T(3),'J4')
            
            F5.plot
            text(F5.T(1),F5.T(2),F5.T(3),'J5')
            
            F6.plot
            text(F6.T(1),F6.T(2),F6.T(3),'J6')
            
            F7.plot
            text(F7.T(1),F7.T(2),F7.T(3),'Tip')
            
            Ftool.plot
            text(Ftool.T(1),Ftool.T(2),Ftool.T(3),'Tool')
            
            xlim([-0.8,0.8]);
            axis equal 
        end
        
        function lDisplay(this)
            % Local Display
            F1_F0 = this.transform.local{1};
            F2_F0 = this.transform.local{2};
            F3_F0 = this.transform.local{3};
            F4_F0 = this.transform.local{4};
            F5_F0 = this.transform.local{5};
            F6_F0 = this.transform.local{6};
            F7_F0 = this.transform.local{7};
            
            Ftool_F7 = this.transform.tool_offset;
            Ftool_F0 = Ftool_F7.transform(F7_F0);
            
            F0 = F1_F0;
            F1 = F1_F0;
            F2 = F2_F0;
            F3 = F3_F0;
            F4 = F4_F0;
            F5 = F5_F0;
            F6 = F6_F0;
            F7 = F7_F0;
            
            Ftool = Ftool_F0;
            
            F0.plot
            text(F0.T(1),F0.T(2),F0.T(3),'    B')
            
            F1.plot
            text(F1.T(1),F1.T(2),F1.T(3),'J1')
            
            F2.plot
            text(F2.T(1),F2.T(2),F2.T(3),'J2')
            
            F3.plot
            text(F3.T(1),F3.T(2),F3.T(3),'J3')
            
            F4.plot
            text(F4.T(1),F4.T(2),F4.T(3),'J4')
            
            F5.plot
            text(F5.T(1),F5.T(2),F5.T(3),'J5')
            
            F6.plot
            text(F6.T(1),F6.T(2),F6.T(3),'J6')
            
            F7.plot
            text(F7.T(1),F7.T(2),F7.T(3),'Tip')
            
            Ftool.plot
            text(Ftool.T(1),Ftool.T(2),Ftool.T(3),'Tool')
            
            xlim([-0.8,0.8]);
            axis equal 
        end
        
        function lDisplay_plane(this, plane)
            % Local Display
            
            hold on;
            
            F1_F0 = this.transform.local{1};
            F2_F0 = this.transform.local{2};
            F3_F0 = this.transform.local{3};
            F4_F0 = this.transform.local{4};
            F5_F0 = this.transform.local{5};
            F6_F0 = this.transform.local{6};
            F7_F0 = this.transform.local{7};
            
            Ftool_F7 = this.transform.tool_offset;
            Ftool_F0 = Ftool_F7.transform(F7_F0);
            
            F0 = F1_F0;
            F1 = F1_F0;
            F2 = F2_F0;
            F3 = F3_F0;
            F4 = F4_F0;
            F5 = F5_F0;
            F6 = F6_F0;
            F7 = F7_F0;
            
            Ftool = Ftool_F0;
            
            F0.plot_plane(plane)
            text(F0.T(1),F0.T(2),F0.T(3),'    B')
            
            F1.plot_plane(plane)
            text(F1.T(1),F1.T(2),F1.T(3),'J1')
            
            F2.plot_plane(plane)
            text(F2.T(1),F2.T(2),F2.T(3),'J2')
            
            F3.plot_plane(plane)
            text(F3.T(1),F3.T(2),F3.T(3),'J3')
            
            F4.plot_plane(plane)
            text(F4.T(1),F4.T(2),F4.T(3),'J4')
            
            F5.plot_plane(plane)
            text(F5.T(1),F5.T(2),F5.T(3),'J5')
            
            F6.plot_plane(plane)
            text(F6.T(1),F6.T(2),F6.T(3),'J6')
            
            F7.plot_plane(plane)
            text(F7.T(1),F7.T(2),F7.T(3),'Tip')
            
            Ftool.plot_plane(plane)
            text(Ftool.T(1),Ftool.T(2),Ftool.T(3),'Tool')
            
            xlim([-0.8,0.8]);
            axis equal 
        end
        
        function ForwardKinematics(this, joint_angles)
           
            joint_angles = deg2rad(joint_angles);
            
            % Calculate Forward Transform
            this.Transform(joint_angles);
        end
        
        function fK(this, joint_angles)
           
            joint_angles = deg2rad(joint_angles);
            
            % Calculate Forward Transform
            this.Transform(joint_angles);
            
            % Display in Local Frame
            figure('name',sprintf("%0.1f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f",...
                rad2deg(joint_angles(1)),...
                rad2deg(joint_angles(2)),...
                rad2deg(joint_angles(3)),...
                rad2deg(joint_angles(4)),...
                rad2deg(joint_angles(5)),...
                rad2deg(joint_angles(6))));
            hold on
            this.lDisplay;
        end
        
        function [mDX, mDY, mDZ, mDXYZ] = findWorkspace(this)
            sim = this;

            angles = linspace(-180,180,9);

            l = (length(angles)^6)+1;
            DX = zeros(1,l);
            DY = zeros(1,l);
            DZ = zeros(1,l);
            DXYZ = zeros(1,l);
            i = 1;
            checkpoint = 0.01;

            for a = 1:length(angles)

                for b = 1:length(angles)

                    for c = 1:length(angles)

                        for d = 1:length(angles)

                            for e = 1:length(angles)

                                for f = 1:length(angles)

                                    A = angles(a);
                                    B = angles(b);
                                    C = angles(c);
                                    D = angles(d);
                                    E = angles(e);
                                    F = angles(f);

                                    sim.ForwardKinematics([A,B,C,D,E,F]);

                                    %% Distance between tool and base
                                    F1 = sim.transform.local{1};
                                    F7 = sim.transform.local{7};
                                    Ftool_F7 = sim.transform.tool_offset;
                                    Ftool = Ftool_F7.transform(F7);

                                    Base = F1.T;
                                    Tool = Ftool.T;

                                    dx = Base(1)-Tool(1);
                                    dy = Base(2)-Tool(2);
                                    dz = Base(3)-Tool(3);

                                    dxyz = sqrt(dx^2+dy^2+dz^2);

                                    if(i/l>checkpoint)
                                        disp( strcat(string(round(i/l*100)),'%') );
                                        checkpoint = checkpoint + 0.01;
                                    end

                                    DX(i) = dx;
                                    DY(i) = dy;
                                    DZ(i) = dz;
                                    DXYZ(i) = dxyz;
                                    i = i+1;
                                end
                            end
                        end
                    end
                end
            end

            mDX = max(abs(DX));
            mDY = max(abs(DY));
            mDZ = max(abs(DZ));
            mDXYZ = max(abs(DXYZ));
        end
        
        
    end
    
end