classdef Simulation < handle
    properties (GetAccess='public', SetAccess='private')
       
       robot_class
       parameters
       
       transform
       joints
       tool
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
            this.tool.local = Homogeneous.Empty;
            this.tool.global = Homogeneous.Empty;
            
            this.Transform([0,0,0,0,0,0]);
            this.Calculate_Tool_Radius();
            this.parameters.bot_margin = 0.05;
        end
        
        function Calculate_Tool_Radius(this)          
            % needs transform to have been run at least once
            A = this.transform.local{5}.T;
            B = this.transform.local{8}.T;

            dx = abs(B(1) - A(1));
            dy = abs(B(2) - A(2));
            dz = abs(B(3) - A(3));
            distance = sqrt(dx^2 + dy^2 + dz^2); 
            
            this.parameters.tool_radius = distance;
        end
        
        function Set_Base_Offset(this,offset)
           this.transform.base_offset = offset; 
        end
        
        function Set_Tool_Offset(this,offset)
           this.transform.tool_offset = offset; 
        end
        
        function Frames = Transform(this, joint_angles)
            
            this.joints.rad = joint_angles;
            
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
            
            %% Eigth Transform : Tooltip -> Tool 
            FTool_F7 = this.transform.tool_offset;
            FTool_F0 = FTool_F7.transform(F7_F0);
            
            %% Update Internal Information
            this.transform.chain{1} = F1_F0;
            this.transform.chain{2} = F2_F1;
            this.transform.chain{3} = F3_F2;
            this.transform.chain{4} = F4_F3;
            this.transform.chain{5} = F5_F4;
            this.transform.chain{6} = F6_F5;
            this.transform.chain{7} = F7_F6; 
            this.transform.chain{8} = FTool_F7; 
            
            this.transform.local{1} = F1_F0;
            this.transform.local{2} = F2_F0;
            this.transform.local{3} = F3_F0;
            this.transform.local{4} = F4_F0;
            this.transform.local{5} = F5_F0;
            this.transform.local{6} = F6_F0;
            this.transform.local{7} = F7_F0;
            this.transform.local{8} = FTool_F0;
            
            Frames = cell(7,1);
            Frames{1} = F1_F0;
            Frames{2} = F2_F0;
            Frames{3} = F3_F0;
            Frames{4} = F4_F0;
            Frames{5} = F5_F0;
            Frames{6} = F6_F0;
            Frames{7} = F7_F0;
            Frames{8} = FTool_F0;
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
           
            this.joints.deg = joint_angles;
            joint_angles = deg2rad(joint_angles);
            %disp(joint_angles)
            
            if ~isreal(joint_angles)
                %disp('Error! Imaginary Angle');
                
                this.Transform(real(joint_angles));
                return
            end
                        
            % Calculate Forward Transform
            this.Transform(joint_angles);
        end
        
        function fK(this, joint_angles)
           
            this.joints.deg = joint_angles;
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
        
        function a_diff = align_right_ascension(this, Target)
            
            current_joints = this.joints.deg;
            
            %% Align with Target in Right-Ascension
            F1 = this.transform.local{1};
            FTool = this.transform.local{8};

            Base = F1.T;
            Tool = FTool.T;
            Targ = Target.T;

            % Find angle diff between Target and Tool
            l1 = sqrt( (Base(1)-Tool(1))^2 + (Base(2)-Tool(2))^2  );
            l2 = sqrt( (Base(1)-Targ(1))^2 + (Base(2)-Targ(2))^2  );
            l3 = sqrt( (Tool(1)-Targ(1))^2 + (Tool(2)-Targ(2))^2  );

            % by Cosine Rule
            angle_diff = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff = round(rad2deg(angle_diff),6);

            if(isreal(a_diff))
                % Move first Joint until angle diff is zero
                this.ForwardKinematics(current_joints + [a_diff,0,0,0,0,0]);
            else
                a_diff = inf;
                return;
            end

            %% Confirm Direction
            F1 = this.transform.local{1};
            FTool = this.transform.local{8};

            Base = F1.T;
            Tool = FTool.T;
            Targ = Target.T;

            % Find angle diff between Target and Tool
            l1 = sqrt( (Base(1)-Tool(1))^2 + (Base(2)-Tool(2))^2  );
            l2 = sqrt( (Base(1)-Targ(1))^2 + (Base(2)-Targ(2))^2  );
            l3 = sqrt( (Tool(1)-Targ(1))^2 + (Tool(2)-Targ(2))^2  );

            % by Cosine Rule
            angle_diff_new = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff_new = round(rad2deg(angle_diff_new),6);

            % If angle difference increased: invert direction
            if a_diff_new > a_diff
                this.ForwardKinematics(current_joints - [a_diff,0,0,0,0,0]);
            else
                this.ForwardKinematics(current_joints + [a_diff,0,0,0,0,0]);
            end
        end
        
        function a_diff = align_declination(this, Target)
            
            current_joints = this.joints.deg;
                        
           %% Align with Target in declination
            F2 = this.transform.local{2};
            FTool = this.transform.local{8};

            Base = F2.T;
            Tool = FTool.T;
            Targ = Target.T;

            % Find angle diff between Target and Tool
            l1 = sqrt( (Base(1)-Tool(1))^2 + (Base(2)-Tool(2))^2 + (Base(3)-Tool(3))^2  );
            l2 = sqrt( (Base(1)-Targ(1))^2 + (Base(2)-Targ(2))^2 + (Base(3)-Targ(3))^2 );
            l3 = sqrt( (Tool(1)-Targ(1))^2 + (Tool(2)-Targ(2))^2 + (Tool(3)-Targ(3))^2 );

            % by Cosine Rule
            angle_diff = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff = round(rad2deg(angle_diff),6);

            if(isreal(a_diff))
                % Move first Joint until angle diff is zero
                this.ForwardKinematics(current_joints - [0,a_diff,0,0,0,0]);
            else
                a_diff = inf;
                return;
            end

            %% Confirm Direction
            F2 = this.transform.local{2};
            FTool = this.transform.local{8};

            Base = F2.T;
            Tool = FTool.T;
            Targ = Target.T;

            % Find angle diff between Target and Tool
            l1 = sqrt( (Base(1)-Tool(1))^2 + (Base(2)-Tool(2))^2 + (Base(3)-Tool(3))^2  );
            l2 = sqrt( (Base(1)-Targ(1))^2 + (Base(2)-Targ(2))^2 + (Base(3)-Targ(3))^2 );
            l3 = sqrt( (Tool(1)-Targ(1))^2 + (Tool(2)-Targ(2))^2 + (Tool(3)-Targ(3))^2 );

            % by Cosine Rule
            angle_diff_new = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff_new = round(rad2deg(angle_diff_new),6);

            % If angle difference increased: invert direction
            if a_diff_new > a_diff
                this.ForwardKinematics(current_joints + [0,a_diff,0,0,0,0]);
            else
                this.ForwardKinematics(current_joints - [0,a_diff,0,0,0,0]);
            end 
        end
        
        function a_diff = align_altitude(this, Target)
            
            current_joints = this.joints.deg;
            
            % Current Joints
            j1 = current_joints(1);
            j2 = current_joints(2);
            j3 = current_joints(3);
            j4 = current_joints(4);
            j5 = current_joints(5);
            j6 = current_joints(6);
            
            %% J4-J3-J2 Triangle
            F2 = this.transform.local{2};
            F3 = this.transform.local{3};
            F4 = this.transform.local{4};

            J2 = F2.T;
            J3 = F3.T;
            J4 = F4.T;

            % L1 : J2 and J3
            L1 = sqrt( (J2(1)-J3(1))^2 + (J2(2)-J3(2))^2 + (J2(3)-J3(3))^2 );

            % L2 : J3 and J4
            L2 = sqrt( (J3(1)-J4(1))^2 + (J3(2)-J4(2))^2 + (J3(3)-J4(3))^2 );

            % Shoulder : J2
            Js = j2;

            % Elbow : J3
            Je = j3;

            % Current Altitude of Wrist : Distance J2 to J4
            Altitude.wrist = sqrt( (J2(1)-J4(1))^2 + (J2(2)-J4(2))^2 + (J2(3)-J4(3))^2 );

            % Altitude of Tool
            F8 = this.transform.local{8};
            J8 = F8.T;
            Altitude.tool = sqrt( (J2(1)-J8(1))^2 + (J2(2)-J8(2))^2 + (J2(3)-J8(3))^2 );

            % Altitude of Target
            T = Target.T;
            Altitude.target = sqrt( (J2(1)-T(1))^2 + (J2(2)-T(2))^2 + (J2(3)-T(3))^2 );

            % Difference in Altitude
            Altitude.diff = Altitude.target - Altitude.tool;

            % New altitude of wrist
            Altitude.new = Altitude.wrist + Altitude.diff; 

            [new_Js, new_Je, dJw] = AltControl(Js, Je, L1, L2, Altitude.new);

            new_j2 = rad2deg(new_Js);
            new_j3 = rad2deg(new_Je);

            new_j4 = j4 + rad2deg(dJw);
            this.ForwardKinematics([j1, new_j2, new_j3, new_j4, j5, j6]); 
            
            a_diff = Altitude.diff;
            
        end
        
        function a_diff = align_Joint6(this, Target_F0)
            current_joints = this.joints.deg;
            scale = 0.1;
            

            % Find Joints in F7 Frame
            F7_F0 = this.transform.local{7};
            F0_F7 = F7_F0.inv;
            Target_F7 = Target_F0.transform(F0_F7);
            FTool_F7 = this.transform.chain{8}; 

            TargetX = Homogeneous.fromT([scale,0,0]);
            TargetX = TargetX.transform(Target_F7).T;
            ToolX = Homogeneous.fromT([scale,0,0]);
            ToolX = ToolX.transform(FTool_F7).T;
            FTool = FTool_F7.T;

            % Find Triangle Distances
            l1 = sqrt((TargetX(1)-FTool(1))^2+(TargetX(2)-FTool(2))^2);
            l2 = sqrt((ToolX(1)-FTool(1))^2+(ToolX(2)-FTool(2))^2);
            l3 = sqrt((ToolX(1)-TargetX(1))^2+(ToolX(2)-TargetX(2))^2);

            % Cosine Rule
            angle = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff = rad2deg(angle);

            this.ForwardKinematics(current_joints + [0,0,0,0,0,a_diff]);

            %% Check if Moved in Correct Direction
            % Find Joints in F7 Frame
            F7_F0 = this.transform.local{7};
            F0_F7 = F7_F0.inv;
            Target_F7 = Target_F0.transform(F0_F7);
            FTool_F7 = this.transform.chain{8}; 

            TargetX = Homogeneous.fromT([scale,0,0]);
            TargetX = TargetX.transform(Target_F7).T;
            ToolX = Homogeneous.fromT([scale,0,0]);
            ToolX = ToolX.transform(FTool_F7).T;
            FTool = FTool_F7.T;

            % Find Triangle Distances
            l1 = sqrt((TargetX(1)-FTool(1))^2+(TargetX(2)-FTool(2))^2);
            l2 = sqrt((ToolX(1)-FTool(1))^2+(ToolX(2)-FTool(2))^2);
            l3 = sqrt((ToolX(1)-TargetX(1))^2+(ToolX(2)-TargetX(2))^2);

            % Cosine Rule
            angle = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff_new = rad2deg(angle);

            if a_diff_new > a_diff
                this.ForwardKinematics(current_joints - [0,0,0,0,0,a_diff]);
            else
                this.ForwardKinematics(current_joints + [0,0,0,0,0,a_diff]);
            end
        end
        
        function a_diff = align_Joint4(this, Target_F0)            
            %% Make Z axis of Tool parallel to Z axis of Target in X-Y by moving F4
            scale = 0.1;
            current_joints = this.joints.deg;

            % In Frame of F4
            F6_F5 = this.transform.chain{6};
            F7_F6 = this.transform.chain{7}; 
            FTool_F7 = this.transform.chain{8}; 
            F4_F0 = this.transform.local{4};
            F0_F4 = F4_F0.inv;
            
            F5_F4 = this.transform.chain{5};
            F6_F4 = F6_F5.transform(F5_F4);
            F7_F4 = F7_F6.transform(F6_F4);
            FTool_F4 = FTool_F7.transform(F7_F4);
            Target_F4 = Target_F0.transform(F0_F4);
            FTool = FTool_F4.T;
            TargetZ = Homogeneous.fromT([0,0,scale]);
            TargetZ = TargetZ.transform(Target_F4);
            TargetZ = TargetZ.T;
            ToolZ = Homogeneous.fromT([0,0,scale]);
            ToolZ = ToolZ.transform(FTool_F4);
            ToolZ = ToolZ.T;

            % Find Triangle Distances
            l1 = sqrt((TargetZ(1)-FTool(1))^2+(TargetZ(2)-FTool(2))^2);
            l2 = sqrt((ToolZ(1)-FTool(1))^2+(ToolZ(2)-FTool(2))^2);
            l3 = sqrt((ToolZ(1)-TargetZ(1))^2+(ToolZ(2)-TargetZ(2))^2);

            % Cosine Rule
            angle = acos( (l1^2+l2^2-l3^2)/(2*l1*l2) );
            a_diff = rad2deg(angle);

            this.ForwardKinematics(current_joints + [0,0,0,a_diff,0,0]);

            % In Frame of F4
            F4_F0 = this.transform.local{4};
            F0_F4 = F4_F0.inv;
            
            Target_F4 = Target_F0.transform(F0_F4);
            Target = Target_F4.T;
            TargetZ = Homogeneous.fromT([0,0,scale]);
            TargetZ = TargetZ.transform(Target_F4);
            TargetZ = TargetZ.T;

            % Find out if Tool Z axis projected in XY parallel with Targ Z
            Target_dx = Target(1)-TargetZ(1);
            Target_dy = Target(2)-TargetZ(2);
            Target_grad = Target_dy/Target_dx;
            
            %ie. If grad is close to zero => success
            if abs(Target_grad)<1e-4
                this.ForwardKinematics(current_joints + [0,0,0,a_diff,0,0]);
            else
                this.ForwardKinematics(current_joints - [0,0,0,a_diff,0,0]);
            end
        end
        
        function align_orientation(this, Target, verbose)
           
            %verbose = false;
            
            for i = 1:1e2
                %% Joint 6
                current_joints = this.joints.deg;
                a = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                if(isreal(a))
                    this.ForwardKinematics(current_joints + [0,0,0,0,0,a(3)]);
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    if abs(a(3))<abs(a_diff(3))
                        this.ForwardKinematics(current_joints - [0,0,0,0,0,a(3)]);

                        a_diff_ = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                        if abs(a_diff(3))<abs(a_diff_(3))
                            this.ForwardKinematics(current_joints);
                        end
                    else
                        this.ForwardKinematics(current_joints + [0,0,0,0,0,a(3)]);
                    end
                end
                
                
                if verbose
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    fprintf('\nA3: %f -> %f\n', a(3),a_diff(3))
                    disp(a_diff)
                end

                %% Joint 5
                current_joints = this.joints.deg;
                a = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                if(isreal(a))
                    this.ForwardKinematics(current_joints + [0,0,0,0,a(2),0]);
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    if abs(a(2))<abs(a_diff(2))
                        this.ForwardKinematics(current_joints - [0,0,0,0,a(2),0]);

                        a_diff_ = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                        if abs(a_diff(2))<abs(a_diff_(2))
                            this.ForwardKinematics(current_joints);
                        end
                    else
                        this.ForwardKinematics(current_joints + [0,0,0,0,a(2),0]);
                    end
                end
                
                if verbose
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    fprintf('\nA2: %f -> %f\n', a(2),a_diff(2))

                    disp(a_diff)
                end
                
                %% Joint 4
                current_joints = this.joints.deg;
                a = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                if(isreal(a))
                    this.ForwardKinematics(current_joints + [0,0,0,a(1),0,0]);
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    if abs(a(1))<abs(a_diff(1))
                        this.ForwardKinematics(current_joints - [0,0,0,a(1),0,0]);

                        a_diff_ = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                        if abs(a_diff(1))<abs(a_diff_(1))
                            this.ForwardKinematics(current_joints);
                        end
                    else
                        this.ForwardKinematics(current_joints + [0,0,0,a(1),0,0]);
                    end
                end
                if verbose
                    a_diff = rad2deg(RMatrix(this.transformTarget_Tool(Target).R).toXYZ);
                    fprintf('\nA1: %f -> %f\n', a(1),a_diff(1))
                    disp(a_diff)
                end
                if max(abs(a_diff))<1e-5
                    break
                end 
            end
            
        end
        
        function distance = get_distance_to(this, Target)
            % Distance from Tooltip to Target
            FTool = this.transform.local{8};
                        
            dx = FTool.T(1)-Target.T(1);
            dy = FTool.T(2)-Target.T(2);
            dz = FTool.T(3)-Target.T(3);
            
            distance = sqrt(dx^2+dy^2+dz^2);
        end
        
        function [Target_Tool] = transformTarget_Tool(this,Target)
            
            FTool_F0 = this.transform.local{8};
            F0_FTool = FTool_F0.inv;

            Target_F0 = Target;
            Target_Tool = Target_F0.transform(F0_FTool);
        end
        
        function [joint_history, distances] = InverseK(this,Target,plot_dist)
            
            debug = false;
            
            %% Find a solution Iteratively
            max_i = 1e2;
            max_d = 1e-4;
            distances = zeros(1,max_i*3);
            distance = this.get_distance_to(Target);
            distances(1:3) = ones(1,3)*distance;
            joint_history = zeros(max_i*3,6);
            joint_history(1:3,:) = zeros(3,6)+this.joints.deg;
            for i = 2:max_i+1

                if(this.get_distance_to(Target) <= max_d)
                    distance = this.get_distance_to(Target);
                    distances(3*i-2) = distance;
                    distances = distances(3:3*i-2);
                    joint_history(3*i-2,:) = this.joints.deg;
                    joint_history = joint_history(3:3*i-2,:);

                    fprintf('\nSolution found within %i iterations\n', i);
                    disp(this.joints)

                    break;
                end

                %% Align in Right Ascension
                if debug 
                    disp('ARA') 
                end
                this.align_right_ascension(Target);

                % Align Orientation
                if debug
                    disp('ARA-O')
                end
                this.align_orientation(Target, false);
                
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i-2)
                    disp(new_distance-distance)
                end
                distance = new_distance;
                distances(3*i-2) = distance;
                joint_history(3*i-2,:) = this.joints.deg;
                
                
                
                %% Align with Target in Declination
                if debug
                    disp('AD')
                end
                this.align_declination(Target);
                
                % Align Orientation
                if debug
                    disp('AD-O')
                end
                this.align_orientation(Target, false);
                
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i-1)
                    disp(new_distance-distance)
                end
                distance = new_distance;
                distances(3*i-1) = distance;
                joint_history(3*i-1,:) = this.joints.deg;
                
                

                %% Align with Target in altitude
                if debug
                    disp('AA')
                end
                this.align_altitude(Target);
       
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i)
                    disp(new_distance-distance)
                end
                
                distance = new_distance;
                
                distances(3*i) = distance;
                joint_history(3*i,:) = this.joints.deg;
                
                if(i>1)
                    if abs(distances(3*(i-1))-distances(3*i))<1e-3 || distance < 1e-3
                        
                        joint_history = joint_history(3:3*i,:);
                        distances = distances(3:3*i);
                        
                        break
                    end
                end
            end
            if i == max_i
                disp('Solution not found within allowed iteration');
            end

            if plot_dist
                figure('name', 'Distance from Tool to Target');
                hold on;
                title('Distance from Tool to Target')
                ylabel('Distance To Target')
                xlabel('Iteration Epoch')
                plot([0,max_i],[0,0],'-k');
                plot([0,max_i],[max_d,max_d],'--r');
                plot(0,distance,'xb');
                for i = 1:length(distances)-1
                    plot(i,distances(i),'xb');
                end
                plot(i+1,distances(i+1),'xg');
                plot([i+1,max_i],[distances(i+1),distances(i+1)],'-g');
                xlim([0,length(distances)+1]);
            end
        end
        
        function [joint_history, distances] = InverseK_Position(this,Target,plot_dist)
            % Inverse Kinematics for position only
            
            debug = false;
            
            %% Find a solution Iteratively
            max_i = 1e2;
            max_d = 1e-4;
            distances = zeros(1,max_i*3);
            distance = this.get_distance_to(Target);
            distances(1:3) = ones(1,3)*distance;
            joint_history = zeros(max_i*3,6);
            joint_history(1:3,:) = zeros(3,6)+this.joints.deg;
            for i = 2:max_i+1

                if(this.get_distance_to(Target) <= max_d)
                    distance = this.get_distance_to(Target);
                    distances(3*i-2) = distance;
                    distances = distances(3:3*i-2);
                    joint_history(3*i-2,:) = this.joints.deg;
                    joint_history = joint_history(3:3*i-2,:);

                    fprintf('\nSolution found within %i iterations\n', i);
                    disp(this.joints)

                    break;
                end

                %% Align in Right Ascension
                if debug 
                    disp('ARA') 
                end
                this.align_right_ascension(Target);
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i-2)
                    disp(new_distance-distance)
                end
                distance = new_distance;
                distances(3*i-2) = distance;
                joint_history(3*i-2,:) = this.joints.deg;
                
                
                
                %% Align with Target in Declination
                if debug
                    disp('AD')
                end
                this.align_declination(Target);
                
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i-1)
                    disp(new_distance-distance)
                end
                distance = new_distance;
                distances(3*i-1) = distance;
                joint_history(3*i-1,:) = this.joints.deg;
                
                
                %% Align with Target in altitude
                if debug
                    disp('AA')
                end
                this.align_altitude(Target);
       
                new_distance = this.get_distance_to(Target);
                
                if debug
                    disp(3*i)
                    disp(new_distance-distance)
                end
                
                distance = new_distance;
                
                distances(3*i) = distance;
                joint_history(3*i,:) = this.joints.deg;
                
                if(i>1)
                    if abs(distances(3*(i-1))-distances(3*i))<1e-3 || distance < 1e-3
                        
                        joint_history = joint_history(3:3*i,:);
                        distances = distances(3:3*i);
                        
                        break
                    end
                end
            end
            if i == max_i
                disp('Solution not found within allowed iteration');
            end

            if plot_dist
                figure('name', 'Distance from Tool to Target');
                hold on;
                title('Distance from Tool to Target')
                ylabel('Distance To Target')
                xlabel('Iteration Epoch')
                plot([0,max_i],[0,0],'-k');
                plot([0,max_i],[max_d,max_d],'--r');
                plot(0,distance,'xb');
                for i = 1:length(distances)-1
                    plot(i,distances(i),'xb');
                end
                plot(i+1,distances(i+1),'xg');
                plot([i+1,max_i],[distances(i+1),distances(i+1)],'-g');
                xlim([0,length(distances)+1]);
            end
        end
        
        function [valid, constraint] = CheckWorkspace(this, Target)
            bot_margin = this.parameters.bot_margin;


            %% Check that Target is within Workspace
            % workspace is a sphere offset at Joint 2
            workspace.radius = 0.5*abs(mean(this.parameters.workspace(1:3,1) -  this.parameters.workspace(1:3,2)));
            workspace.offset = -1*0.5*(this.parameters.workspace(1:3,1) +  this.parameters.workspace(1:3,2));

            %% determine whether target is in reachable workspace
            % find distance between Target and workspace offset
            dx = Target.T(1) - workspace.offset(1);
            dy = Target.T(2) - workspace.offset(2);
            dz = Target.T(3) - workspace.offset(3);

            distance = sqrt( dx^2 + dy^2 + dz^2 );

            if distance <= workspace.radius
                constraint.reachable = true;
            else
                constraint.reachable = false;
            end

            %% plot operational workspace boundaries

            % target sphere of influence radius is sim.parameters.tool_radius
            operational_radius = workspace.radius-this.parameters.tool_radius;

            %% determine if point + orientation is in operational workspace

            if distance <= operational_radius
                constraint.operational = true;
            else
                constraint.operational = false;
            end

            %% reduce workspace with further constraints (eg. table)

            % create a cube (6 faces) which the target has to be within
            cube.bot = 0 + bot_margin;
            cube.top =  workspace.offset(3)+workspace.radius;


            %% determine whether target is within allowed workspace
            if Target.T(3) >= cube.bot
                constraint.bot = true;
            else
                constraint.bot = false;
            end

            if Target.T(3) <= cube.top
                constraint.top = true;
            else
                constraint.top = false;
            end

            if constraint.top &&  constraint.bot 
                constraint.allowed = true;
            else
                constraint.allowed = false;
            end

            if constraint.reachable && constraint.operational && constraint.allowed
                constraint.all = true;
            else
                constraint.all = false;
            end

            valid = constraint.all;
        end
        
        function DisplayWorkspace(this)
    
            bot_margin = this.parameters.bot_margin;


            %% Check that Target is within Workspace

            %% plot reachable workspace boundaries

            % workspace is a sphere offset at Joint 2
            workspace.radius = 0.5*abs(mean(this.parameters.workspace(1:3,1) -  this.parameters.workspace(1:3,2)));
            workspace.offset = -1*0.5*(this.parameters.workspace(1:3,1) +  this.parameters.workspace(1:3,2));

            [reach.X,reach.Y,reach.Z] = sphere(10);
            reach.X = (reach.X * workspace.radius) + workspace.offset(1);
            reach.Y = (reach.Y * workspace.radius) + workspace.offset(2);
            reach.Z = (reach.Z * workspace.radius) + workspace.offset(3);

            hold on;
            lightGrey = 0.8*[1 1 1];
            surface(reach.X, reach.Y, reach.Z, 'FaceColor', 'none','EdgeColor',lightGrey)
            axis equal

            %% plot operational workspace boundaries

            % target sphere of influence radius is sim.parameters.tool_radius
            operational_radius = workspace.radius-this.parameters.tool_radius;

            [operate.X,operate.Y,operate.Z] = sphere(20);
            operate.X = (operate.X * operational_radius ) + workspace.offset(1);
            operate.Y = (operate.Y * operational_radius ) + workspace.offset(2);
            operate.Z = (operate.Z * operational_radius ) + workspace.offset(3);

            lightBlue = 0.9*[.7 .7 1];
            surface(operate.X, operate.Y, operate.Z, 'FaceColor', 'none','EdgeColor',lightBlue)

            
            %% reduce workspace with further constraints (eg. table)

            % create a cube (6 faces) which the target has to be within
            cube.bot = 0 + bot_margin;
            cube.top =  workspace.offset(3)+workspace.radius;
            cube.sid = workspace.radius*2;

            cube.squ = [ cube.sid/2, cube.sid/2;...
                         cube.sid/2,-cube.sid/2;...
                        -cube.sid/2,-cube.sid/2;...
                        -cube.sid/2, cube.sid/2;...
                         cube.sid/2, cube.sid/2;...
                         ];


            %% determine whether target is within allowed workspace
            surf(cube.squ(:,1),...
                 cube.squ(:,2),...
                 cube.bot*ones(5,5), 'FaceColor',lightGrey )

            surf(cube.squ(:,1),...
                 cube.squ(:,2),...
                 cube.top*ones(5,5), 'FaceColor',lightGrey )
             alpha(.1)
            view(45,30)
        end
        
        function constraint = DisplayTarget(this, Target)
    
            %% Check that Target is within Workspace

            %% plot reachable workspace boundaries

            % workspace is a sphere offset at Joint 2
            workspace.radius = 0.5*abs(mean(this.parameters.workspace(1:3,1) -  this.parameters.workspace(1:3,2)));
            workspace.offset = -1*0.5*(this.parameters.workspace(1:3,1) +  this.parameters.workspace(1:3,2));

            hold on;
            axis equal

            Target.plotL('T');

            %% determine whether target is in reachable workspace

            % find distance between Target and workspace offset
            dx = Target.T(1) - workspace.offset(1);
            dy = Target.T(2) - workspace.offset(2);
            dz = Target.T(3) - workspace.offset(3);

            distance = sqrt( dx^2 + dy^2 + dz^2 );

            if distance <= workspace.radius
                constraint.reachable = true;
            else
                constraint.reachable = false;
            end

            %% plot operational workspace boundaries

            % target sphere of influence radius is sim.parameters.tool_radius
            operational_radius = workspace.radius-this.parameters.tool_radius;

            %% determine if point + orientation is in operational workspace

            if distance <= operational_radius
                constraint.operational = true;
            else
                constraint.operational = false;
            end

            if constraint.operational
                target.color = 0.9*[.7 1 .7];
            else
                if constraint.reachable
                    target.color = 0.9*[.7 .7 1];
                else
                    target.color = 0.9*[1 .7 .7];
                end
            end

            % sphere is centred along the vector of target-workspace offset
            % at one radius away from the target
            [target.X,target.Y,target.Z] = sphere(10);
            target.X = (target.X * this.parameters.tool_radius) + Target.T(1);
            target.Y = (target.Y * this.parameters.tool_radius) + Target.T(2);
            target.Z = (target.Z * this.parameters.tool_radius) + Target.T(3);

            surface(target.X, target.Y, target.Z, 'FaceColor', 'none','EdgeColor',target.color)
        end
        
        
        function valid = CheckPose(this)
            %% determine whether each robot joints are within allowed workspace
            valid = true;
            for i = 2:8
                [~, constraint] = this.CheckWorkspace(this.transform.local{i});
                valid = constraint.allowed && valid;
            end 
        end
        
    end
    
end