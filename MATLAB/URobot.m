

classdef URobot < handle
    properties (GetAccess='public', SetAccess='public')
       
       ip
       rtde
       connected
       sim
       
       latest_log
       
    end
    
   
    methods (Access = 'public')
        function this = URobot(ip)
            this.ip = ip;
            this.connected = false;
        end
        
        function Connect(this)
            this.rtde = RTDE(this.ip, 30004);
            this.rtde.Connect();
            this.connected = true;
        end
        
        function j_pos = Get_Joint_Pos(this)
           
            this.rtde.Set_Outputs('timestamp,actual_q');
            this.rtde.Start();
            pause(0.01);
            this.rtde.Pause();
            this.rtde.Collect();
            
            j_pos = this.rtde.latest{2};
            
        end
        
        function tcp_pose = Get_TCP_Pos(this)
            this.rtde.Set_Outputs('timestamp,actual_TCP_pose');
            this.rtde.Start();
            pause(0.01);
            this.rtde.Pause();
            this.rtde.Collect();
            
            tcp_pose = this.rtde.latest{2};
        end
        
        function tcp_pose = Get_Runtime(this)
            this.rtde.Set_Outputs('timestamp,runtime_state');
            this.rtde.Start();
            pause(0.01);
            this.rtde.Pause();
            this.rtde.Collect();
            
            tcp_pose = this.rtde.latest{2};
        end
        
        function tcp_pose = Get_ToolVoltage(this)
            this.rtde.Set_Outputs('timestamp,tool_output_voltage');
            this.rtde.Start();
            pause(0.01);
            this.rtde.Pause();
            this.rtde.Collect();
            
            tcp_pose = this.rtde.latest{2};
        end
        
        
        
        
        function Move_TCP(this, tcp, x)

            
            
            
            if length(x)<2
                max_speed = 0.5;
                accel = 0.5;
                time = 0;
                blend_radius = 0;
            else
                if length(x) == 2
                    max_speed = x(1);
                    accel = x(2);
                    time = 0;
                    blend_radius = 0;
                else
                    max_speed = x(1);
                    accel = x(2);
                    time = x(3);
                    blend_radius = x(4);
                end
            end
            
            tcp = tcp';
            tcp_bin = ones(1, length(tcp)*8); 
            for i = 1:length(tcp)
                
                buffer = bin2uint8_array(double2bin(tcp(i)));
                tcp_bin( (i*8)-7: i*8) = buffer;
                
            end
            
            
            this.rtde.Flush();
            
            this.rtde.Set_Outputs('timestamp,output_int_register_0,actual_q,actual_qd,target_q,target_qd,target_qdd')
            
            this.rtde.Set_Inputs('input_int_register_0,input_int_register_1,input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5,input_double_register_6,input_double_register_7,input_double_register_8,input_double_register_9')
            
            % Execute Move Program 
            t = tcpclient(this.ip,30002);
            directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
            command = fileread(directory+"/Program_Control_MoveL.script");
            write(t, unicode2native(command,'US-ASCII'));
            clear t;
            
            live = 1;
            watchdog = 1;
            
            this.rtde.Start();
            this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), tcp_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
            
            while true
                watchdog = uint8(watchdog+1);
                
                %verbose
                if false
                    disp(this.rtde.latest{1});
                    disp(this.rtde.latest{2});

                    disp('Actual (Pos, Vel)');
                    disp([this.rtde.latest{3},this.rtde.latest{4}]);

                    disp('Target (Pos, Vel, Acc)');
                    disp([this.rtde.latest{5},this.rtde.latest{6},this.rtde.latest{7}]);
                end
                
                if this.rtde.latest{2} == 1
                   disp("Move Ended") 
                   live = 0;
                   
                   this.rtde.Start();
            this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), tcp_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
                   this.rtde.Pause();
                   break
                end
                
                this.rtde.Start();
            this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), tcp_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
                pause(0.01);
                this.rtde.Collect();
                pause(0.0112);
            end
            
            this.latest_log = this.rtde.log;
        end
        
        
        function Move_Joints(this, joints, x)

            if length(x)<2
                max_speed = 0.5;
                accel = 0.5;
                time = 0;
                blend_radius = 0;
            else
                if length(x) == 2
                    max_speed = x(1);
                    accel = x(2);
                    time = 0;
                    blend_radius = 0;
                else
                    max_speed = x(1);
                    accel = x(2);
                    time = x(3);
                    blend_radius = x(4);
                end
            end  
            
            
            joints = joints';
            joints_bin = ones(1, length(joints)*8); 
            for i = 1:length(joints)
                
                buffer = bin2uint8_array(double2bin(joints(i)));
                joints_bin( (i*8)-7: i*8) = buffer;
                
            end
            
            this.rtde.Flush();
            
            this.rtde.Set_Outputs('timestamp,output_int_register_0,actual_q,actual_qd,target_q,target_qd,target_qdd')
            
            this.rtde.Set_Inputs('input_int_register_0,input_int_register_1,input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5,input_double_register_6,input_double_register_7,input_double_register_8,input_double_register_9')
            
            % Execute Move Program 
            t = tcpclient(this.ip,30002);
            directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
            command = fileread(directory+"/Program_Control_MoveJ.script");
            write(t, unicode2native(command,'US-ASCII'));
            clear t;
            
            live = 1;
            watchdog = 1;
            
            this.rtde.Start();
            this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), joints_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
            
            
            while true
                watchdog = uint8(watchdog+1);
                
                %verbose
                if false
                    disp(this.rtde.latest{1});
                    disp(this.rtde.latest{2});

                    disp('Actual (Pos, Vel)');
                    disp([this.rtde.latest{3},this.rtde.latest{4}]);

                    disp('Target (Pos, Vel, Acc)');
                    disp([this.rtde.latest{5},this.rtde.latest{6},this.rtde.latest{7}]);
                end
                
                if this.rtde.latest{2} == 1
                   disp("Move Ended") 
                   live = 0;
                   
                   this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), joints_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
                   this.rtde.Pause();
                   break
                end
                
                this.rtde.Send_Command('U', [uint32b2uint8(watchdog), uint32b2uint8(live), joints_bin, ...
                       double2uint8(max_speed),double2uint8(accel),double2uint8(time),double2uint8(blend_radius)]);
                pause(0.01);
                this.rtde.Collect();
                pause(0.01);
            end
            
            this.latest_log = this.rtde.log;
        end
        
        function Plot_Log(this)
            
            l = this.latest_log;
            
            t = length(l);
            actual_j = zeros(6, length(l));
            actual_v = zeros(6, length(l));
            target_j = zeros(6, length(l));
            target_v = zeros(6, length(l));
            target_a = zeros(6, length(l));

            for i = 1:length(l)
                log = l{i};
                t(i) = log{1};
                actual_j(:,i) = log{3};
                actual_v(:,i) = log{4};

                target_j(:,i) = log{5};
                target_v(:,i) = log{6};
                target_a(:,i) = log{7};
            end

            t = t-min(t);

            t = seconds(t);

            figure
            subplot(3,1,1)
            plot(t, actual_j',t, target_j', ':k')
            title('Joint Positions')

            subplot(3,1,2)
            plot(t, actual_v',t, target_v', ':k')
            title('Joint Velocities')

            subplot(3,1,3)
            plot(t, target_a')
            title('Joint Accelerations')
            
        end
        
        
        
    end
    
end