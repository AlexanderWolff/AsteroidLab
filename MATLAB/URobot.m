

classdef URobot < handle
    properties (GetAccess='public', SetAccess='private')
       
       ip
       rtde
       connected
       sim
       
    end
    
    properties (Constant)
        ur3 = '192.168.0.11';
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
        
        function Move_TCP(this)

            tcp = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]';
            tcp = tcp';
            
            tcp_bin = ones(1, length(tcp)*8); 
            for i = 1:length(tcp)
                
                buffer = bin2uint8_array(double2bin(tcp(i)));
                tcp_bin( (i*8)-7: i*8) = buffer;
                
            end
            
            this.rtde.Set_Outputs('timestamp,output_int_register_0')
            
            this.rtde.Set_Inputs('input_int_register_0,input_int_register_1,input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5')
            
            this.rtde.Start();
            
            % Execute Move Program 
            t = tcpclient(this.ip,30002);
            directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
            command = fileread(directory+"/Program_Control_MoveL.script");
            write(t, unicode2native(command,'US-ASCII'));
            
            
            live = 1;
            watchdog = 1;
            while true
                watchdog = uint8(watchdog+1);
                this.rtde.Collect();
                
                if this.rtde.latest{2} == 1
                   disp("Move Ended") 
                   live = 0;
                   
                   this.rtde.Send_Command('U', [bin2uint8_array(dec2bin(watchdog,32)), bin2uint8_array(dec2bin(live,32)), tcp_bin]);
                   this.rtde.Pause();
                   break
                end
                
                this.rtde.Send_Command('U', [bin2uint8_array(dec2bin(watchdog,32)), bin2uint8_array(dec2bin(live,32)), tcp_bin]);
                pause(0.5);
            end
        end
        
        function Move_Joints(this, joints)

            tcp = joints';
            
            tcp_bin = ones(1, length(tcp)*8); 
            for i = 1:length(tcp)
                
                buffer = bin2uint8_array(double2bin(tcp(i)));
                tcp_bin( (i*8)-7: i*8) = buffer;
                
            end
            
            this.rtde.Set_Outputs('timestamp,output_int_register_0')
            
            this.rtde.Set_Inputs('input_int_register_0,input_int_register_1,input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5')
            
            this.rtde.Start();
            
            % Execute Move Program 
            t = tcpclient(this.ip,30002);
            directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
            command = fileread(directory+"/Program_Control_MoveJ.script");
            write(t, unicode2native(command,'US-ASCII'));
            
            
            live = 1;
            watchdog = 1;
            while true
                watchdog = uint8(watchdog+1);
                this.rtde.Collect();
                
                if this.rtde.latest{2} == 1
                   disp("Move Ended") 
                   live = 0;
                   
                   this.rtde.Send_Command('U', [bin2uint8_array(dec2bin(watchdog,32)), bin2uint8_array(dec2bin(live,32)), tcp_bin]);
                   this.rtde.Pause();
                   break
                end
                
                this.rtde.Send_Command('U', [bin2uint8_array(dec2bin(watchdog,32)), bin2uint8_array(dec2bin(live,32)), tcp_bin]);
                pause(0.5);
            end
        end
    end
    
end