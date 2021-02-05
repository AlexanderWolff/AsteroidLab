

classdef URobot < handle
    properties (GetAccess='public', SetAccess='private')
       
       ip
       rtde
       connected
       sim
       
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
            
            j_pos = this.rtde.robot_packets{end-1}.info{2};
            
        end
        
        function tcp_pose = Get_TCP_Pos(this)
            this.rtde.Set_Outputs('timestamp,actual_TCP_pose');
            this.rtde.Start();
            pause(0.01);
            this.rtde.Pause();
            this.rtde.Collect();
            
            tcp_pose = this.rtde.robot_packets{end-1}.info{2};
        end
        
        
        function Move_TCP(this)

            tcp = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]';
            tcp = tcp';
            
            t = tcpclient(this.ip,30002);
            directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
            command = fileread(directory+"/Program_Control.script");
            write(t, unicode2native(command,'US-ASCII'));
            
            this.rtde.Set_Inputs('input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5');
            
            % TODO : move complete script
            
            %this.rtde.Send_Command('U', tcp);
            
        end
    end
    
end