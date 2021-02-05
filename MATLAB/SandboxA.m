%t = tcpclient('192.168.0.11',30002);

%directory = "/home/wolff/Documents/ASTEROIDLAB/UR5RobotControlV3.2";
%command = fileread(directory+"/Program_Template.script");
%command = fileread(directory+"/Robot_Dance.script");

t = tcpclient('192.168.0.11',30002);
directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
command = fileread(directory+"/Program_Control.script");
write(t, unicode2native(command,'US-ASCII'));

% %header = command(1:486);
% %footer = command(548:end);
% 
% %'get_actual_joint_positions()'
% 
% %unicode2native(command,'US-ASCII')
% write(t, unicode2native(command,'US-ASCII'));

% RTDE Commands 
% 86  V RTDE_REQUEST_PROTOCOL_VERSION
% 118 v RTDE_GET_URCONTROL_VERSION
% 77  M RTDE_TEXT_MESSAGE
% 0-exception, 1-error, 2-warning, 3-info 
%
% 85  U RTDE_DATA_PACKAGE
% 79  O RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
% 73  I RTDE_CONTROL_PACKAGE_SETUP_INPUTS
% 83  S RTDE_CONTROL_PACKAGE_START
% 80  P RTDE_CONTROL_PACKAGE_PAUSE
% 
% 
% if t.Port ~= 30004
%     t = tcpclient('192.168.0.11',30004);
% end
% 
% command = 'P';
% binary_command = unicode2native(command,'US-ASCII');
% 
% 
% % Message (command 'M')
% % binary_payload = [12, str2message('message test'), 5, str2message('error'), 2];
% 
% % timestamp (double 64bit), actual_current (VECTOR6D 6x64bit)
% %binary_payload = str2message('timestamp');
% 
% binary_payload = str2message('');
% 
% 
% % size of packet: 2-byte length + 1-byte command + payload bytes
% size = 2 + 1 + length(binary_payload);
% 
% % convert to binary->unsigned 16 bit
% binary_packet_size = dec2bin(uint16(size));
% len = length(binary_packet_size);
% if len<16
%     for i = 1:16-len
%        binary_packet_size = strcat('0', binary_packet_size);
%     end
% end
% binary_packet_size = [bin2dec(binary_packet_size(1:8)), bin2dec(binary_packet_size(9:16))];
% 
% % size uint16 (2 byte unsigned int) 
% packet = [binary_packet_size, binary_command, binary_payload];
% 
% write(t, packet);
% t
% 
% if t.BytesAvailable>0
%     incoming_packet = read(t, t.BytesAvailable);
% 
%     package_length = bin2dec(strcat(dec2bin(incoming_packet(1)), dec2bin(incoming_packet(2))));
%     type = incoming_packet(3);
%     data = incoming_packet(4:end);
%     message = strings(1,length(data)-3);
% 
%     for i = 1:length(data)
%         if data(i)>=32
%             message(i) = native2unicode(data(i),'UTF-8') ;
%         else
%             message(i) = native2unicode(data(i)+48,'UTF-8');
%         end
%     end
%     message = join(message,'');
% 
%     display(message);
% end
% 
% % bin_data = dec2bin(data,8);
% % stamp = reshape(dec2bin(pack(4:end))',1,[])

% TODO write message intercepting code



% UR3 = URobot_Protocol();
% UR3 = UR3.Connect('192.168.0.11',30004);

% UR3.Send_Command('O', 'timestamp,actual_q');
% pause(0.5);
% UR3 = UR3.Collect();
% UR3 = UR3.Translate_Packet();
% UR3.robot_packets{end}
% 
% UR3.Send_Command('S', '');
% pause(1);
% UR3.Send_Command('P', '');
% pause(0.5);
% UR3 = UR3.Collect();
% UR3 = UR3.Translate_Packet();
% UR3.robot_packets{end-1}.info{2}
% 
% UR3.Send_Command('V', 1);
% 
% UR3.Send_Command('I', 'input_double_register_20');
% pause(0.5);
% UR3 = UR3.Collect();
% UR3 = UR3.Translate_Packet();
% UR3.robot_packets{end}






































