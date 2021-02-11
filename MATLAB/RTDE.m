

classdef RTDE < handle
    properties (GetAccess='public', SetAccess='private')
       ip
       port
       tcp
       
       % Stores Bytes from Robot
       buffer_stack
       
       robot_packets
       
       output_format
       
       input_format
       
       inputs
       
       recipe_id
       
       log
       
       latest
       
    end
    
    methods
        
        function this = RTDE(ip,port)
            this.ip = ip;
            this.port = port;
        end
        
        function this = Connect(this)
            % Create TCP Client for RTDE 
            this.tcp = tcpclient(this.ip,this.port);
            
            this.buffer_stack = [];
            this.robot_packets = {};
            this.output_format = {};
            this.input_format = {};
            this.recipe_id = 0;
            this.log = {};
            this.latest = {};
            this.inputs = '';
        end
        
%         function stack = Append(this,stack,data)
%             stack{length(stack)+1} = data;
%         end
%         
        function this = Collect(this)
            % Collect Packets from URobot
            if this.tcp.BytesAvailable > 0
                
                incoming = read(this.tcp, ...
                                this.tcp.BytesAvailable);
                            
                this.buffer_stack = [this.buffer_stack, incoming];
            end
            
            this.Segment_Packets();
        end
        
        function this = Segment_Packets(this)

            % offline segmentation function
            % expecting binary format from dec2bin(x,8)
            input = dec2bin(this.buffer_stack,8);

            
            while 1 < length(input)

                % First Two Bytes are for counting packet length
                b = input(1:2,:); 
                packet_length = bin2uint16(b);

                % Third Byte is packet type
                b = input(3,:);
                b = bin2uint8(b);
                type = native2unicode(b,'UTF-8');

                % Check type validity
                switch type

                    case 'V' % Request Protocol Version
                        long = 'Request Protocol Version';
                    case 'v' % Get URControl Version
                        long = 'Get URControl Version';
                    case 'M' % Text Message
                        long = 'Text Message';
                    case 'U' % Data Package
                        long = 'Data Package';
                    case 'O' % Control Package Setup Outputs
                        long = 'Control Package Setup Outputs';
                    case 'I' % Control Package Setup Inputs
                        long = 'Control Package Setup Inputs';
                    case 'S' % Control Package Start
                        long = 'Control Package Start';
                    case 'P' % Control Package Pause
                        long = 'Control Package Pause';
                    otherwise
                        
                        % Remove Byte from Stack
                        input(1,:) = [];
                        this.buffer_stack(1) = [];

                        % Skip
                        continue
                end

                clear packet

                % Remainder Bytes are data
                if ~isnan(type)
                    try
                        data = input(4:packet_length,:);
                    catch
                        disp('Packet Error: Incomplete data');
                        break
                    end

                    packet.type = type;
                    packet.long = long;
                    packet.size = packet_length;
                    packet.data = data;
                    packet.info = '';
                    packet.text = '';

                else
                    packet.type = NaN;
                    packet.long = false;
                    packet.size = false;
                    packet.data = false;
                    packet.info = '';
                    packet.text = '';
                end

                
                % Remove Packet from Stack
                input(1:packet.size, :) = [];
                this.buffer_stack(1:packet.size) = [];
                
                % Append Packet
                this.robot_packets{length(this.robot_packets)+1} = this.Translate_Packet(packet);
                
                
            end

        end
        
        function packet = Translate_Packet(this,packet)
            
            packet.info = '';

            if isempty(packet.info)

                data = packet.data;

                switch packet.type
                    case 'V' % Request Protocol Version
                        % data : accepted (uint8)
                        % accepted = 1 (success) or 0 (failed)

                        if size(data,1)~=1
                            info = NaN;
                            text = 'Data Error: Incorrect Format.';
                        else

                            accepted = bin2dec(data);

                            if accepted == 1
                                info = 1;
                                text = 'Protocol Version sucessfully set';
                            elseif accepted == 0
                                info = 0;
                                text = 'Protocol Version not set';
                            end 
                        end


                    case 'v' % Get URControl Version
                        if size(data,1)~=16
                            info = [NaN, NaN, NaN, NaN];
                            text = 'Data Error: Incorrect Format.';
                        else

                            % data : major (uin32), minor (uint32),
                            % bugfix (uint32), build (uint32)
                            major = bin2dec(reshape(data(1:4,:)',1,[]));
                            minor = bin2dec(reshape(data(5:8,:)',1,[]));
                            bugfix = bin2dec(reshape(data(9:12,:)',1,[]));
                            build = bin2dec(reshape(data(13:16,:)',1,[]));

                            info = [major, minor, bugfix, build];
                            text = sprintf("Major.Minor.Bugfix.Build : %d.%d.%d.%d", ...
                                major, minor, bugfix, build);
                        end

                    case 'M' % Text Message
                        % message type (uint8), message (char array)
                        message_type = data(1,:);
                        message = native2unicode(bin2dec(data(2:end,:)))';

                        switch message_type
                            case 0
                                warning_level = 'Exception';
                            case 1
                                warning_level = 'Error';
                            case 2
                                warning_level = 'Warning';
                            case 3
                                warning_level = 'Info';
                            otherwise
                                warning_level = '?';
                        end

                        info = message_type;
                        text = sprintf("%s:\t%s",warning_level,message);

                    case 'U' % Data Package

                        text = sprintf("Recieved %d Bytes",size(data,1));

                        info = Convert_Output(this, data);

                        this.log{length(this.log)+1} = info;
                        
                        info{1} = seconds(info{1});
                        
                        this.latest = info;

                    case 'O' % Control Package Setup Outputs
                        % data : variable types (char string)
                        text = native2unicode(bin2dec(data)');
                        info = split(text, ',');
                        this.output_format = info;


                        text = sprintf("Output Format: %s",text);

                    case 'I' % Control Package Setup Inputs

                        recipe = bin2dec(data(1,:));

                        text = sprintf("Recipe %i: %s",recipe,native2unicode(bin2dec(data(2:end,:))'));

                        info = {recipe,split(native2unicode(bin2dec(data(2:end,:))'), ',')};

                        text = sprintf("Input Format: %s",text);

                    case 'S' % Control Package Start
                        % data : accepted (uint8) 1 or 0
                        accepted = bin2dec(data);

                        if accepted == 1
                            info = 1;
                            text = 'Start Success!';
                        elseif accepted == 0
                            info = 0;
                            text = 'Start Failure!';
                        end 

                    case 'P' % Control Package Pause
                        % data : accepted (uint8) 1 or 0
                        accepted = bin2dec(data);

                        if accepted == 1
                            info = 1;
                            text = 'Pause Success!';
                        elseif accepted == 0
                            info = 0;
                            text = 'Pause Failure!';
                        end 

                    otherwise
                          info = '';
                          text = '';
                end
                packet.info = info;
                packet.text = text;
            end
            
        end
        
        
        function info = Convert_Output(this, data) 
            
            
            info = {};
            for i = 1:length(this.output_format)
            
               switch this.output_format{i}
                   
                   case 'UINT32'
                       % 32 bits = 4 bytes
                       d = data(1:4,:);
                       
                       % convert to binary
                       d = reshape(d',1,[]);
                       
                       % convert to decimal
                       info{i} = bin2dec(d);
                       
                       % remove from stack
                       data(1:4,:) = [];
                       
                   case 'INT32'
                       % TODO : conversion with 2 complement
                       % right now is just uint32
                       % 32 bits = 4 bytes
                       d = data(1:4,:);
                       
                       % convert to binary
                       d = reshape(d',1,[]);
                       
                       % convert to decimal
                       info{i} = bin2dec(d);
                       
                       % remove from stack
                       data(1:4,:) = [];
                   
                   case 'DOUBLE'
                       % 64 bits = 8 bytes
                       d = data(1:8,:);
                       
                       % convert to binary
                       d = reshape(d',1,[]);
                       
                       % convert to double
                       info{i} = bin2double(d);
                       
                       % remove from stack
                       data(1:8,:) = [];
                       
                   case 'VECTOR3D'
                       
                       % Vector of 3 doubles
                       buffer = [1,1,1];
                       for j = 1:3
                           % 64 bits = 8 bytes
                           d = data(1:8,:);

                           % convert to binary
                           d = reshape(d',1,[]);

                           % convert to double
                           buffer(j) = bin2double(d);

                           % remove from stack
                           data(1:8,:) = [];
                       end
                       
                       info{i} = buffer';
                       
                   case 'VECTOR6D'
                       
                       % Vector of 6 doubles
                       buffer = [1,1,1,1,1,1];
                       for j = 1:6
                           % 64 bits = 8 bytes
                           d = data(1:8,:);

                           % convert to binary
                           d = reshape(d',1,[]);

                           % convert to double
                           buffer(j) = bin2double(d);

                           % remove from stack
                           data(1:8,:) = [];
                       end
                       
                       info{i} = buffer';
                  
                   otherwise
                       info{i} = NaN;
               end
            end
        end
        
        function Start(this)
            this.Send_Command('S', '');
            this.log = [];
        end
        
        function Pause(this)
            this.Send_Command('P', '');
        end
        
        function [text, info] = Get_URControl_Version(this)
            this.Send_Command('v', '');
            pause(0.01);
            this.Collect();
            text = this.robot_packets{end}.text;
            info = this.robot_packets{end}.info;
        end
        
        function [text, info] = Set_Protocol_Version(this, version)
            this.Send_Command('V', version);
            pause(0.01);
            this.Collect();
            text = this.robot_packets{end}.text;
            info = this.robot_packets{end}.info;
        end
        
        function Log_Message(this,level, source, message)
            %warning level : 
            %exception (1), error (2), warning (3), info (4)
            this.Send_Command('M', {message, source, level});
        end
        
        function [text, info] = Set_Outputs(this,outputs)
            % Configure Outputs
            this.Send_Command('O', outputs);
            pause(0.01);
            this.Collect();
            text = this.robot_packets{end}.text;
            info = this.robot_packets{end}.info;
            this.latest = cell(1,length(split(outputs, ',')));
        end
        
        function [text, info] = Set_Inputs(this,inputs)
            
            % Check if inputs are identical to this.inputs
            if strcmp(this.inputs,inputs)
                text = sprintf("Recipe %i: %s",this.recipe_id, strjoin(this.input_format,','));
                info = this.input_format;
                return
            end
            
            % Configure Inputs
            this.Send_Command('I', inputs);
            while true
                try
                    pause(0.01);
                    this.Collect();
                    text = this.robot_packets{end}.text;
                    info = this.robot_packets{end}.info;
                    
                    this.input_format = info{2};
                    this.recipe_id = info{1};
                    this.inputs = inputs;
                    return
                catch
                end
            end
        end
        
        function Flush(this)
           
            while this.tcp.BytesAvailable>0
                read(this.tcp, this.tcp.BytesAvailable);
            end
            this.latest = cell(length(this.output_format),1);
            
        end
        
        function this = Send_Command(this, command, payload)

            binary_command = unicode2native(command,'US-ASCII');
            
            switch command

                case 'V' % Request Protocol Version 
                    % Payload : protocol version (uin16) 
                    b = dec2bin(payload,16);
                    binary_payload = [bin2dec(b(1:8)), bin2dec(b(9:16))];
                    
                case 'v' % Get URControl Version
                    % Payload : empty
                    binary_payload = [];
                    
                case 'M' % Text Message
                    % Payload : message length (uint8), message (char
                    % string), source length (uint8), source (char string),
                    % warning level (uint8) : exception (1), error (2), 
                    % warning (3), info (4)
                    message_len = length(payload{1});
                    source_len = length(payload{2});
                    binary_payload = [message_len, unicode2native(payload{1},'US-ASCII'), ...
                                      source_len, unicode2native(payload{2},'US-ASCII'), ...
                                      payload{3}];
                    
                case 'U' % Data Package
                    % Payload : recipe id (uint8), data
                    binary_payload = [uint8(this.recipe_id), payload];
                    
                case 'O' % Control Package Setup Outputs
                    % Payload : variable names (char string)
                    binary_payload = unicode2native(payload,'US-ASCII');
                    
                case 'I' % Control Package Setup Inputs
                    % Payload : variable names (char string)
                    binary_payload = unicode2native(payload,'US-ASCII');
                    
                case 'S' % Control Package Start
                    % Payload : empty
                    binary_payload = [];
                    
                case 'P' % Control Package Pause
                    % Payload : empty
                    binary_payload = [];
                    
                otherwise
                    disp('Command Error: Incorrect command');
                    return
            end
            
            % size of packet: 2-byte length + 1-byte command + payload bytes
            size = 2 + 1 + length(binary_payload);

            % convert to binary->unsigned 16 bit
            binary_packet_size = dec2bin(uint16(size));
            len = length(binary_packet_size);
            if len<16
                for i = 1:16-len
                   binary_packet_size = strcat('0', binary_packet_size);
                end
            end
            binary_packet_size = [bin2dec(binary_packet_size(1:8)), bin2dec(binary_packet_size(9:16))];

            % size uint16 (2 byte unsigned int) 
            packet = [binary_packet_size, binary_command, binary_payload];

            write(this.tcp, packet);
        end
    end
end