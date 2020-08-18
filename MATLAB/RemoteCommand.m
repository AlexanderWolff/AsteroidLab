function [log, data] = RemoteCommand( socket, log, cmd, verbose )

    data = {};

    try
        try
            try
                cmd_option{1} = cmd{2};
                cmd_option{2} = cmd{3};
                cmd_option{3} = cmd{4};
                cmd_option{4} = cmd{5};
            catch
                cmd_option{1} = cmd{2};
                cmd_option{2} = cmd{3};
                cmd_option{3} = cmd{4};
            end
        catch
            cmd_option = cmd{2};
        end
        
        cmd = upper(cmd{1}(1:4));
    catch
        cmd = upper(cmd(1:4));
    end
    
    if verbose
        fprintf(strcat('\nCommand :\t', cmd, '\n'))
    end
    
    switch cmd
        
        case 'HALT'
            % Stops Robot operation
            WriteSocket(socket, strcat('<',cmd,'>') );
            
            % Expecting 'Halting operation.'
            [output, log] = ReadSocket(socket, log, 18, verbose);

            % Expecting 'Robot A: Halted.'
            [output, log] = ReadSocket(socket, log, 16, verbose);
            
        case 'INST'
            % Writes instructional popup on robot interface
            WriteSocket(socket, strcat('<',cmd,'>') );
            WriteSocket(socket, strcat('[',cmd_option,']') );
            
            % Expecting message echo
            [output, log] = ReadSocket(socket, log, length(cmd_option), verbose);
            
            % Expecting 'Acknowledged.'
            [output, log] = ReadSocket(socket, log, 13, verbose);
        
        case 'POSE'
            % Asks for the current pose of the robot
            WriteSocket(socket, strcat('<',cmd,'>') );
            
            for i = 1:7
                if i == 1 || i == 7
                    [output, log] = ReadSocket(socket, log, 0, verbose);
                else 
                    [output, log] = ReadSocket(socket, log, 0, false);
                    data{ length(data)+1 } = output;
                end
            end
            data=data';
            data = ExtractPose(data);
        
        case 'MOVE'
            % Moves robot to the requested pose
            WriteSocket(socket, strcat('<',cmd,'>') );
            
            switch upper(cmd_option{1})
            
                case "J"
                
                    WriteSocket(socket, strcat('<','J','>') );

                    WriteSocket(socket, strcat('(',FlattenFloats(cmd_option{2}),',',FlattenFloats(cmd_option{3}'),')') );

                    % Expecting 'Starting movej.'
                    [output, log] = ReadSocket(socket, log, 15, verbose);
                    
                case "L"
                
                    WriteSocket(socket, strcat('<','L','>') );

                    WriteSocket(socket, strcat('(',FlattenFloats(cmd_option{2}),',',FlattenFloats(cmd_option{3}'),')') );

                    % Expecting 'Starting movel.'
                    [output, log] = ReadSocket(socket, log, 15, verbose);
                    
                case "C"
                
                    WriteSocket(socket, strcat('<','C','>') );

                    WriteSocket(socket, strcat('(',FlattenFloats(cmd_option{2}),',',FlattenFloats(cmd_option{3}'),',',FlattenFloats(cmd_option{4}),')') );

                    % Expecting 'Starting movec.'
                    [output, log] = ReadSocket(socket, log, 15, verbose);
                otherwise
                % Nothing
            end
            
            % Expecting 'Move completed.'
            [output, log] = ReadSocket(socket, log, 15, verbose);
            
        case 'WAIT'
            % Waits for the requested amount of time
            WriteSocket(socket, strcat('<',cmd,'>') );
            
            WriteSocket(socket, strcat('(',num2str(cmd_option),')') );
            
            % Expecting 'Waiting start.'
            [output, log] = ReadSocket(socket, log, 14, verbose);
            
            % Expecting 'Waiting end.'
            t = timer;
            t.StartDelay = cmd_option;
            t.TimerFcn = @(myTimerObj, thisEvent)ReadSocketTimer(socket, 12, verbose);
            start(t)

            
            
        otherwise
            fprintf("\n\nError: Unknown Command.\n\n");
    end


end


