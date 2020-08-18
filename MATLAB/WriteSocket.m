function [] = WriteSocket( socket, instruction )
    % Writes instruction to connected socket

    instruction = unicode2native(instruction,'US-ASCII');
    fwrite(socket, instruction);
end