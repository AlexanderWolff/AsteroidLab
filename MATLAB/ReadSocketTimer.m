function [] = ReadSocketTimer(socket, characters, verbose )
    % Reads connected socket and appends it to data
    
    global log
    
    if characters <= 0
        output = fgetl(socket)';
    else
        output = fread(socket, characters);
    end
    
    output = native2unicode(output,'UTF-8')';
    log{ length(log)+1 } = output;
    
    if verbose==true
        fprintf('\n[Robot]\t');
        fprintf(output);
        fprintf('\n');
    end
end