function packets = segment_packets(input)

    % offline segmentation function
    % expecting binary format from dec2bin(x,8)
    
    
    % guess starting point
    start_index = 1;

    packets = {};

    while start_index < length(input)

        % First Two Bytes are for counting packet length
        b = input(start_index:start_index+1,:); 
        packet_length = bin2uint16(b);

        % Third Byte is packet type
        b = input(start_index+2,:);
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
                type = NaN;
                start_index = start_index+1;

                % Skip
                continue
        end

        clear packet

        % Remainder Bytes are data
        if ~isnan(type)
            try
                data = input(start_index:start_index+packet_length,:);
            catch
                data = input(start_index:end,:);
                disp('Packet Error: Incomplete data');
            end

            packet.type = type;
            packet.long = long;
            packet.size = packet_length;
            packet.data = data;

        else
            packet.type = NaN;
            packet.long = false;
            packet.size = false;
            packet.data = false;
        end

        % Set new packet starting point
        start_index = start_index+packet.size;

        % Append Packet
        packets{length(packets)+1} = packet;
    end

end