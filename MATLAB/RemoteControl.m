% The aim of this script is to control the UR robots directly using TCP/IP
% protocol, ie. the ability to send script directly to the robots
% Currently not operational

t=tcpclient('192.168.1.248', 30003)
 
%write(t, uint8('set_digital_out(1,True)\n'))

%cmd = 'set_digital_out(1,True)\r\n';
cmd = 'popup("test")\r\n';
cmd = unicode2native(cmd,'US-ASCII');
write(t, cmd)

while true
    if t.BytesAvailable>0
        data = read(t);
        native2unicode(data,'UTF-8')
        break
    end
end

