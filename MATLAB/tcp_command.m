

if t.Port ~= 30002
    t = tcpclient('192.168.0.11',30002);
end

directory = "/home/wolff/AsteroidLab/AsteroidLab/MATLAB";
template = fileread(directory+"/Program_Template.script");
%command = fileread(directory+"/Robot_Dance.script");

header = template(1:501);
footer = template(562:end);


%command = 'popup("This is a test", "Title", False, False, blocking=True)';
command = 'get_actual_joint_positions()';

output = sprintf("%s\n\t%s\n%s", header,command,footer);

write(t, unicode2native(output,'US-ASCII'));

