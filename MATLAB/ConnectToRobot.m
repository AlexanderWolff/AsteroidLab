
% Robot IP addresses
robot_IP = {'192.168.1.248', '192.168.1.156'};
port = [7000, 7001];

for i = 1:2 
    socket = tcpip(robot_IP{i}, port(i), 'NetworkRole', 'server');
    s{i} = socket;
    fclose(s{i});
    disp('Press PLAY button on Robot '+string(i)+'');
    fopen(s{i});
    s{i}
    disp('Connected to Robot '+string(i)+' !');
    
    global log
    log = {'Log Start'};
    % Expecting 'Robot X: Connected!'
    [output, log] = ReadSocket(s{i}, log, 19, true);
    log = log';
    break %% only connecting with first robot
end

S = s{i}; %% placeholder for developing


