
% Robot IP addresses
robot_IP = {'192.168.0.11', '192.168.0.12'};
port = [30004, 7001]; %[7000, 7001];

for i = 1:2 
    socket = tcpip(robot_IP{i}, port(i), 'NetworkRole', 'client');
    s{i} = socket;
    fclose(s{i});
    s{i}
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

S = s{1}; %% placeholder for developing


