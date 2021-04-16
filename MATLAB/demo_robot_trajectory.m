
clc

%% Connect to robot
while true
    try
        try
            if UR3.connected == 0
                UR3.Connect();
            end
        catch
           UR3 = URobot('192.168.0.11');
           UR3.Connect();
        end

        rad2deg(UR3.Get_Joint_Pos())
        UR3.Get_Runtime()
        UR3.Get_ToolVoltage()
        UR3.Get_TCP_Pos()

        UR3.rtde.Get_URControl_Version()

        UR3.rtde.Set_Protocol_Version(1);
        
        break

    catch
        clear UR3
        UR3 = URobot('192.168.0.11');
        UR3.Connect();
        continue
    end
end

pose = [0,-90,0,-90,0,0]';

UR3.Move_Joints(deg2rad(pose),[0.1,0.1,0,0]);

%% Make sure solutions are close to each other
for i = 1:size(Solutions,1)-1
   
    
    for j = 1:6
        
        if abs(Solutions(i,j)-Solutions(i+1,j))>180
            
            Solutions(i+1,j) = Solutions(i+1,j)-360;
            
        end
        
    end
end


%% Set Parameters
v = 0.2;
a = 0.2;
t = 0;
b = 0.45;



epochs = size(Solutions,1);
V = ones(epochs,1)*v;
A = ones(epochs,1)*a;
T = ones(epochs,1)*t;
B = ones(epochs,1);

blend = 0;
for i = 1:size(Solutions,1)
    
    if i == 1
        blend = 0;
    else
        dist = Trajectory{i-1}.distanceTo(Trajectory{i});
        
        blend = dist*b;
    end
    
    B(i) = blend;
end


%% Blended Move

while true
    
    pause(1);
    UR3.Move_Joints(deg2rad(Solutions(1,:))',[0.1,0.1,0,0]);
    pause(1);
    
    UR3.BlendMove(Solutions,{V,A,T,B})
end


% for i = 1:size(Solutions,1)
%    
%     pose = Solutions(i,:)'
%     
%     UR3.Move_Joints(pose,[])
%     break
% end

