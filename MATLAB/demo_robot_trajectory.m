


% connect to robot
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



% 
% pose = round(Solutions(1,:)')
% 
% UR3.Move_Joints(pose,[0.8,1])


% for i = 1:size(Solutions,1)
%    
%     pose = Solutions(i,:)'
%     
%     UR3.Move_Joints(pose,[])
%     break
% end

