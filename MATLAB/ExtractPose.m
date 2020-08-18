function [pose] = ExtractPose(data)
    % Converts the char cell array from robot to a struct containing all 
    % the values

    pose = struct;

    for i=1:length(data)
       
        % Separate Header from data
        component = split(data{i},':');
        
        switch component{1}
           
            case 'Base'
                component = component{2}(3:end-1);
                component = split(component,',');
                
                pose.base = zeros(6,1);
                
                for j = 1:6
                   pose.base(j) = str2double(component{j}); 
                end
            
            case 'Actual Joint Position'
                component = component{2}(2:end-1);
                component = split(component,',');
                
                pose.actual_joint_position = zeros(6,1);
                
                for j = 1:6
                   pose.actual_joint_position(j) = str2double(component{j}); 
                end
                
            case 'Tool'
                component = component{2}(3:end-1);
                component = split(component,',');
                
                pose.tool = zeros(6,1);
                
                for j = 1:6
                   pose.tool(j) = str2double(component{j}); 
                end
            
            case 'Actual TCP Pose'
                component = component{2}(3:end-1);
                component = split(component,',');
                
                pose.actual_tcp_pose = zeros(6,1);
                
                for j = 1:6
                   pose.actual_tcp_pose(j) = str2double(component{j}); 
                end
            
            case 'TCP Force'
                component = component{2}(3:end-1);
                component = split(component,',');
                
                pose.tcp_force = zeros(6,1);
                
                for j = 1:6
                   pose.tcp_force(j) = str2double(component{j}); 
                end
            
            otherwise
                break
        end
        
    end
end