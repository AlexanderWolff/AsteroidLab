

classdef URobot
    properties (GetAccess='public', SetAccess='public')
       ip
       
    end
    
    methods
        function changeIP(this)
            this.ip = 10;
        end
    end
    
end