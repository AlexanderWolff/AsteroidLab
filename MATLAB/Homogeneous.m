classdef Homogeneous < handle 
    properties (GetAccess='public', SetAccess='public')
       
       H
       T
       R
       
    end
    
    properties (GetAccess='private', SetAccess='private')
       
       
        
    end
    
    methods (Static)
        function this = Empty()
           this = Homogeneous(eye(4)); 
        end
        
        function this = fromT(T)
            % From a Translation (Vector 1x3)
            this = Homogeneous.Empty;
            this.setT(T);
        end
        
        function this = fromR(R)
            % From a Rotation (Matrix 3x3)
            this = Homogeneous.Empty;
            this.setR(R);
        end
        
        function this = fromDH_B(DH_param)
           % From Denavit-Hartenberg (back comparison)
           % [theta, a(-1), d, alpha(-1),]
           
           theta = DH_param(1);
           a = DH_param(2);
           d = DH_param(3);
           alpha = DH_param(4);
           
           a = abs(a);
           
           ct = cos(theta);
           st = sin(theta);
           ca = cos(alpha);
           sa = sin(alpha);
           
           T = [ct,      -st,   0,     a;...
                st*ca, ct*ca, -sa, -sa*d;...
                st*sa, ct*sa,  ca,  ca*d;...
                    0,     0,   0,     1];
                
           this = Homogeneous(T);
        end
        
        function this = fromDH_F(DH_param)
           % From Denavit-Hartenberg (forward comparison)
           % [theta, a(+1), d, alpha(+1),]
           
           theta = DH_param(1);
           a = DH_param(2);
           d = DH_param(3);
           alpha = DH_param(4);
           
           %a = abs(a);
           
           ct = cos(theta);
           st = sin(theta);
           ca = cos(alpha);
           sa = sin(alpha);
           
           T = [ct, -st*ca,  st*sa, a*ct;...
                st,  ct*ca, -ct*sa, a*st;...
                 0,     sa,     ca,    d;...
                 0,      0,      0,    1];
                
           this = Homogeneous(T);
        end
        
        function product = RotateThrough(R,P)
           % Rotate this frame by R through point P
           % R (Matrix 3x3), P (Vector 1x3)
           
           % Translate
           T_ab = Homogeneous.fromT(P);
           
           % Rotate
           T_bc = Homogeneous.fromR(R);
           
           % Translate
           T_cd = Homogeneous.fromT(-1*P);
           
           % Transform
           T_ad = T_ab.H*T_bc.H*T_cd.H;
           
           product = Homogeneous(T_ad);
        end
        
    end
    
    
    methods (Access = 'public')
        function this = Homogeneous(value)
            this.H = value;
            this.R = value(1:3,1:3);
            this.T = value(1:3,4);
        end
        
        function inverse = inv(this)
            % Transform Inverse
            R_ = this.R';
            T_ = -1.*(this.R')*this.T;
            inverse = [[R_;0,0,0],[T_;1]];
            inverse = Homogeneous(inverse);
        end
        
        function product = p(this,point)
            % Transforms Point (Vector 1x3)
            product = (this.R*point) + this.T; 
        end
        
        function this = setT(this,T)
            % Sets new Translatio parameter
           this.T = T;
           this.H(1:3,4) = T;
        end
        
        function this = setR(this,R)
            % Sets new Rotation parameter
           this.R = R;
           this.H(1:3,1:3) = R;
        end
        
        function new = dot(this,other)
            % Multiplies two Homogeneous Transforms
            
            % TODO: optimize by using .R and .T
            new = Homogeneous(this.H*other.H);
        end
        
        function new = transform(this,other)
            % Transform this Frame by Homogeneous Transform
            
            % TODO: optimize by using .R and .T
            new = Homogeneous(other.H*this.H);
        end
        
        function plot(this)
            
            scale = 0.1;
            
            O = Homogeneous.Empty;
            X = Homogeneous.fromT([scale,0,0]);
            Y = Homogeneous.fromT([0,scale,0]);
            Z = Homogeneous.fromT([0,0,scale]);
            
            %Transform
            O = O.transform(this);
            X = X.transform(this);
            Y = Y.transform(this);
            Z = Z.transform(this);

            %Plot
            p = [O.T, X.T]';
            x1 = p(:,1);
            y1 = p(:,2);
            z1 = p(:,3);
            
            p = [O.T, Y.T]';
            x2 = p(:,1);
            y2 = p(:,2);
            z2 = p(:,3);
            
            p = [O.T, Z.T]';
            x3 = p(:,1);
            y3 = p(:,2);
            z3 = p(:,3);
            plot3(x1,y1,z1,'-b',x2,y2,z2,'-g',x3,y3,z3,'-r');
            
            xlim([-1,1]);
            axis equal;
        end
        
        function plotk(this)
            
            scale = 0.1;
            
            O = Homogeneous.Empty;
            X = Homogeneous.fromT([scale,0,0]);
            Y = Homogeneous.fromT([0,scale,0]);
            Z = Homogeneous.fromT([0,0,scale]);
            
            %Transform
            O = O.transform(this);
            X = X.transform(this);
            Y = Y.transform(this);
            Z = Z.transform(this);

            %Plot
            p = [O.T, X.T]';
            x1 = p(:,1);
            y1 = p(:,2);
            z1 = p(:,3);
            
            p = [O.T, Y.T]';
            x2 = p(:,1);
            y2 = p(:,2);
            z2 = p(:,3);
            
            p = [O.T, Z.T]';
            x3 = p(:,1);
            y3 = p(:,2);
            z3 = p(:,3);
            plot3(x1,y1,z1,'-k',x2,y2,z2,'-k',x3,y3,z3,'-k');
            
            xlim([-1,1]);
            axis equal;
        end
        
        function plotL(this,label)
           this.plot;
           text(this.T(1),this.T(2),this.T(3),label);
        end
        
    end
    
end