%% Find Workspace
function [X,Y,Z,XYZ] = findWorkspace(sim)
    sim = this;

    angles = linspace(-180,180,9);

    l = (length(angles)^6)+1;
    DX = zeros(1,l);
    DY = zeros(1,l);
    DZ = zeros(1,l);
    DXYZ = zeros(1,l);
    i = 1;
    checkpoint = 0.01;

    for a = 1:length(angles)

        for b = 1:length(angles)

            for c = 1:length(angles)

                for d = 1:length(angles)

                    for e = 1:length(angles)

                        for f = 1:length(angles)

                            A = angles(a);
                            B = angles(b);
                            C = angles(c);
                            D = angles(d);
                            E = angles(e);
                            F = angles(f);

                            sim.ForwardKinematics([A,B,C,D,E,F]);

                            %% Distance between tool and base
                            F1 = sim.transform.local{1};
                            F7 = sim.transform.local{7};
                            Ftool_F7 = sim.transform.tool_offset;
                            Ftool = Ftool_F7.transform(F7);

                            Base = F1.T;
                            Tool = Ftool.T;

                            dx = Base(1)-Tool(1);
                            dy = Base(2)-Tool(2);
                            dz = Base(3)-Tool(3);

                            dxyz = sqrt(dx^2+dy^2+dz^2);

                            if(i/l>checkpoint)
                                disp( strcat(string(round(i/l*100)),'%') );
                                checkpoint = checkpoint + 0.01;
                            end

                            if(i>l)
                               disp([a,b,c,d,e,f]);
                               disp('error') 
                            end

                            DX(i) = dx;
                            DY(i) = dy;
                            DZ(i) = dz;
                            DXYZ(i) = dxyz;
                            i = i+1;
                        end
                    end
                end
            end
        end
    end

    X = [min(DX),max(DX)];
    Y = [min(DY),max(DY)];
    Z = [min(DZ),max(DZ)];
    XYZ = [min(DXYZ),max(DXYZ)];
    
end





