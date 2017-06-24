 function [x,P] = append(x,P,u,idx,R,pos)
            % Check if landmark is new
            numOfLandmarks = (length(x) - 3) / 2;
            if(numOfLandmarks < idx)
   
                n = length(P);
                
                % Append landmark to x
                x = [x , pos(1), pos(2)];
                
                % Get SLAM-Specific Jacobians (as defined by SLAM for Dummies!)
                jxr = [1 0 -u(1)*sind(x(3)); ...
                    0 1  u(1)*cosd(x(3))];
                
                jz = [cosd(u(2)) -u(1)*sind(u(2)); ...
                    sind(u(2)) u(1)*cosd(u(2))];
                
                % Append landmark to P (again as defined by SLAM for Dummies)
                P(n+1:n+2,n+1:n+2) = jxr*P(1:3,1:3)*jxr' + jz*R*jz';    %C
                P(1:3,n+1:n+2) = P(1:3,1:3)*jxr';                       %I
                P(n+1:n+2,1:3) = P(1:3,n+1:n+2)';                       %H
                for idx = 1: numOfLandmarks
                    P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5)) = jxr*P(((idx-1)*2+4):((idx-1)*2+5),1:3)';   %F
                    P(((idx-1)*2+4):((idx-1)*2+5),(n+1):(n+2)) = P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5))';%G
                end
            end
        end