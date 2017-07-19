classdef EKF_SLAM < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x;  % State vector
        P;  % Covariance matrix
        u;  % Control vector
        Q;  % Process noise covariance matrix
        s;  % Landmark signature vector
        
        % EKF Parameter Values
        C = 0.2;    % Process Noise Constant
        Rc = [0.01,5];   % Measurement Noise Constants
        s_cost = 0.00000000001;
        % Landmark append threshold
        s_thresh = 1000000000;
        
        % Landmark Object
        LM;
        
        % Control and Measurement Subscribers
        contSub;
        measSub;
        
        % Last Control and Measurement Data
        contData;
        measData;
        
        % Last pose from odometry and observed landmarks
        oldOdomPose;
        observed;
    end
    
    methods
        % Constructor - initialize state variables and covariances
        function h = EKF_SLAM(contTopic, measTopic, landmarkMethod)
            % State Vector
            h.x = [0,0,0];
            % Covariance Matrix
            h.P = eye(length(h.x)).*0.1;
            h.P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
            % Initial control vector
            h.u=[0;0;0];
            % Cretae landmark recognition object (e.g., 'RANSAC')
            h.LM = Landmark(landmarkMethod);
            
            %initialize a subscriber node to turtlebot odometry data
            % Example: '/odom' for odometry data
            h.contSub = rossubscriber(contTopic);
            
            %initialize a subscriber node to kinect laser scan data
            % Example: '/scan' for laser scan data
            h.measSub = rossubscriber('/scan'); 
        end
        
        % Prediction Phase of EKF
        % Pass current state vector, covariance matrix, control vector, and
        % prediction noise covariance matrix. Returns predicted state
        % vector and covariance matrix.
        function predict(h,u)
            % Get noise covariance matrix for control signal
            W = [u(1)*cosd(h.x(3)) u(1)*sind(h.x(3)) u(2)]';
            h.Q = zeros(size(h.P));
            h.Q(1:3,1:3) = W*h.C*W';
            
            [h.x,F] = h.f(h.x,u);
            h.P = F*h.P*F' + h.Q;
            
            % Safety first! Ensure orientation doesn't pass 360:
            h.x(3) = wrapTo360(h.x(3));
        end
        
        % Non-linear prediction model
        % Pass current state vector and control vector. Returns predicted
        % state vector and Jacobian of prediction model.
        function [x_new,F] = f(h,x,u)
            x_new = x;
            x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
                x(2) + u(1)*sind(x(3)+u(2)); ...
                x(3) + u(2)];
            % Jacobian F
            F = eye(length(x));
            F(1,3) = -1*u(1)*sind(x(3));
            F(2,3) = u(1)*cosd(x(3));
        end
        
        function append(h,u,R,landmarkPos,signature)
            % Append signature
            %--------------------------------------------------------------
            h.s = [h.s;signature];
            
            % Append landmark x,y position
            %--------------------------------------------------------------
            numOfLandmarks = (length(h.x) - 3) / 2;
            
            n = length(h.P);
            
            % Append landmark to x
            h.x = [h.x , landmarkPos(1), landmarkPos(2)];
            
            % Append landmark covariances to P
            %--------------------------------------------------------------
            % Get SLAM-Specific Jacobians (as defined by SLAM for Dummies!)
            jxr = [1 0 -u(1)*sind(h.x(3)); ...
                0 1  u(1)*cosd(h.x(3))];
            
            jz = [cosd(u(2)) -u(1)*sind(u(2)); ...
                sind(u(2)) u(1)*cosd(u(2))];
            
            % Append landmark to P (again as defined by SLAM for Dummies)
            h.P(n+1:n+2,n+1:n+2) = jxr*h.P(1:3,1:3)*jxr' + jz*R*jz';    %C
            h.P(1:3,n+1:n+2) = h.P(1:3,1:3)*jxr';                       %I
            h.P(n+1:n+2,1:3) = h.P(1:3,n+1:n+2)';                       %H
            for idx = 1: numOfLandmarks
                h.P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5)) = jxr*h.P(((idx-1)*2+4):((idx-1)*2+5),1:3)';   %F
                h.P(((idx-1)*2+4):((idx-1)*2+5),(n+1):(n+2)) = h.P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5))';%G
            end
        end
        
        function measure(h, laserData)
            % Search for landmarks
            [observed_LL] = h.LM.getLandmark(laserData,h.x);
            h.observed=observed_LL; 
            % Apply measurement update in EKF if landmarks are observed
            if(~isempty(observed_LL))
                numOfObservedLandmarks = size(observed_LL,1);
                for ii = 1:numOfObservedLandmarks  % "8: For all observed features..."
                    R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
                    %check if state has no landmarks
                    if(length(h.x)<4)
                        h.append(h.u,R,h.LM.landmarkObj.landmark(find([h.LM.landmarkObj.landmark.index])).loc,1);
                    else
                        %estimate correspondence
                        
                        % Measurement vector (Range and relative orientation)
                        z = observed_LL(ii,:);
                        numOfLandmarks=(length(h.x)-3)/2;
                        if(z(3)>numOfLandmarks)
                            %append new landmark
                            h.append(h.u,R,h.LM.landmarkObj.landmark(find([h.LM.landmarkObj.landmark.index]==z(3))).loc,z(3));
                   
                        else
                            idx=ii;
                            mu = h.x(1:2)' + z(1)*[cosd(z(2) + h.x(3));sind(z(2) + h.x(3))];
                            mu_k = [h.x(4+(idx-1)*2);h.x(4+(idx-1)*2+1)];
                            delta_k=mu_k-h.x(1:2)';
                            q_k=delta_k'*delta_k;
                            
                            %Line 13
                            z_k=[sqrt(q_k); wrapTo360(atan2d(delta_k(2),delta_k(1))-(h.x(3)))];
                            
                            %Line 14
                            numOfLandmarks=(length(h.x)-3)/2;
                            F_k = zeros(5,numOfLandmarks*2+3);F_k(1:3,1:3) = eye(3);F_k(4:5,(4+(idx-1)*2):(5+(idx-1)*2)) = eye(2);
                            
                            %Line 15
                            H_k = (1/q_k)*[-sqrt(q_k)*delta_k(1), -sqrt(q_k)*delta_k(2), 0, sqrt(q_k)*delta_k(1), sqrt(q_k)*delta_k(2); ...
                                delta_k(2), -delta_k(1), -q_k, -delta_k(2), delta_k(1)]*F_k;
                            
                            %Line 16
                            phi_k = H_k*h.P*H_k' + R;
                            
                            K=h.P*H_k'*phi_k^-1;
                            h.x = h.x + (K*(z(1:2)' - z_k))';
                            h.P = (eye(size(h.P)) - K*H_k)*h.P;
                        end
                        
                    end
                end
            end
        end
        
        function run(h)
            % Receive ROS Topics
            %======================================================================
            h.measData  = receive(h.measSub);
            h.contData = receive(h.contSub);
           
            % End receive ROS topics
            %----------------------------------------------------------------------
            
            % Calculate Odometry
            %======================================================================
            % odometry data calculated here is the dead-reckoned position from
            % odometry readings in ROS.
            
            p = h.contData.Pose.Pose;
            x_o = p.Position.X;
            y_o = p.Position.Y;
            
            quat = p.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = wrapTo360(rad2deg(angles(1)));
            
            % store current position in terms of x, y, theta (degrees) as ROS sees
            odomPose = [x_o,y_o,theta];
            % End calculate odometery
            %----------------------------------------------------------------------

            % Estimate Robot's pose
            %======================================================================
            % Initialize variables if first iteration
            if(isempty(h.oldOdomPose))
                h.oldOdomPose=odomPose;
            end
            % Get control vector (change in linear displacement and rotation)to
            % estimate current pose of the robot
            delta_D = sqrt((odomPose(1) - h.oldOdomPose(1))^2 + (odomPose(2) - h.oldOdomPose(2))^2);
            delta_Theta = rad2deg(angdiff(deg2rad(h.oldOdomPose(3)),deg2rad(odomPose(3))));
            h.u = [delta_D, delta_Theta];
            
            % Update position estimate
            h.predict(h.u);
            
            % Record current odometry pose for next iteration
            h.oldOdomPose = odomPose;
            
            h.measure(h.measData);
            
%             cartes_data = readCartesian(h.measData); %read cartesian co-ordinates
%             
%             rot = [cosd(h.x(3)) -sind(h.x(3)) h.x(1)+12.5; sind(h.x(3)) cosd(h.x(3)) h.x(2)+12.5; 0 0 1];
%             
%             %Plot scan data
%             hold on;
%             cartes_data = readCartesian(measData); %read cartesian co-ordinates
%             rot = [cosd(h.x(3)) -sind(h.x(3)) h.x(1); sind(h.x(3)) cosd(h.x(3)) h.x(2); 0 0 1];
%             tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
%             scatter(tmp(1,:),tmp(2,:),'magenta','.');
%             axis([-3.5 3.5 -3.5 3.5]);
        end
        
        
        function plot(h)
            
            
            % Plot robot
            drawRobot(h.x(1),h.x(2),h.x(3),0.25);
            hold on;
            % Plot landmarks
            for ii = 1:((length(h.x)-3)/2)
                scatter(h.x((ii-1)*2 + 4),h.x((ii-1)*2 + 5),'blue','x');
            end
            
            % Plot robot and landmark covariances
            robotSigma=[h.P(1,1),h.P(1,2);h.P(2,1),h.P(2,2)];
            robotMu=[h.x(1);h.x(2)];
            [eigvec,eigval]=eig(robotSigma);
            chi_square=2.2788;
            major=2*sqrt(chi_square*eigval(1,1));
            minor=2*sqrt(chi_square*eigval(2,2));
            t=-pi:0.01:pi;
            if(eigval(1,1)>eigval(2,2))
                arc=atan(eigvec(2,1)/eigvec(1,1));
                robot_x=major*cos(t);
                robot_y=minor*sin(t);
            else
                arc=atan(eigvec(2,2)/eigvec(1,2));
                robot_x=minor*cos(t);
                robot_y=major*sin(t);
            end
            R=[cos(arc) -sin(arc); sin(arc) cos(arc)];
            rCoords=R*[robot_x;robot_y]*.25;
            xr=rCoords(1,:);
            yr=rCoords(2,:);
            xr=xr+robotMu(1);
            yr=yr+robotMu(2);
            plot(xr,yr);
            
            for ii=4:2:size(h.x,2)
                landmarkSigma=[h.P(ii,ii),h.P(ii,ii+1);h.P(ii+1,ii),h.P(ii+1,ii+1)];
                robotMu=[h.x(ii);h.x(ii+1)];
                
                [eigvec,eigval]=eig(landmarkSigma);
                chi_square=2.2788;   %2.2788
                major=2*sqrt(chi_square*eigval(1,1));
                minor=2*sqrt(chi_square*eigval(2,2));
                t=-pi:0.01:pi;
                if(eigval(1,1)>eigval(2,2))
                    arc=atan(eigvec(2,1)/eigvec(1,1));
                    landmark_x=major*cos(t);
                    landmark_y=minor*sin(t);
                else
                    arc=atan(eigvec(2,2)/eigvec(1,2));
                    landmark_x=minor*cos(t);
                    landmark_y=major*sin(t);
                end
                R=[cos(arc) -sin(arc); sin(arc) cos(arc)];
                rCoords=R*[landmark_x;landmark_y]*.50;
                xr=rCoords(1,:);
                yr=rCoords(2,:);
                xr=xr+robotMu(1);
                yr=yr+robotMu(2);
                plot(xr,yr);
            end
            
            hold off;
        end
        
    end
end

