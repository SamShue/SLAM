classdef test_slam_class
    %UNTITLED2 Summary of this class goes here
    
    properties
        laser
        odom
    end
    
    methods
        function h=test_slam_class()

            h.laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
            h.odom = rossubscriber('/odom');
            

        end
        
        function runSlam(h)
            % Receive ROS Topics
            %======================================================================
            laserData  = receive(h.laser); %recieve a laser scan
            odomData = receive(h.odom);

            % End receive ROS topics
            %----------------------------------------------------------------------
            
            % Calculate Odometry
            %======================================================================
            % odometry data calculated here is the dead-reckoned position from
            % odometry readings in ROS.
            
            p = odomData.Pose.Pose;
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
            if(~exist('s','var'))
                oldOdomPose = odomPose;
                h.s = SLAM('EKF_SLAM');
                u = [0, 0];
            else
                % Get control vector (change in linear displacement and rotation)to
                % estimate current pose of the robot
                delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
                delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
                u = [delta_D, delta_Theta];
                
                % Update position estimate
                h.s.predict(u);
                
                % Record current odometry pose for next iteration
                oldOdomPose = odomPose;
            end
            
            h.s.measure(laserData, u);
            
            cartes_data = readCartesian(laserData); %read cartesian co-ordinates
            
            rot = [cosd(h.s.slam.x(3)) -sind(h.s.slam.x(3)) h.s.slam.x(1)+12.5; sind(h.s.slam.x(3)) cosd(h.s.slam.x(3)) h.s.slam.x(2)+12.5; 0 0 1];
            world_frame_laser_scan = rot*[cartes_data,ones(length(cartes_data),1)]';
            
            %setOccupancy(map, world_frame_laser_scan(1:2,:)',1);
            %show(map);
            
            
            
            
            %Plot scan data
            
            cartes_data = readCartesian(laserData); %read cartesian co-ordinates
            rot = [cosd(h.s.slam.x(3)) -sind(h.s.slam.x(3)) h.s.slam.x(1); sind(h.s.slam.x(3)) cosd(h.s.slam.x(3)) h.s.slam.x(2); 0 0 1];
            tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
            scatter(tmp(1,:),tmp(2,:),'magenta','.');
            axis([-3.5 3.5 -3.5 3.5]);
            
            
            %Plot slam data (landmarks, observed landmarks, root)
            h.s.plot();
            
            
            
        end

    end 
end


