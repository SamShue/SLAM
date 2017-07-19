classdef OccupancyGrid < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
        mapLen = 100;
        mapWid = 100;
        res = 5;
        
        chatpub;
        msg;
        map;
        tftree;
    end
    
    methods
        function h = OccupancyGrid()
            h.tftree=rostf;
            %Occupancy grid initalization
            h.map=robotics.BinaryOccupancyGrid(mapLen,mapWid,res);
            h.chatpub = rospublisher('/map','nav_msgs/OccupancyGrid');
            %  h.chatpub = rospublisher('/chatter','std_msgs/String');
            h.msg = rosmessage(h.chatpub);
            
            h.msg.Header.FrameId = 'map';
            
            
            function setOccupancy(h, cartes_data, pose)
                % Calculate rotation matrix local frame -> world frame
                rot = [cosd(pose(3)) -sind(pose(3)) pose(1)+(h.mapLen/2); sind(pose(3)) cosd(pose(3)) pose(2)+(h.mapLen/2); 0 0 1];
                
                %Plot map
                world_frame_laser_scan = rot*[cartes_data,ones(length(cartes_data),1)]';
                world_frame_scan(1,:)=world_frame_laser_scan(1,:) + (h.mapLen/2);
                world_frame_scan(2,:)=world_frame_laser_scan(2,:) + (h.mapWid/2);
                
                
                setOccupancy(h.map, world_frame_scan(1:2,:)',1);
                
                send(h.chatpub,h.msg);
                writeBinaryOccupancyGrid(h.msg,h.map);
                
                
                tfpub = rospublisher('/tf', 'tf2_msgs/TFMessage');
                tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
                tfStampedMsg.ChildFrameId = 'odom';
                tfStampedMsg.Header.FrameId = 'map';
                if robotics.ros.internal.Global.isNodeActive
                    tfStampedMsg.Header.Stamp = rostime('now');
                end
                tfStampedMsg.Transform.Translation.X = h.slam.pose(2)+20;
                tfStampedMsg.Transform.Translation.Y = h.slam.pose(1)+20;
                tfStampedMsg.Transform.Translation.Z = 0;
                tfStampedMsg.Transform.Rotation.W = quat.W;
                tfStampedMsg.Transform.Rotation.X = quat.X;
                tfStampedMsg.Transform.Rotation.Y = quat.Y;
                tfStampedMsg.Transform.Rotation.Z = quat.Z;
                
                % tform = rosmessage('geometry_msgs/TransformStamped')
                % tform.ChildFrameId = 'new_frame';
                % tform.Header.FrameId = 'base_link';
                % tform.Transform.Translation.X = 0.5;
                % tform.Transform.Rotation.Z = 0.75;
                
                sendTransform(h.tftree, tfStampedMsg);
                send(h.chatpub,h.msg);
            end
            
            function plot()
                show(h.map);
            end
        end
        
    end
end
    
