classdef RANSAC < handle
    properties
        landmark;  % RANSAC landmark storage struct
        observed;  % used to plot observed landmarks
    end
    methods
        function h = RANSAC()
            % create variable for RANSAC landmark storage. Initialize as
            % empty: getLandmark() will initialize structure upon first
            % landmark observation.
            h.landmark = [];
        end
        
        function [ observed_LL ] = getLandmark(h, laserdata, pose)
            % NOTE: pose is now the entire state vector
            % Update landmark structre from state vector
            if(~isempty(h.landmark))
                h.updateLandmarkList(pose);
            end
            input_landmark_list = h.landmark;

            %pseudo---
            
            %1. Remove NULLS form laser data
            
            %2. Convert laser data to cartesion coordinates
            
            %3. Rotate cartesion coordinates from local frame to world frame
            
            %4. Take random point from world frame cartesion coordinates
            
            %5. Attempt to find # of points that are within # of degrees of the random
            %   sample, if not enough points can be found then take new point up to # times
            
            %6. Create line of best fit using random point and points within # degrees
            
            %7. Search through rotated cartesion coordinates for any points within # of
            %   the line of best fit.
            
            %8. Create new line of best fit using all points found (including the
            %   inital random point, then remove those points from the rotated
            %   cartesion coordinates.
            
            %9. Determine closest point on line of best fit that is orthoganal to the
            %   origin. Add that point to a potential landmark list
            
            %9.5 Repeat steps 4-9 until the number of points in the rotated cartesion
            %    coordinates is less than # or until timeout occurs.
            
            %10. Search through the input landmark list to see if any points on the
            %    potential landmark list are within # of an input landmark. If so then
            %    increment the count of the landmark on the input landmark list, else
            %    add the potential landmark to the landmark list
            
            %11. During the previous step, if a landmark was reobserved (count was
            %    incremented) then add it to the observed landmark list in the form of
            %    [distance to origion, degrees from pose]
            
            %12. Check to see if the count of any un-indexed landmarks has exceeded #,
            %    if so then index that landmark and add it to the observed landmark
            %    list, else... uh do nothing
            
            %13. Output the now updated input landmark list and the observed landmark
            %    list
            
            
            lineConsensus=300;    %number of points needed to be near a line of best
            %fit in order for it to be considered a potential wall
            
            wallSearchTimeOut=3;  %maximum number of times a single laser scan will be
            %searched for walls
            
            numberOfPointsWithinDegrees=20; %number of points used to create sample line
            
            degrees=5;           %number of degrees a point must be within
            %for it to be considered for use when
            %creating the sample line
            
            lineOfBestFitDistance=.25; %distance a point can be away from the line of
            %best fit and still be considered part of the
            %line
            
            landmarkDistance=.50;  %distance a potential landmark can be away from a
            %input landmark without being considered a new
            %landmark
            
            landmarkCountConsensus=10;  %number of times a landmark must be observed
            %before being considered an 'offical' landmark
            
            freshnessTimer  =50  ;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %start!%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %1 remove null
            ind =  find(isnan(laserdata.Ranges));
            laserdata.Ranges(ind)=[];
            
            %2 convert to cartes
            localFrameCartesCords=readCartesian(laserdata);
            
            %3 get points from robot local frame to world frame
            rot = [cosd(pose(3)) -sind(pose(3)) pose(1); sind(pose(3)) cosd(pose(3)) pose(2); 0 0 1];
            worldFrameCartesCords = rot*[localFrameCartesCords,ones(length(localFrameCartesCords),1)]';
            worldFrameCartesCords=worldFrameCartesCords';
            worldFrameCartesCords(:,3)=[];
            
            
            counter=0;
            potentialLineList=[];
            
            %9.5 keep searching worldFrameCartesCords for walls until not enough points are left
            %    for a wall to pass the lineConsensus (or the loop runs # times)
            while(length(worldFrameCartesCords)>lineConsensus && counter<wallSearchTimeOut)
                
                %4 & 5 Get random point and # of points within 10 degrees
                pointsWithinDegrees=h.findPoints(worldFrameCartesCords,numberOfPointsWithinDegrees,degrees);
                
                %the reamining steps only run if steps 4 & 5 were completed
                %succesfully
                if(~isempty(pointsWithinDegrees))
                    
                    %6 & 7 & 8 Using the random points try to find a wall
                    [potentialLine,worldFrameCartesCords] = h.findPotentialLine(worldFrameCartesCords,pointsWithinDegrees,lineConsensus,lineOfBestFitDistance);
                    potentialLineList=[potentialLineList;potentialLine];
                end
                counter=counter+1;
            end
            
            %9 if any potential lines were found then find the closest point
            %  orthogonal to the origion of each potential line and set it as a
            %  potential landmark
            if(~isempty(potentialLineList))
                potentialLandmarkList=h.getOrthogPoints(potentialLineList);
                
                
                %10 & 11 & 12 & 13 check to see if any of the potentialLandmarks are already on the
                %   input landmark list. if so then increment the count of that input
                %   landmark else add it to the list with an index of 0.
                %   ALSO, if a landmark is already indexed and is 'reobserved' then add
                %   it to an observed_LL list in terms of distance to robot, angle to
                %   robot, and index.
                [observed_LL, output_landmark_list]=h.getOutputLandmarkListAndObservedLandmarkList(input_landmark_list,potentialLandmarkList,landmarkCountConsensus,landmarkDistance,pose(1:3),freshnessTimer);
                
            else
                output_landmark_list=input_landmark_list;
                observed_LL=[];
            end
            h.landmark = output_landmark_list;
            
            
        end
        
        function pointsWithinDeg = findPoints(h, worldFrameCartesCords,numberOfPointsWithinDegrees,degrees)
            
            %take a random point from the avaiable points
            [samplePoint,~] = datasample(worldFrameCartesCords,1,'Replace',false);
            
            %get the degree max and min away from the sample point
            randomSampleDegreeMax = atand(samplePoint(1,2)/samplePoint(1,1)) + degrees/2;
            randomSampleDegreeMin = atand(samplePoint(1,2)/samplePoint(1,1)) - degrees/2;
            
            %remove all points that are outside the # degree limit
            ii=1;
            while ii<length(worldFrameCartesCords)
                deg = atand(worldFrameCartesCords(ii,2)/worldFrameCartesCords(ii,1));
                if( deg>randomSampleDegreeMax || deg <randomSampleDegreeMin)
                    worldFrameCartesCords(ii,:)=[];
                else
                    ii=ii+1;
                end
            end
            
            %take # of random samples from the points within # degrees
            listOfPoints=[];
            if(size(worldFrameCartesCords,1)>numberOfPointsWithinDegrees)
                [listOfPoints,~] = datasample(worldFrameCartesCords,numberOfPointsWithinDegrees,'Replace',false);
            end
            
            pointsWithinDeg=listOfPoints;
            
        end
        
        function [potentialLine,updatedWorldFrameCartesCords] = findPotentialLine(h,worldFrameCartesCords,pointsWithinDegrees,lineConsensus,lineOfBestFitDistance)     
            sampleLine=polyfit(pointsWithinDegrees(:,1),pointsWithinDegrees(:,2),1);
            v1=[1,polyval(sampleLine,1),0];
            v2=[2,polyval(sampleLine,2),0];
            idx = zeros(length(worldFrameCartesCords),1);
            
            for i= 1 : size(worldFrameCartesCords,1)
                p=[worldFrameCartesCords(i,:),0];
                a=v1-v2;
                b=p-v2;
                d =norm(cross(a,b))/norm(a);
                if d<lineOfBestFitDistance
                    idx(i)=i;
                end
            end
            
            idx=idx(idx~=0);
            updatedWorldFrameCartesCords=worldFrameCartesCords;
            potentialLine=[];
            if length(idx)>lineConsensus
                
                updatedWorldFrameCartesCords=worldFrameCartesCords;
                updatedWorldFrameCartesCords(idx,:)=[];
                
                pointsWithinDistance=worldFrameCartesCords(idx,:);
                potentialLine=polyfit(pointsWithinDistance(:,1),pointsWithinDistance(:,2),1);
                
                %plot([0,5],[polyval(potentialLine,0),polyval(potentialLine,5)]);hold on;
                
            end
            
        end
        
        function orthogPoints = getOrthogPoints(h,potentialLineList)
            orthogPoints=zeros(size(potentialLineList,1),2);
            for ii=1:size(potentialLineList,1)
                m= (polyval(potentialLineList(ii,:),0)-polyval(potentialLineList(ii,:),5))/(0-5);
                b=polyval(potentialLineList(ii,:),0)-m*0;
                
                syms x y
                eqn1 = y-m*x == b;
                eqn2 = y+1/m*x == 0;
                [A,B]=equationsToMatrix([eqn1,eqn2],[x,y]);
                X = linsolve(A,B);
                
                orthogPoints(ii,1)=X(1);
                orthogPoints(ii,2)=X(2);
            end
        end
        
        function [reobservedLandmarkList, outputLandmarkList]=getOutputLandmarkListAndObservedLandmarkList(h,input_landmark_list,potentialLandmarkList,landmarkCountConsensus,landmarkDistance,pose,freshnessTimer)       
            reobservedLandmarkList=[];           
            if(isempty(input_landmark_list)&&~isempty(potentialLandmarkList))
                % input_landmark_list=[potentialLandmarkList(1,:),1,0];  %consider changing this once done testing
                input_landmark_list(1).loc=[potentialLandmarkList(1,:)];
                input_landmark_list(1).observe=1;
                input_landmark_list(1).index=0;
                input_landmark_list(1).fresh=freshnessTimer;
            elseif(~isempty(potentialLandmarkList))   
                %loop through all the potential landmarks
                for ii=1:size(potentialLandmarkList,1)
                    flag=0;
                    %loop through all the landmarks that already exsit
                    for jj=1:size(input_landmark_list,2)
                        %check to see if a potential landmark might have already
                        %been observed and marked as a landmark
                        d=norm(potentialLandmarkList(ii,:)-[input_landmark_list(jj).loc(1),input_landmark_list(jj).loc(2)]);
                        
                        %if so then increment that landmarks count
                        if(d<landmarkDistance)
                            
                            %increment count
                            input_landmark_list(jj).observe=input_landmark_list(jj).observe+1;
                            flag=1;
                            
                            %check if the count has increased to the consensus, if
                            %so then index it
                            if(input_landmark_list(jj).observe)>landmarkCountConsensus && input_landmark_list(jj).index==0
                                input_landmark_list(jj).index=max([input_landmark_list(:).index])+1;
                            end
                            
                            %if the landmark is indexed then replace the landmark
                            %list value with the measured value
                            if(input_landmark_list(jj).index~=0)
                                input_landmark_list(jj).loc=potentialLandmarkList(ii,:);
                                
                                xBot=pose(1);
                                yBot=pose(2);
                                xLandmark=input_landmark_list(jj).loc(1);
                                yLandmark=input_landmark_list(jj).loc(2);
                                
                                dist=sqrt((xBot-xLandmark)^2 +(yBot-yLandmark)^2);
                                ang=atan2d(yLandmark-yBot,xLandmark-xBot);
                                ang=wrapTo360(ang-pose(3));
                                
                                if(isempty(reobservedLandmarkList))
                                    reobservedLandmarkList=[reobservedLandmarkList;dist,ang,input_landmark_list(jj).index];
                                    
                                elseif ~find(reobservedLandmarkList(:,3) == input_landmark_list(jj).index)
                                    reobservedLandmarkList=[reobservedLandmarkList;dist,ang,input_landmark_list(jj).index];
                                end
                                
                            end
                            
                            %break jj for loop
                            jj=size(input_landmark_list,2);
                            
                        end
                    end
                    %if the potential landmark was not associated then add it to
                    %the list
                    if(flag==0)
                        %input_landmark_list=[input_landmark_list;potentialLandmarkList(ii,:),1,0];
                        input_landmark_list(size(input_landmark_list,2)+1).loc=potentialLandmarkList(ii,:);
                        input_landmark_list(size(input_landmark_list,2)).observe=1;
                        input_landmark_list(size(input_landmark_list,2)).index=0;
                        input_landmark_list(size(input_landmark_list,2)).fresh=freshnessTimer;
                        
                    end
                    
                end
                
            end
            %  reobservedLandmarkList=sort(reobservedLandmarkList,4);
            
            %freshnessssss
            %     idx=reobservedLandmarkList(:,4);
            %     for ii=1:length(idx)
            %         for jj=1:size(input_landmark_list,2)
            %             if(input_landmark_list.index ~= idx && input_landmark_list.count<landmarkCountConsensus
            %                 %decrement freshness
            %         end
            %
            %     end
            
            %since indexed landmarks are NEVER decremented really just need to
            %decrement nonindexed landmarks every loop.
            ii=1;
            while(ii<=size(input_landmark_list,2))
                if(input_landmark_list(ii).index==0)
                    input_landmark_list(ii).fresh=input_landmark_list(ii).fresh-1;
                    if(input_landmark_list(ii).fresh==0)
                        input_landmark_list(ii)=[];
                        ii=ii-1;
                    end
                end
                ii=ii+1;
            end
            
            outputLandmarkList=input_landmark_list;
        end
        
        function  updateLandmarkList(h, state_vector)
            %UPDATELANDMARKLIST The function will update the input_landmark_list x and
            %y coordinates based on the thier index within the state_vector. It will
            %then output the updated input_landmark_list as updated_landmark_list
            %
            %STATE_VECTOR Will hold landmark coordinates that the input_landmark_list
            %will be updated with.
            %
            %INPUT_LANDMARK_LIST Will hold landmark coordinates with indexes that
            %represent thier position in the state_vector. If a input_landmark_list
            %coordinate does not match the indexed state_vector coordinate then the
            %input_landmark_list will be updated using the state_vector
            %
            %UPDATED_LANDMARK_LIST The output will be the updated input_landmark_list
            %that was updated based on the state vector
            
            
            if(length(state_vector)>3)
                %loop through all of the coordinates in the confirmed landmark list
                for ii=(length(state_vector)-3)/2
                    %update the landmark_list coordinates based on the confirmed
                    %landmark list coordinates
                    for jj=1:length(h.landmark)  %FIX: will search through entire list even after it finds the coordinates
                        if(h.landmark(jj).index==ii)
                            %  landmark_list(jj,1)=state_vector(ii);
                            %  landmark_list(jj,2)=state_vector(ii+1);
                            h.landmark(jj).loc(1)=state_vector((2*(ii-1))+4);
                            h.landmark(jj).loc(2)=state_vector((2*(ii-1))+5);
                            %FIX THE INDEXING ABOVE
                            jj=length(h.landmark);
                        end
                    end
                    %each pair of coordinates occupies two locations in the vector
                    %so need to iterate location by 2
                    %             ii=ii+1;
                end
            end
        end
        
        
        function plot(h,x,observed)
            % Plot "unofficial"/pre-filtered landmarks
            temp=[h.landmark(:).index];
            idx = find(temp(:) == 0);
            temp=[];
            for mm=1:size(idx,1)
                temp=[temp;h.landmark(idx(mm)).loc(1),h.landmark(idx(mm)).loc(1)];
            end
            if(~isempty(idx))
                scatter(temp(:,1),temp(:,2),[],[.5 .5 .5],'x');
            end
            
            % Plot range and orientation of observed landmarks
            if(~isempty(observed))
                % Plot observed landmark locations
                for ii = 1:size(observed,1)
                    temp = [h.landmark(:).index];
                    idx2 = find(temp(:)==observed(ii,3));  % Landmark of correspondence idx
                    scatter(h.landmark(idx2).loc(1),h.landmark(idx2).loc(2),'o','b');
                    % Plot observed landmark distances and orientations
                    lineptsx = x(1) + observed(:,1).*cosd(observed(:,2) + x(3));
                    lineptsy = x(2) + observed(:,1).*sind(observed(:,2) + x(3));
                    for jj = 1:length(lineptsx)
                        plot([x(1) lineptsx(jj)],[x(2) lineptsy(jj)],'red');
                    end
                end
            end
            
        end
        
    end
    
end

