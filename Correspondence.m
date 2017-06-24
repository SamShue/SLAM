classdef Correspondence
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s_cost;
        s_thresh;
        method;
    end
    
    methods
        function h = Correspondence(cost, thresh, method)
            h.method=method;
            switch(method)
                case 'EKF_SLAM_UC'
                h.s_cost = cost;
                h.s_thresh = thresh;

                otherwise
                warning('Improper method specified. Using ML as default.');
                h.s_cost = cost;
                h.s_thresh = thresh;
                h.method = 'ML';
            end 
        end
        
        function [newLL,index]= estimateCorrespondence(h,z,R,x,P,s)
            %consider using argnum in addition to a swtich 
            switch(h.method)
                case 'EKF_SLAM_UC'
                    [newLL,index]=estimateCorrespondenceUC(h,z,R,x,P,s);
                otherwise 
            end
        end
        
        function [newLL,index] = estimateCorrespondenceUC(h,z,R,x,P,s)
            %ASSUMPTION, x contains at least one landmark
            %            z is of the form [range,bearing]
            
            %default values for if the landmark is new
            newLL=true;
            
            % Line 9: Estimate landmarks position from measured range/bearing and robots current position
            mu = x(1:2)' + z(1)*[cosd(z(2) + x(3));sind(z(2) + x(3))];
            
            %number of landmarks in the state vector
            numOfLandmarks = (length(x)-3)/2;
            index = numOfLandmarks + 1;
            
            %starting likelihood of the measurment being a new landmark
            min_log_likelihood = Inf;
            
            %array for storing the likelihood of the measurment being an landmark already in the state vector
            log_likelihood = zeros(numOfLandmarks,1);
            
            % Line 10: Compare the measured landmark with all exsiting landmarks
            for kk=1:numOfLandmarks
                mu_k=[x((kk-1)*2 + 4);x((kk-1)*2 + 5)];
                
                delta_k=mu_k-x(1:2)';
                q_k=delta_k'*delta_k;
                
                %Line 13
                z_k=[sqrt(q_k); wrapTo360(atan2d(delta_k(2),delta_k(1))-(x(3)))];
                
                %Line 14
                F_k = zeros(5,numOfLandmarks*2+3);F_k(1:3,1:3) = eye(3);F_k(4:5,(4+(kk-1)*2):(5+(kk-1)*2)) = eye(2);
                
                %Line 15
                H_k = (1/q_k)*[-sqrt(q_k)*delta_k(1), -sqrt(q_k)*delta_k(2), 0, sqrt(q_k)*delta_k(1), sqrt(q_k)*delta_k(2); ...
                    delta_k(2), -delta_k(1), -q_k, -delta_k(2), delta_k(1)]*F_k;
                
                %Line 16
                phi_k = H_k*P*H_k' + R;
                
                %Line 17
                position_cost = (z(1:2)' - z_k)'*phi_k^-1*(z(1:2)' - z_k);
                
                signiture_cost = (z(3) - s(kk))'*h.s_cost^-1*(z(3) - s(kk));
                
                %Store the likelihood per this landmark in the state
                %log_likelihood(kk)=position_cost+signiture_cost;
                log_likelihood(kk)=signiture_cost;
                %check to see if the likelihood is below the new landmark
                %threshold
                if (log_likelihood(kk) <= h.s_thresh)
                    %check to see if the landmark the measurment is currently being compared to
                    %is more likely than any of the previous landmarks
                    if (log_likelihood(kk) < min_log_likelihood)
                        newLL = false; %not a new landmark
                        min_log_likelihood = log_likelihood(kk); %update min_log_likelihood
                        index=kk; %update index
                    end
                end
            end
        end

    end 
        
end

