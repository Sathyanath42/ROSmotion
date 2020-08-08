clc
clear all
close all
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Matrix initialisation 
i = 400;                   % Number of iterations 
x = zeros(2,i);            % True State
mubar = zeros(2,i);        % Apriori state estimates
mu = zeros(2,i);           % Aposteriori state estimates
sigmabar = zeros(2,2,i);   % Apriori error covariance estimates
sigma = zeros(2,2,i);      % Aposteriori error covariance estimates
z = zeros(2,i);            % Measurements
K = zeros(2,2,i);          % Kalman Gain
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Model Definitions
V = [ 1 0 ;                % State-error Jacobian
      0 0 ];
H = [ 1 0 ;                % Measurement-state Jacobian
      0 1 ];
W = [ 1 0 ;                % Measurement-error Jacobian
      0 1 ];
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Initialisation
x(:,1)  = [ 0         ;    % True initial state
            3*pi/300 ];
mu(:,1) = [ 1         ;    % Initial aposteriori state estimate
            1*pi/300 ];
R = [0.001    0 ;          % Process noise covariance
     0        0];
Q = [0.1    0 ;            % Measurement noise covariance
     0    0.01];
sigma(:,:,1) = [ 1   0  ;  % Initial aposteriori error covariance estimate
                 0   1 ];
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%EKF MAIN
 
for t = 2:i
    
    %Step 1 Update true state
    x(:,t) = [sin(x(2,t-1)*(t-1));x(2,t-1)] + sqrt(R)*[randn;randn];
    
    % Update measurements
    z(:,t) = x(:,t) + sqrt(Q)*[randn;randn];
    
    %Step 2 Update apriori estimate - algorithm step mio=g(Ut,mio_t-1)
    mubar(:,t) = [sin(mu(2,t-1)*(t-1));mu(2,t-1)];  %mubar is the first derivate of xt without noise nad replace xt with mio
    
    Gt = [0   (t-1)*cos(mu(2,t-1)*(t-1)) ; %state jacobian:derivative of g
          0                    1       ] ;
      
    Rt = V*R*V';  %Q and R as defined in text
    Qt = W*Q*W';
    %step 3 Update aprioiri error covariance estimate
    sigmabar(:,:,t) = Gt*sigma(:,:,t-1)*Gt' + Rt;
    
    %step 4 Update Kalman gain
    K(:,:,t) = sigmabar(:,:,t)*H' / (H*sigmabar(:,:,t)*H'+ Qt);
    
    %step 5 Update aposteriori state estimate
    mu(:,t) = mubar(:,t) + K(:,:,t) * (z(:,t) - mubar(:,t));
    
    %step 6 Update aposteriori error covariance estimate
    sigma(:,:,t) = (eye(2) - K(:,:,t)*H) * sigmabar(:,:,t);
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plotting
figure (1)
% Actual state position
b = plot(2:i,x(1,2:i),'b');
hold on
% State position estimates
r = plot(2:i,mu(1,2:i),'r');
% State measurements
g = plot(2:i,z(1,2:i),'g+');
title('Extended Kalman Filtering of a Sine Wave - Position');
legend([g b r],'Position Measurements','True Position','Position Estimates');
xlabel('Time')
ylabel('Position')
 
figure(2)
% Actual state frequency
b = plot(2:i,x(2,2:i),'b');
hold on
% State frequency estimates
r = plot(2:i,mu(2,2:i),'r');
% Frequency measurements
g = plot(2:i,z(2,2:i),'g+');
title('Extended Kalman Filtering of a Sine Wave - Frequency');
legend([g b r],'Frequency Measurements','True Frequency','Frequency Estimates');
xlabel('Time')
ylabel('Frequency')
 
