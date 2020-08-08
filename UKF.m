clc
clear all
close all
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Matrix initialisation  
ic = 400;                      % Number of iterations 
x = zeros(2,1,ic);             % True State
mubar = zeros(2,1,ic);         % Apriori state estimates
mu = zeros(2,1,ic);            % Aposteriori state estimates
sigmabar = zeros(2,2,ic);      % Apriori error covariance estimates
sigma = zeros(2,2,ic);         % Aposteriori error covariance estimates
z = zeros(2,1,ic);             % Measurements
K = zeros(2,2,ic);             % Kalman Gain
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Model Definitions
V = [ 1 0 ;                    % State-error Jacobian
      0 0 ];
H = [ 1 0 ;                    % Measurement-state Jacobian
      0 1 ];
W = [ 1 0 ;                    % Measurement-error Jacobian
      0 1 ];
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Initialisation
x(:,1) = [ 0         ;         % True initial state
           3*pi/300 ];
mu(:,1) = [ 1         ;        % Initial aposteriori state estimate
            pi/300 ];
R = [0.000    0 ;              % Process noise covariance
     0    0.000];
Q = [0.0001   0 ;              % Measurement noise covariance
     0    0.001];
sigma(:,:,1) = [ 1   0  ;      % Initial aposteriori error covariance estimate
                     0   1 ]; 
                 
% weights and Covariance Initialization for segma points
n=2;                          %dim_of_state_vector
[wt_pt,wt_cov]=sigmaPntCovWt(n); 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UKF MAIN
 
for t = 2:ic
    
    %Step 1 Update true state
    x(:,t) = [sin(x(2,t-1)*(t-1));x(2,t-1)] + sqrt(R)*[randn;randn];
    
    % Update measurements
    z(:,t) = x(:,t) + sqrt(Q)*[randn;randn];
    
    %Step 2 - Sigma Points 
    si(1:2,1:2*n+1,t-1)=sigmapnt(mu(1:2,1,t-1),sigma(1:2,1:2,t-1),n);
   
    %Step 3 - State vector at each sigma points
    for i=1:2*n+1
    sibar(1:2,i,t) = [sin(si(2,i,t-1)*(t-1));si(2,i,t-1)];
    end
    
    %Step 4 - Mean of State vector Sigma Points
    a = zeros(2,1);
    for i=1:2*n+1
        a = a+wt_pt(i)*sibar(1:2,i,t);
    end
    mubar(:,1,t) = a;
    
    %Step 5 - Covariance of the transformed Sigma Points
    b= zeros(2,2);
    for i=1:2*n+1
        sigmabar(1:2,1:2,t)= b+wt_cov(i)*(sibar(1:2,i,t)-mubar(1:2,1,t))*(sibar(1:2,i,t)-mubar(1:2,1,t))';
        b=sigmabar(1:2,1:2,t);
    end
    Rt = V*R*V';
    sigmabar(:,:,t)=sigmabar(:,:,t)+Rt;
   
    %Step 6  - Sigma points around the transformed mean 
    sibart(1:2,1:2*n+1,t)=sigmapnt(mubar(1:2,1,t),sigmabar(1:2,1:2,t),n);
    
    %Step 7  - Observation matrix determination as the the sigma-points 
              %determined at step 6 are passed through the (noise-free) observation function h(·).
    for i=1:2*n+1
    z_bar(:,i,t) = x(:,t);  %[sin(sp_tr(2,i,t)*(t-1));sp_tr(2,i,t)];  %assuming it measures accurately after excluding the noise
    end
    
    %Step 8  - Mean Observation
    %the observation sigma-points Z_bar calculated in step 7, are used to calculate the predicted observation Z_hat and its uncertainty St.
    q=zeros(2,1);
    for i=1:2*n+1
        q=q+wt_pt(i)*z_bar(1:2,i,t);
    end
    z_hat(:,1,t)=q;
    
    %Step 9 - Innovation covariance - uncertainty St of predicted observation Z_hat 
    L=zeros(2,2);
    for i=1:2*n+1
        L=L+ wt_cov(i)*(z_bar(:,i,t)-mubar(:,1,t))*(z_bar(:,i,t)-z_hat(:,1,t))';
    end
    Qt = W*Q*W';
    St(:,:)=L+Qt;
    
    %Step 10 - State-Observation Covariance
    % the cross-covariance between the state and the observation is computed then used to compute the Kalman gain 
    x1=zeros(2,2);
    for i=1:2*n+1
        x1= x1+ wt_cov(i)*(sibart(:,i,t)-mubar(:,1,t))*(z_bar(:,i,t)-z_hat(:,1,t))';
    end
    SO_Cov(:,:)=x1
    
    %Step 11 - Kalman Gain
    K(1:2,1:2,t)=SO_Cov(:,:)*((St(:,:))^(-1));
    K(1:2,1:2,t)
    
    %Step 12 - Mean State
    mu(:,1,t)=mubar(:,1,t)+K(:,:,t)*(z(:,t)-z_hat(:,1,t));
    
    %Step 13 - Covariance SP of the states
    sigma(:,:,t)=sigmabar(1:2,1:2,t)-K(:,:,t)*St(:,:)*K(:,:,t)';
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Plotting 
figure (3)
subplot(2,1,1)
% Actual state position
b = plot(2:ic,x(1,2:ic),'b');
hold on
% State position estimates
r = plot(2:ic,mu(1,2:ic),'r');
% State measurements
g = plot(2:ic,z(1,2:ic),'g+');
title('Unscented Kalman Filtering of a Sine Wave - Position');
legend([b r g],'True Position','Position Estimates','Position Measurements');
xlabel('Time')
ylabel('Position')
grid on
subplot(2,1,2)
% Actual state frequency
b = plot(2:ic,x(2,2:ic),'b');
hold on
% State frequency estimates
r = plot(2:ic,mu(2,2:ic),'r');
% Frequency measurements
g = plot(2:ic,z(2,2:ic),'g+');
title('Unscented Kalman Filtering of a Sine Wave - Frequency');
legend([b r g],'True Frequency','Frequency Estimates','Frequency Measurements');
xlabel('Time')
ylabel('Frequency')
grid on
%--------------------------------------------------------------------------
  
function [weights_pnt,weights_cov] = sigmaPntCovWt(n)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Returns two vectors with weights assigned to each sigma point and
% covariance around each point

% Input: 
% n = dimension of the state vector
% alpha=Factor deciding the spread of sigma points around the mean
% k= parameter that changes with the distribution of the data.k=2 for
% Gaussian distribution
% lambda = controls the spread of sigma points based on different distributions
% and dimensionality of state vectors

% Output:
% weights_pnt= weights corresponding to each sigma point,(2Xd+1,1)
% weights_cov= weights corresponding to covraiance,(2Xd+1,1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha=0.7;      
k=2;      
beta=2;  
weights_pnt=zeros(2*n+1,1);
weights_cov=zeros(2*n+1,1);
lambda=alpha^2*(n+k)-n;
for i=1:2*n+1
    if(i==1)
        weights_pnt(i)= lambda/(n+lambda);
    else
        weights_pnt(i)=1/(n+lambda);
    end
end
for j=1:2*n+1
    if (j==1)
        weights_cov(j)=lambda/(n+lambda)+(1-alpha^2+beta);
    else
        weights_cov(j)=1/(n+lambda);
    end
end
end
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function name: sigmapnt
% Returns the sigma points calculated around the given mean. Matrix
% dimensionality:dX1X2*n+1
% Input: 
% mu=Mean of the state (dX1) vector
% covaria= Covariance of the state dXd matrix
% n = dimension of the state vector
 
% alpha=Factor deciding the spread of sigma points around the mean
% k= parameter that changes with the distribution of the data.k=2 for
% Gaussian distribution
% lambda = controls the spread of sigma points based on different distributions
% and dimensionality of state vectors
% Output:
% sigmapoints = sigma points around the mean for the particular instance
 
% Name: Vineet Pandey
% CWID: 10826588
% Date: 11/22/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
function [sigmapoints] = sigmapnt(mu,covaria,n)
sigmapoints=zeros(n,1,2*n+1);
sigmapoints(:,1,1)=mu;
alpha=0.7;
k=2;
lambda=alpha^2*(n+k)-n;
 
[v,l] = eig((n+lambda)*covaria);
shifts = v*sqrt(l)*v^-1;
 
for i=2:n+1
    sigmapoints(:,1,i)=mu+shifts(:,(i-1));
end
for i=n+2:2*n+1
    sigmapoints(:,1,i)=mu-shifts(:,(i-1)-n);
end
 
if any(~isreal(sigmapoints))
    sigmapoints = real(sigmapoints);
    disp('imaginary sigma points!');
end
 
end
 
