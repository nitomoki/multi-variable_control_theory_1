function footstep = generate_walking_pattern(step_duration, T)
    N = 10;
    
    stride  = 0.3;
    spacing = 0.2;
    
    % generate footsteps
    % columns: k
    % rows   : pos_x pos_y pos_z theta dcm_x dcm_y zmp_x zmp_y
    footstep = zeros(8,N);
    
    % first two steps
    footstep(1:4,1) = [0; -spacing/2; 0; 0];  % 1st step  right
    footstep(1:4,2) = [0;  spacing/2; 0; 0];  % 2nd step  left
    
    % intermediate footsteps
    sign = -1;
    for k = 2:N-2
       footstep(1:4,k+1) = footstep(1:4,k) + [stride/2; sign*spacing; 0; 0];
       sign = -sign;
    end
    
    % last footstep
    footstep(1:4,N) = footstep(1:4,N-1) + [0; sign*spacing; 0; 0];
    
    % generate reference zmp
    for k = 1:N-1
       % set the same as foot position 
       footstep(7:8,k) = footstep(1:2,k); 
    end
    % last reference zmp is set to the middle of two feets
    footstep(7:8,N) = (footstep(1:2,N) + footstep(1:2,N-1))/2;
    
    % generate reference dcm
    % last dcm is set to the middle of footsteps
    footstep(5:6,N)   = (footstep(1:2,N) + footstep(1:2,N-1))/2;
    footstep(5:6,N-1) = (footstep(1:2,N) + footstep(1:2,N-1))/2;
    
    for k = N:-1:2
        footstep(5:6,k-1) = footstep(7:8,k-1) + exp(-step_duration/T)*(footstep(5:6,k) - footstep(7:8,k-1));
    end
    
    % initial dcp is set to the middle of footsteps
    footstep(5:6,1) = (footstep(1:2,1) + footstep(1:2,2))/2;
    
    % adjust initial zmp to make it consistent with initial dcm
    footstep(7:8,1) = (footstep(5:6,2) - exp(step_duration/T)*footstep(5:6,1))/(1 - exp(step_duration/T));
    
end
