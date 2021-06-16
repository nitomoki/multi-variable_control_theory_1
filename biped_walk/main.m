global playing

playing = 1;

% parameters
h             = 1.0;
g             = 9.8;
step_duration = 0.5;
foot_size     = [0.2; 0.1];
swing_height  = 0.03;

% constant of LIPM
T = sqrt(h/g);

% time step
dt = 0.01;

%
animation_speed = 1;

% genereate walking pattern
footstep = generate_walking_pattern(step_duration, T);

% number of footsteps
N = size(footstep,2);

% variables
sup   = 1;     % current support foot  1:right  2:left
swg   = 2;     % current swing foot    1:right  2:left

k          = 2;  % current footstep index
t          = 0;  % elapsed time since last foot exchange
t_total    = 0;  % total elapsed time
foot       = [0 0  % foot pose
              0 0  % rows:    x y z theta
              0 0  % columns: right left
              0 0];
p_com      = [0; 0];
v_com      = [0; 0];
a_com      = [0; 0];
p_zmp      = [0; 0];
p_com_ref  = [0; 0];
p_dcm_ref  = footstep(5:6,k);
p_zmp_ref  = footstep(7:8,k);

x = [p_com(1)
     v_com(1)
     p_com(2)
     v_com(2)];
u = p_zmp;

%
% linear system
%
A = [0     1     0     0
     1/T^2 0     0     0
     0     0     0     1
     0     0     1/T^2 0];
B = [ 0       0
     -1/T^2   0
      0       0
      0      -1/T^2];

%
% design feedback gain
%
Q = diag([1 1 1 1]);
R = diag([10000 10000]);
[K, P, e] = lqr(A, B, Q, R);

% open canvas
canvas = figure('ButtonDownFcn', @on_click);

% Axes settings
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);
zlabel('z','FontSize',14);
set(gca,'FontSize',14);
axis vis3d equal;
% set plot range
axis([-0.5 1.5 -0.5 0.5 0.0 1.5]);
% set view angle
view([142.5,30]);
camlight;
grid on;

[xsphere, ysphere, zsphere] = sphere(12);
sphere_patch = surf2patch(xsphere, ysphere, zsphere, zsphere);
h_trans_com     = hgtransform;
h_trans_zmp     = hgtransform;
h_trans_foot_r  = hgtransform;
h_trans_foot_l  = hgtransform;
h_patch_com_line = patch('XData', [0.0 0.0], 'YData', [0.0 0.0], 'ZData', [0.0 0.0], 'EdgeColor', [0.0, 0.0, 0.0]);
h_patch_com      = patch(sphere_patch, 'EdgeColor', [0.5 0.5 0.5], 'FaceColor', [0.5 0.5 0.5], 'Parent', h_trans_com);
h_patch_zmp      = patch(sphere_patch, 'EdgeColor', [1.0 0.0 0.0], 'FaceColor', [1.0 0.0 0.0], 'Parent', h_trans_zmp);
h_patch_foot_r   = patch('XData', [-foot_size(1)/2 -foot_size(1)/2 foot_size(1)/2 foot_size(1)/2], 'YData', [-foot_size(2)/2 foot_size(2)/2 foot_size(2)/2 -foot_size(2)/2], 'FaceColor', 'none', 'Parent', h_trans_foot_r);
h_patch_foot_l   = patch('XData', [-foot_size(1)/2 -foot_size(1)/2 foot_size(1)/2 foot_size(1)/2], 'YData', [-foot_size(2)/2 foot_size(2)/2 foot_size(2)/2 -foot_size(2)/2], 'FaceColor', 'none', 'Parent', h_trans_foot_l);

for i=1:10000
    if ~ishghandle(canvas)
        break;
    end
    
    % 
    %if i == 2
    %    input('press enter to start');
    %end
    
    if playing == 1
        % support foot pose
        foot(:,sup) = footstep(1:4,k);

        if k < N
            % swing foot pose is expressed as a cycloid curve connecting two footsteps
            s  = t/step_duration;
            % ch and cv are horizontal and vertical components of the cycloid curve
            ch = (s   - sin(2*pi*s)/(2*pi));
            cv = (1.0 - cos(2*pi*s))/2.0;
            foot(1:2,swg) = (1-ch)*footstep(1:2,k-1) + ch*footstep(1:2,k+1);
            foot(3,swg)   = cv*swing_height;
            foot(4,swg)   = footstep(4,k-1) + ch*wrap_pi(footstep(4,k+1) - footstep(4,k-1));
        else
            foot(1:4,swg) = footstep(1:4,k-1);
        end
        
        % update reference state using dcm equation
        p_com_ref = p_com_ref - (1/T)*(p_com_ref - p_dcm_ref)*dt;
        p_dcm_ref = p_dcm_ref + (1/T)*(p_dcm_ref - p_zmp_ref)*dt;
        
        % calc reference CoM velocity from reference DCM
        v_com_ref = (p_dcm_ref - p_com_ref)/T;

        % reference state
        xref = [p_com_ref(1)
                v_com_ref(1)
                p_com_ref(2)
                v_com_ref(2)];

        % reference input
        uref = [p_zmp_ref(1)
                p_zmp_ref(2)];
            
        % state feedback
        u = K*(xref - x) + uref;

        % calculate force distribution
        %[force, ulim] = calc_force(u, cur_foot_pos, cur_foot_ori, phase{cur_phase}, foot_size, 1);
        %u = ulim;

        % state update with noise
        %v = [0; 0; 0; 0];
        v = generate_noise(t_total);

        % update state
        xd = A*x + B*u + v;
        x = x + xd*dt;
        
        h_patch_com_line.XData = [u(1) x(1)];
        h_patch_com_line.YData = [u(2) x(3)];
        h_patch_com_line.ZData = [0    h   ];
        h_trans_com.Matrix     = makehgtform('translate', [x(1) x(3) h], 'scale', 0.05);
        h_trans_zmp.Matrix     = makehgtform('translate', [u(1) u(2) 0], 'scale', 0.01);
        h_trans_foot_r.Matrix  = makehgtform('translate', foot(1:3, 1), 'zrotate', foot(4,1));
        h_trans_foot_l.Matrix  = makehgtform('translate', foot(1:3, 2), 'zrotate', foot(4,2));
        
        t       = t       + dt;
        t_total = t_total + dt;

        % duration of the current phase elapsed
        if t >= step_duration
            % reset local time
            t = 0;

            % increment step index
            if k < N
                k = k + 1;
            end

            % switch foot
            if sup == 1
                sup = 2;
                swg = 1;
            else
                sup = 1;
                swg = 2;
            end
            
            % reset reference dcm and reference zmp
            p_dcm_ref = footstep(5:6, k);
            p_zmp_ref = footstep(7:8, k);            
        end
    end
    
    %axis([-1.0,1.0,-1.0,1.0]);
    drawnow;

    pause(dt/animation_speed);
end
