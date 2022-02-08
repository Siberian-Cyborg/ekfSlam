function EKF_SLAM()
    % this is the entire SLAM function

    clear;
    close all;
    clc;
    
    % enter the correct path here
    addpath("path to hebi matlab api functions");

    x_save=[];
    P_save=[];
    
    NumberOfTags=3;
    %% Setup Camera

    %Calibrate Camera to get camera intrinsics
    CameraCalibration;

    %camera intrinsics measured during calibration
    intrinsics = params.Intrinsics; 

    %Specify the tag size in meters.
    tagSize = 16*10^(-2);  %16cm

    % Camera Initialization
    % Create the ipcam object as cam
    CamInit;

    % Capture one frame to get its size.
    videoFrame = snapshot(cam);
    frameSize = size(videoFrame);
    % Create the video player object.
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);


    %% Setup Hebi Modules and I/O App
    % creates comand struct as cmd
    %A2_2aHEBIInit;
    HEBIInit;

    %% Transformations
    % Pose of Camera raltive to center of platform (vector, pitch)
    % assumme no roll and yaw
    % p_cam in cm [x;y;z]
    % pitch_cam in rad
    % pitch_cam in rad
    [p_cam, pitch_cam] = CamPose;

    % Vehicle parameters:
    %R Radius der Räder in cm
    %p Position der Räder in B (Plattformkoordinatensystem) in cm
    %a Ausrichtung der Rollen in Radkoordinatensystem in rad
    %g Ausrichtung der Räder in rad
    %n_wh Anzahl an Rädern
    [R,p,g,a,n_wh] = Mecanum4W; 

    % Calculates Tranformation Platform to Mecanum (PM)
    % Transformiert Geschwindigkeit (x_trans, y_trans, rot) der Plattform in
    % die nötigen Raddrehzahlen
    T_PM =PlatformMecanumTransform(R,p,g,a);

    % Calculates Tranformation Camera to Platform (CP)
    T_CP = CameraPlatformTransform(p_cam, pitch_cam);

    %% 1. define Variables
    x = zeros(3 + 2*NumberOfTags, 1); % state vector x
    P = zeros(3 + 2*NumberOfTags); % covariance matrix P
    
    %initialize covariance of landmarks to a high number
    for i = 1:NumberOfTags
        idx = (3+2*i-1):(3+2*i);
        P(idx,idx)=10000*eye(2);
    end
    
    table = zeros(NumberOfTags,1); % table to keep track which tags have been seen

    % observation noises
    sigmaR= 0.001; % metres
    sigmaB= (0.01*pi/180); % radians
    R= [sigmaR^2 0; 0 sigmaB^2];

    % control noises on global controls
    sigmaVx= 0.1; % m/s
    sigmaVy= 0.8; % m/s
    sigmaG= (1*pi/180); % radians
    Q= [sigmaVx^2 0 0;0 sigmaVy^2 0; 0 0 sigmaG^2];
    
    % control noise per wheel
    sigmaW1= (3.0*pi/180); % radians/s
    sigmaW2= (3.0*pi/180); % radians/s
    sigmaW3= (3.0*pi/180); % radians/s
    sigmaW4= (3.0*pi/180); % radians/s
    Q_wheels= [sigmaW1^2 0 0 0;0 sigmaW2^2 0 0; 0 0 sigmaW3^2 0; 0 0 0 sigmaW4^2];
    
    

    % control parameters
    V = 50; % 150 Maximum translational speed
    OMEGA = 1; % 100 Maximum rotational speed
    WHEELBASE= 0.4822; % metres, vehicle wheel-base
    TRACK= 0.2; % metres, vehicle wheel-base

    % others
    %use time from hebi feedback and update it within the while loop
    % dt = 0.025; %time step for prediction
    t_old = 0;
    
    %% Init the Animation
    % setup plots
    fig=figure;
    hold on, axis equal
    axis([-10,10,-10,10]);
    xlabel('metres'), ylabel('metres')
    set(fig, 'name', 'EKF-SLAM Simulator')
    h= setup_animations;
    % triangle as car
    triangle= [WHEELBASE/2 -WHEELBASE/2 -WHEELBASE/2; 
                   0          -TRACK/2     TRACK/2]; 

    
    % some bools
    runLoop = true;
    exitFlag = false;
    use_feedback = true;
    
    %% EKF Loop
    %while runLoop && ~exitFlag
    while runLoop && ~exitFlag
        %% 2. Observe the enviroment
        % here all the camera related stuff happens
        % we can probably copy a lot from FollowTag.m
        % OUTPUT: z: Array of range bearing measurements
        %         tag_id: Array of corresponding ids for each tag

        [z, z_cart, tag_id, runLoop] = observe(videoPlayer, videoFrame, intrinsics, tagSize, T_CP, cam);
        
        %% 3. Move the robot
        % Read app inputs: 
        % A8 defines velocity in x, A7 velocity y direction,
        % A1 angular velocity in z, B1 stops program
        fbkIO = mobile.getNextFeedbackIO();
        v_Bx = sum(V * fbkIO.a8,'omitnan');
        v_By = sum(V * fbkIO.a7,'omitnan');
        omega_B = sum(-OMEGA * fbkIO.a1,'omitnan');
%         v_Bx = sum(V * fbkIO.a5,'omitnan');
%         v_By = sum(V * fbkIO.a6,'omitnan');
%         omega_B = sum(-OMEGA * fbkIO.a3,'omitnan');
        exitFlag = sum(fbkIO.b1,'omitnan');
        
        % Show feedback
        %fprintf('v_Bx\t\t%f\nv_By\t\t%f\nomega_B\t\t%f\n\n',v_Bx,v_By,omega_B);

        % velocities of vehicle in coordinate system B
        q_dot = [v_Bx; v_By; omega_B];
        
        % Calculate motor speed for every wheel with transformation matrix
        omega_wh = T_PM * q_dot;

        % HEBI updaten
        cmd.velocity = omega_wh';
        %disp(cmd.velocity);
        wheels.send(cmd);
        wi = wheels.getNextFeedback.velocity; 
    
        %% 4. EKF Prediction
        % predict next pose of robot given the controls and the current pose
        % also predic the uncertainty P of the next pose
        
         % Receive feedback and computing time difference
        fbk = wheels.getNextFeedback;    % get next feedback from module (check to see how does this work with many modules)
        t = fbk.time; % get time form feedback struct
        dt = t-t_old;                   % compute time from last to current feedback
        t_old = t;                      % save current time     
        
        if dt>3
            dt=3;
        end
        % Show time step
        fprintf('dt\t\t%f\n\n',dt);
        
        %only do a prediction step if a command is given
        if q_dot == zeros(size(q_dot)) 
            fprintf("command is zero\n")
        else
            if ~use_feedback
                [x,P] = EKF_prediction1(x, P, q_dot, Q, dt);
            else
                [x,P] = EKF_prediction2(x, P, wi, Q_wheels, dt, T_PM);
            end
        end
        
        
        %% 5. EKF Correction
         if isempty(tag_id)
             disp("no observations\n")
         else
             for i = length(tag_id) % loop thru all detected april tags

                 % tag id of the current observation
                 j = tag_id(i) + 1; %+1 becauase april tags start with id 0
                 % index this tag has in x and P
                 idx = (3+2*j-1):(3+2*j);

                 % have we already seen that tag or is it a new one?
                 [observed, table] = associate_tag(j,table, idx);

                 if ~observed % if we haven't observed the tag before
                    % add newly detected tag to state vector
                    x = add_landmark(x, z(:,i),z_cart(:,i), idx);
                 end

                 % update the position and covariance of the tag
                 [x,P]= update_landmark(x, P, z(:,i), R, idx);
            end %correction
         end
        %% 6. Visualization
        % plot everything to see the map and the pose of the robot
        %pcov(:,1:size(ptmp,2))= ptmp;
        %pcov = EKF_visualize(x, P, triangle, h, z, pcov);
        %EKF_visualize2(x, NumberOfTags, triangle);
        %EKF_visualize2(x, table, triangle);
        %ptmp= make_covariance_ellipses(x(1:3),P(1:3,1:3));
        %pcov(:,1:size(ptmp,2))= ptmp;
        %pcov = EKF_visualize(x, P, table, triangle, h, tag_id, pcov);
        x_save = [x_save, x];
        P_save = [P_save, zeros(size(P,1),1), P];
        save("State.mat", 'x_save', 'P_save');
        EKF_visualize(x, P, table, triangle, h);
    end %while loop
end %function
