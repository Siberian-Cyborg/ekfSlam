function [z, pose_plat, tag_id, runLoop] = observe(videoPlayer, videoFrame, intrinsics, tagSize, T_CP, cam)
%function [z, tag_id] = observe(videoPlayer, videoFrame, intrinsics, tagSize, T_CP)
%
% Inputs:
%   videoPlayer - video player object
%   videoFrame - current frame
%   intrinsics - intrinsics of the camera we got thru calibration
%   tagSize - size in meters of the tags we detect
%   T_CP - 4x4 Transfromation matrix from Camera to Plattform
%
% Outputs: 
%   z - 2xN matrix with the range-bearing information of each tag in
%       plattfrom coordinate system
%   pose_plat - 3xN matrix with the x,y,z information of each tag in
%       plattfrom coordinate system
%   tag_id - N-element vector of tag IDs
%   runLoop - boolean, true if video player is open, false once its closed
    
    videoFrame = snapshot(cam);
    step(videoPlayer, videoFrame);
    % if we close the videoPlayer runLoop will be false and the program
    % will quit
    runLoop = isOpen(videoPlayer);
    
    I = videoFrame;
    I = undistortImage(I,intrinsics,"OutputView","same");
    
    %Detect a specific family of AprilTags and estimate the tag poses
    %pose is a rigidbody3d object see "help readAprilTag"
    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    
    tag_id = id;
    
    %TODO check id lenth(id) is equal to length(pose)
    % number of observed tags
    M = length(id); 
    
    %convert cartesian pose in camera system into range-bearing measuremtn in plattform system
    %poses in plattfrom coordinates
    
    %first convert cartesian pose from camera to plattform
    pose_vec = zeros(3,length(pose));
    for i= 1:length(pose)
        pose_vec(:,i) = pose(i).Translation';
    end
    
    
    pose_plat = T_CP * [pose_vec; ones(1, M)];
   
    %convert cartesian into range-bearing
    z = zeros(2,M);
    
    for i = 1:M
        range = sqrt(pose_plat(1,i)^2 + pose_plat(2,i)^2); 
        bearing = pi_to_pi(atan2(pose_plat(2,i),pose_plat(1,i)));
        z(:,i) = [range; bearing];
    end

end