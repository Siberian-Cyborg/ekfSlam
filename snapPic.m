clear; close all;
% Camera Initialization
CamInit

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);
% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true; % flag for checking if user stops program by closing camera window
exitFlag = 0; % flag for checking if user stops program in app (B1)
i = 1;

picsPath =  fullfile(pwd,'Photos');
if ~exist(picsPath, 'dir')
    % Create folder if not existing
    mkdir(picsPath)
else
    filePattern = fullfile(picsPath, '*.png');
    theFiles = dir(filePattern);
    % Delete all existing .png files in the folder
    for k = 1 : length(theFiles)
      baseFileName = theFiles(k).name;
      fullFileName = fullfile(picsPath, baseFileName);
      % fprintf(1, 'Now deleting %s\n', fullFileName);
      delete(fullFileName);
    end
end
% loop
while  runLoop && ~exitFlag && i < 2
    
    % Get the ext frame.
    videoFrame = snapshot(cam);

    picsSavePath = fullfile(picsPath, sprintf('Image%03d.png',i));
        
    % Save the point cloud as pcd and image in png format
    % in the given locations
    imwrite(videoFrame,picsSavePath);
    i = i + 1;

    % Display the annotated video frame using the video player
    % object
    step(videoPlayer, videoFrame);
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    
    % wait 3s
    pause(3);
    
end
