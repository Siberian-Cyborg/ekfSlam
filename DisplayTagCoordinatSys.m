function [id,loc,pose] = DisplayTagCoordinatSys(videoFrame,intrinsics, tagSize)
%DISPLAYTAGCOORDINATSYS Summary of this function goes here
    I = videoFrame;
    I = undistortImage(I,intrinsics,"OutputView","same");

    %Detect a specific family of AprilTags and estimate the tag poses.

    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);

    %Set the origin for the axes vectors and for the tag frames.

    worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

    %Add the tag frames and IDs to the image.

    for i = 1:length(pose)
    % Get image coordinates for axes.
        imagePoints = worldToImage(intrinsics,pose(i).Rotation, ...
                  pose(i).Translation,worldPoints);

    % Draw colored axes.
        I = insertShape(I,"Line",[imagePoints(1,:) imagePoints(2,:); ...
            imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
            "Color",["red","green","blue"],"LineWidth",1);

        I = insertText(I,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",2);
    end

%Display the annotated image.

    imshow(I)
end

