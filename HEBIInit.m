%% HEBI Initialization
% Download SCOPE from HEBI to see the names and families of the HEBI
% modules and your phone

% this initializes the hebi modules
wheels = HebiLookup.newGroupFromNames('Family of Robot',{'front left', 'rear left', 'front right', 'rear right'}); % Define new group with HEBI modules 

% this initiliazes your phone so you can use the HEBI app to control the
% motors
mobile = HebiLookup.newGroupFromNames('family of Phone',{'Name of Phone'}); % Define new group with mobile IO app
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cmd = CommandStruct();