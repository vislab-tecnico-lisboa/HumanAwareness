close all
addpath('tracking');
initial_estimate_error=[400, 100];
motion_noise=[100, 25];
measurement_noise=100;
%% create video from images
% outputVideo = VideoWriter(fullfile(pwd,'dataset/cvpr10_tud_stadtmitte.avi'));
% outputVideo.FrameRate = 25;
% open(outputVideo)
%
% for i = 1 : length(srcFiles)
%
%     filename = strcat('dataset/cvpr10_tud_stadtmitte/',srcFiles(i).name);
%     I = imread(filename);
%     writeVideo(outputVideo,I)
% end
% close(outputVideo)
%% Create System objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects('dataset/cvpr10_tud_stadtmitte.avi');

%% Initialize pedestrian detector
detector=initializeDetector();

%% Prepare a figure to show the images in the upper half of the screen.
figure(1);
screenSize = get(0, 'ScreenSize');

%% Initialize trackers
tracks = initializeTracks(); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Detect moving objects, and track them across video frames.
while ~isDone(obj.reader)
    frame=readFrame(obj);
    width=size(frame,2);
    height=size(frame,1);
    x=width-width;
    y=height-height;
    [detection_centroids, detection_bboxes, time_elapsed]=...
        detectObjects(detector,...
        uint8(frame*256),...
        x,y,width,height);
    %fps=1.0/time_elapsed
    
    tracks=predictNewLocationsOfTracks(tracks);
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment(tracks,detection_centroids);
    
    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes);
    tracks=updateUnassignedTracks(tracks,unassignedTracks);
    deleteLostTracks(tracks);
    tracks=createNewTracks(tracks,...
        unassignedDetections,...
        detection_centroids,...
        detection_bboxes,...
        nextId,...
        initial_estimate_error,...
        motion_noise,...
        measurement_noise);
    displayTrackingResults(obj,frame,tracks,detection_bboxes);

end
