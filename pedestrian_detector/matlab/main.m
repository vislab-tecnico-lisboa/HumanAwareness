close all
addpath('tracking');

invisibleForTooLong = 4;
ageThreshold = 4;
minVisibleCount = 4;

centroid_initial_estimate_error=[400, 100];
centroid_motion_noise=[50, 50];
centroid_measurement_noise=15;
costOfNonAssignmentCentroid = 30;

size_initial_estimate_error=[400, 100];
size_motion_noise=[20, 20];
size_measurement_noise=15;
costOfNonAssignmentSize=20;

%% Create System objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects('dataset/cvpr10_tud_stadtmitte.avi');

%% Initialize pedestrian detector
detector=initializeDetector();

%% Initialize trackers
tracks = initializeTracks(); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Initialize resource contraint policy optimizer

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
        detectionToTrackAssignment(tracks,...
        detection_centroids,detection_bboxes,...
        costOfNonAssignmentCentroid,...
        costOfNonAssignmentSize);
    
    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes);
    tracks=updateUnassignedTracks(tracks,unassignedTracks);
    deleteLostTracks(tracks,...
        invisibleForTooLong,...
        ageThreshold);
    [tracks,nextId]=createNewTracks(tracks,...
        unassignedDetections,...
        detection_centroids,...
        detection_bboxes,...
        nextId,...
        centroid_initial_estimate_error,...
        centroid_motion_noise,...
        centroid_measurement_noise,...
        size_initial_estimate_error,...
        size_motion_noise,...
        size_measurement_noise);
    displayTrackingResults(obj,frame,tracks,detection_bboxes,minVisibleCount);
    
end
