close all
addpath('tracking');
addpath('resource_constraint');
% min size 128x52  
min_width=41;
min_height=100;
% x = [u v s u_v v_v s_v]
% tracking parameters
invisibleForTooLong = 10;
ageThreshold = 5;
minVisibleCount = 2;

state_transition_model=[1 0 0; 0 1 0; 0 0 1]; % constant position
state_measurement_model=[1 0 0; 0 1 0; 0 0 1];
state_init_state_covariance=[100 0 0; 0 100 0; 0 0 2.0];
state_process_noise=[50 0 0; 0 50 0; 0 0 1.0];
state_measurement_noise=[10 0 0; 0 10 0; 0 0 1.0];
costOfNonAssignmentState=120;

% optimization parameters    
capacity_constraint=0.3; % percentage of image to be process at each time instant
max_items=11;            % max regions to be process (To do) IT EXPLODES RIGHT NOW!!! FIX
time_horizon=2;          % planning time horizon (To do: now its 1 by default)

%% Create System objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects('dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
v = VideoReader('dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(read(v,1));
%% Initialize pedestrian detector
detector=initializeDetector();
 
%% Initialize trackers
tracks = initializeTracks(); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Initialize resource contraint policy optimizer
darap=initializeDARAP(frame_size(2), frame_size(1),capacity_constraint,max_items,min_width,min_height);
%% Detect moving objects, and track them across video frames.
detection_times=[];
optimization_times=[];
tracking_times=[];
while ~isDone(obj.reader)
    frame=readFrame(obj);
    width=size(frame,2);
    height=size(frame,1);
    x=width-width;
    y=height-height;
    
    %% dynamic resource allocation
    [rois,optimization_time]=imageProb(tracks,darap);
    optimization_times=[optimization_times optimization_time];
    
    probability_maps=get_probability_maps(darap);
%     for p=size(probability_maps,2)
%     figure(1),imagesc(probability_maps{1,p});
%     end
%rois=[];
    %% detection 
    [detection_centroids,... 
        detection_bboxes,...
        detection_time]=...
        detectObjects(detector,...
        uint8(frame*256),...
        x,y,width,height,rois);
    detection_times=[detection_times detection_time];
    %fps=1.0/time_elapsed
    
    %% tracking
    tracks=predictNewLocationsOfTracks(tracks);
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment(tracks,...
        detection_centroids,...
        detection_bboxes,...
        costOfNonAssignmentState);
    
    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes);
    tracks=updateUnassignedTracks(tracks,unassignedTracks);
    tracks=deleteLostTracks(tracks,...
        invisibleForTooLong,...
        ageThreshold);
    
    [tracks,nextId]=createNewTracks(tracks,...
        unassignedDetections,...
        detection_centroids,...
        detection_bboxes,...
        nextId,...
        state_transition_model,...
        state_measurement_model,...
        state_init_state_covariance,...
        state_process_noise,...
        state_measurement_noise);
    
    displayTrackingResults(obj,frame,tracks,detection_bboxes,minVisibleCount,rois);
    
end

average_optimization_time=mean(optimization_times);
average_detection_time=mean(detection_times);
average_total_time=average_optimization_time+average_detection_time;
average_frame_rate=1.0/average_total_time;

