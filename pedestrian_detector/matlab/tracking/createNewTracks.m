function [tracks,nextId]=createNewTracks(tracks,...
    unassignedDetections,...
    centroids,...
    bboxes,...
    nextId,...
    centroid_initial_estimate_error,...
    centroid_motion_noise,...
    centroid_measurement_noise,...
    size_transition_model,...
    size_measurement_model,...
    size_init_state_covariance,...
    size_process_noise,...
    size_measurement_noise)
centroids = centroids(unassignedDetections, :);
bboxes = bboxes(unassignedDetections, :);

for i = 1:size(centroids, 1)
    
    centroid = centroids(i,:);
    bbox = bboxes(i, :);
    
    % Create a Kalman filter object for centroids.
    centroidsKalmanFilter = configureKalmanFilter('ConstantVelocity', ...
        centroid,...
        centroid_initial_estimate_error, centroid_motion_noise, centroid_measurement_noise);
    % Create a Kalman filter object for centroids.
%     sizeKalmanFilter = configureKalmanFilter('ConstantVelocity', ...
%         bbox(3:4),...
%         size_initial_estimate_error, size_motion_noise, size_measurement_noise);
    sizeKalmanFilter=vision.KalmanFilter(...
        size_transition_model,...
        size_measurement_model,...
        'State',...
        bbox(3:4),...
        'StateCovariance',...
        size_init_state_covariance,...
        'ProcessNoise',...
        size_process_noise,...
        'MeasurementNoise',...
        size_measurement_noise);

    % Create a new track.
    newTrack = struct(...
        'id', nextId, ...
        'bbox', bbox, ...
        'centroidsKalmanFilter', centroidsKalmanFilter, ...
        'sizeKalmanFilter', sizeKalmanFilter,...
        'age', 1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0);
    
    % Add it to the array of tracks.
    tracks(end + 1) = newTrack;
    
    % Increment the next id.
    nextId = nextId + 1;
end
end