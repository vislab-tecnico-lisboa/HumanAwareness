function displayTrackingResults(obj,frame,tracks, detection_bboxes, minVisibleCount)
% Convert the frame and the mask to uint8 RGB.
frame = im2uint8(frame);
detector_frame=frame;
tracking_frame=frame;

if ~isempty(tracks)
    
    % Noisy detections tend to result in short-lived tracks.
    % Only display tracks that have been visible for more than
    % a minimum number of frames.
    reliableTrackInds = ...
        [tracks(:).totalVisibleCount] > minVisibleCount;
    reliableTracks = tracks(reliableTrackInds);
    
    % Display the objects. If an object has not been detected
    % in this frame, display its predicted bounding box.
    if ~isempty(reliableTracks)
        % Get bounding boxes.
        bboxes = cat(1, reliableTracks.bbox);
        
        % Get ids.
        ids = int32([reliableTracks(:).id]);
        
        % Create labels for objects indicating the ones for
        % which we display the predicted rather than the actual
        % location.
        tracker_labels = cellstr(int2str(ids'));
        predictedTrackInds = ...
            [reliableTracks(:).consecutiveInvisibleCount] > 0;
        isPredicted = cell(size(tracker_labels));
        isPredicted(predictedTrackInds) = {' predicted'};
        tracker_labels = strcat(tracker_labels, isPredicted);
        
        % Draw the detections on the frame.
        detector_frame = insertObjectAnnotation(frame, 'rectangle', ...
            detection_bboxes, cellstr(int2str([1:size(detection_bboxes,1)]')),...
            'Color','red');
        
        % Draw the trackers on the frame.
        tracking_frame = insertObjectAnnotation(frame, 'rectangle', ...
            bboxes, tracker_labels,'Color','green');
    end
end

obj.detectorPlayer.step(detector_frame);
obj.trackerPlayer.step(tracking_frame);

end