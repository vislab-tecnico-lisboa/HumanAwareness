function [assignments, unassignedTracks, unassignedDetections] = ...
    detectionToTrackAssignment(...
    tracks,...
    centroids,...
    detection_bboxes,costOfNonAssignmentCentroid,...
    costOfNonAssignmentSize)

nTracks = length(tracks);
nDetections = size(centroids, 1);
if nDetections == 0
    assignments=[];
    unassignedTracks=[];
    unassignedDetections=[];
    return
end
% Compute the cost of assigning each detection to each track (centroid).
cost = zeros(nTracks, nDetections);
for i = 1:nTracks
    cost(i, :) = distance(tracks(i).centroidsKalmanFilter, centroids);
end

% Solve the assignment problem for the centroid
[assignments, unassignedTracks, unassignedDetections] = ...
    assignDetectionsToTracks(cost, costOfNonAssignmentCentroid);



% Compute the cost of assigning each detection to each track (size).
size_cost = zeros(nTracks, nDetections);
for i = 1:nTracks
    size_cost(i, :) = distance(tracks(i).sizeKalmanFilter, detection_bboxes(:,3:4));
end

% Solve the assignment problem for the size
[size_assignments, size_unassignedTracks, size_unassignedDetections] = ...
    assignDetectionsToTracks(size_cost, costOfNonAssignmentSize);

assignments=intersect(size_assignments,assignments,'rows');
unassignedTracks=union(unassignedTracks, size_unassignedTracks);
unassignedDetections=union(unassignedDetections, size_unassignedDetections);

end