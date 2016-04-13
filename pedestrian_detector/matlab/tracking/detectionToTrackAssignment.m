function [assignments, unassignedTracks, unassignedDetections] = ...
    detectionToTrackAssignment(...
    tracks,...
    centroids,...
    detection_bboxes,costOfNonAssignmentState)

nTracks = length(tracks);
nDetections = size(centroids, 1);
% if nDetections == 0
%     assignments=[];
%     unassignedTracks=[];
%     unassignedDetections=[];
%     return
% end
% Compute the cost of assigning each detection to each track (centroid).

cost = zeros(nTracks, nDetections);
if nDetections >0 
for i = 1:nTracks
    cost(i, :) = distance(tracks(i).stateKalmanFilter, [detection_bboxes(:,1:2) detection_bboxes(:,4)./detection_bboxes(:,3)]);
end
end
% Solve the assignment problem for the centroid
[assignments, unassignedTracks, unassignedDetections] = ...
    assignDetectionsToTracks(cost, costOfNonAssignmentState);

% assignments=intersect(size_assignments,assignments,'rows');
% unassignedTracks=union(unassignedTracks, size_unassignedTracks);
% unassignedDetections=union(unassignedDetections, size_unassignedDetections);

end