function [rois,time_elapsed]=imageProb(tracks,darp)
rois=[];
time_elapsed=0.0;

centroid_means=zeros(length(tracks),2);
centroid_variances=zeros(length(tracks),2);


state_means=zeros(length(tracks),3);
state_variances=zeros(length(tracks),3);
upper_left_corner=zeros(length(tracks),2);
if ~isempty(tracks)
for i=1:length(tracks)
   
    state_means(i,:)=tracks(i).stateKalmanFilter.State';
    state_variances(i,:)=[...
        tracks(i).stateKalmanFilter.StateCovariance(1,1)...
        tracks(i).stateKalmanFilter.StateCovariance(2,2)...
        tracks(i).stateKalmanFilter.StateCovariance(3,3)];
    upper_left_corner(i,:)=tracks(i).bbox(1:2);
end
[rois,time_elapsed]=compute_probabilities(darp,state_means',state_variances',upper_left_corner');
rois=reshape(cell2mat(rois)',4,size(rois,2))';
end
end
