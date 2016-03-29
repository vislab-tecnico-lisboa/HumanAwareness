function prob_image=imageProb(tracks,darp)

centroid_means=zeros(length(tracks),2);
centroid_variances=zeros(length(tracks),2);


size_means=zeros(length(tracks),2);
size_variances=zeros(length(tracks),2);

if ~isempty(tracks)
for i=1:length(tracks)
    centroid_means(i,:)=[tracks(i).centroidsKalmanFilter.State(1) tracks(i).centroidsKalmanFilter.State(3)];
    centroid_variances(i,:)=[...
        tracks(i).centroidsKalmanFilter.StateCovariance(1,1)...
        tracks(i).centroidsKalmanFilter.StateCovariance(2,2)];
    
    size_means(i,:)=[tracks(i).sizeKalmanFilter.State(1) tracks(i).sizeKalmanFilter.State(3)];
    size_variances(i,:)=[...
        tracks(i).sizeKalmanFilter.StateCovariance(1,1)...
        tracks(i).sizeKalmanFilter.StateCovariance(2,2)];
    
    %% Compute bounding box
%     size_thresholded=size_mean+...
%         [3*sqrt(size_variances(1,1)) 3*sqrt(size_variances(2,2))]+...
%         [3*sqrt(centroid_variances(1,1)) 3*sqrt(centroid_variances(2,2))];
%     roi=zeros(size_thresholded);
%     %% For this bounding box, get probabilities;
%     mean=size_thresholded/2.0;
%     for u=1:size_thresholded(1)
%         for v=1:size_thresholded(2)
%             roi(u,v)=gaussian2d([u v]',mean',centroid_variances);
%         end
%     end
end
compute_probabilities(darp,centroid_means',centroid_variances',size_means',size_variances');
end
end
