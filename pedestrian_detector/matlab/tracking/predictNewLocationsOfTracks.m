function tracks=predictNewLocationsOfTracks(tracks)
for i = 1:length(tracks)
    bbox = tracks(i).bbox;
    
    % Predict the current location of the track.
    predictedCentroid = predict(tracks(i).centroidsKalmanFilter);
    predictedSize = predict(tracks(i).sizeKalmanFilter);

    % Shift the bounding box so that its center is at
    % the predicted location.
    predictedCentroid = (predictedCentroid) - bbox(3:4) / 2;
    tracks(i).bbox = [predictedCentroid,predictedSize];
end
end