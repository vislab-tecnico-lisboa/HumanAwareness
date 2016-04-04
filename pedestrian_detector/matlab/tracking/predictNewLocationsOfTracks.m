function tracks=predictNewLocationsOfTracks(tracks)
for i = 1:length(tracks)
    bbox = tracks(i).bbox;
    
    % Predict the current location of the track.
    predictedState = predict(tracks(i).stateKalmanFilter);

    % Shift the bounding box so that its center is at
    % the predicted location.
    predictedState(1:2) = predictedState(1:2) - bbox(3:4) / 2;
    tracks(i).bbox(1:2) = predictedState(1:2);
end
end