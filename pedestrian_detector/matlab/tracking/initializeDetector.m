function detector = initializeDetector()
cd ..
current_directory=pwd;
cd matlab
conf_pedestrians = strcat(current_directory,'/configuration.xml');
conf_heads=strcat(current_directory,'/configurationheadandshoulders.xml');
detector_type='full';
path=current_directory;

detector = pedestrian_detector(conf_pedestrians,conf_heads,detector_type,path);
% create an empty array of detectors

end