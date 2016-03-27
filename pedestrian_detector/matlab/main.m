srcFiles = dir('dataset/cvpr10_tud_stadtmitte/*.png');
T=length(srcFiles);
mov = struct('cdata', cell(1,T+1), 'colormap', cell(1,T+1));
for i = 1 : length(srcFiles)
    
    filename = strcat('dataset/cvpr10_tud_stadtmitte/',srcFiles(i).name);
    I = imread(filename);
    mov(i)=im2frame(I);
    %figure, imshow(I);
end
cd ..
current_directory=pwd;
cd matlab
conf_pedestrians = strcat(current_directory,'/configuration.xml');
conf_heads=strcat(current_directory,'/configurationheadandshoulders.xml');
detector_type='full';
path=current_directory;
pedestrian_detector_ = pedestrian_detector(conf_pedestrians,conf_heads,detector_type,path);

% Prepare a figure to show the images in the upper half of the screen.
figure(1);
screenSize = get(0, 'ScreenSize');
%newWindowPosition = [1 screenSize(4)/2 - 70 screenSize(3) screenSize(4)/2];
%set(gcf, 'Position', newWindowPosition); % Maximize figure.
% 	set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.

for i=1:length(srcFiles)
    [bounding_boxes, time_elapsed]=detect(pedestrian_detector_,mov(i).cdata);
    
    figure(1),imshow(mov(i).cdata);
    hold on
    for b=1:length(bounding_boxes)
        rectangle('Position', [bounding_boxes(b).x, bounding_boxes(b).y, bounding_boxes(b).width, bounding_boxes(b).height],...
            'EdgeColor','r', 'LineWidth', 3)
    end
    hold off
    drawnow
end
