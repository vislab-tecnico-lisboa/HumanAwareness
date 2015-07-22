# Human Awareness_ROS

## ROS Nodes ##

-> detector: detects pedestrians on an image and sends the bounding boxes to the tracker.

subscribes to: /vizzy/l_camera/image_raw (é preciso mudar no vizzy real...)

advertises to: image_out (the image with printed detections that we can view on Rviz), detections (list of bounding boxes to be used by the tracker node)
	
	rosrun pedestrian_detector detector
	
-> tracker: receives the detections, computes the position of pedestrians, creates/associates trackers to each detection, filters the results and let's you choose a target to follow

subscribes to: detections

advertises to: person_position

It also uses INTERACTIVE MARKERS so that we can see each individual tracker position on Rviz and choose one to follow. So don't forget to add that on Rviz!
	
	rosrun pedestrian_detector tracker
	
-> follower: simply receives the target position from the tracker and follows it. Uses the ros navigation stack (eband_local_planner) with a sampling rate of 2Hz to start following persons that are at more than 3m and stops at aproximatelly 2.5m.

subscribes to: person_position

advertises to: move_base (well... it's actually ros actionlib that is advertising...)
	
	rosrun pedestrian_detector follower
	
-> viewer: ignore this... I made it before I knew ros image_view existed... -_-
	

## Camera instrinsics ##

In order to be able to calculate the position of a person, the tracker node needs the camera instrinsics. They are loaded from the following file: HumanAwareness_ROS/pedestrian_detector/camera_model/config.yaml .
The only thing that matters in that file is the camera_matrix. Ignore the remaining things... I'm not using them.


## Warning regarding people size! ##

I'm assuming people with a height of 1.7m. That value is defined on HumanAwareness_ROS/pedestrian_detector/src/tracker/cameraModel.cpp
(Yeah, it's #defined). So if you want to teste with other people heights try other values. Some day I will parse it (maybe) or abbandon it completelly and use stereo vision (if time is generous with me and my thesis writting skills are good).


Fica já escrito em inglês (com algumas calinadas) para quando isto for público.
	
## Warning regarding compilation flags ##

On CMakeLists.txt, line 51. Change that line to one of these according to your version of gcc (I will find a way to use an IF to do it later):

  gcc 4.6.x:
  SET(CMAKE_CXX_FLAGS_RELEASE " -lpthread -std=c++0x -Wall -O3")
  
  Tested on Ubuntu 12.04, fresh installed!
  
  gcc 4.8.x:
  SET(CMAKE_CXX_FLAGS_RELEASE " -lpthread -std=c++11 -Wall -O2 -finline-functions -fpredictive-commoning -fgcse-after-reload -ftree-slp-vectorize -ftree-loop-distribute-patterns -fipa-cp-clone -funswitch-loops -fvect-cost-model -ftree-partial-pre")

  Tested on Ubuntu 14.04

  (-std=c++11 vs -std=c++0x makes the difference)

  (why so many flags on 4.8? because -O3 uses the same flags of -O2 plus all these flags and another one that is bugged on 4.8 and makes my detector segfault, so I removed it...)