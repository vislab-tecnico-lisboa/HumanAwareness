# Human Awareness_ROS

## ROS Nodes ##

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
