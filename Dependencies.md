## Moss Rock
Thiss package is a fork of the awesome MORB-SLAM, a fork of OS3 which turns it into a cmake package. I have not used this feature, but instead made use of their refactoring and bug fixing. If you are looking to use ORB-SLAM3 without ros (ie as a normal c/c++ import) please check it out. Many thanks to both MORB-SLAM and OS3's contributors! 


## ORB-SLAM3 v1.0

Below we list all the pieces of code used by ORB-SLAM3 and linked libraries which are not property of the authors of ORB-SLAM3.


##### Code in **src** and **include** folders

* *ORBextractor.cc*.
This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

* *PnPsolver.h, PnPsolver.cc*.
This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit. 
This code can be found in popular BSD licensed computer vision libraries as [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is FreeBSD.

* *MLPnPsolver.h, MLPnPsolver.cc*.
This is a modified version of the MLPnP of Steffen Urban from [here](https://github.com/urbste/opengv). 
The original code is BSD licensed.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

##### Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

* All code in **g2o** folder.
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

* All code in **Sophus** folder.
This is a modified version of [Sophus](https://github.com/strasdat/Sophus). [MIT license](https://en.wikipedia.org/wiki/MIT_License).

#####Library dependencies 

* **Pangolin (visualization and user interface)**.
[MIT license](https://en.wikipedia.org/wiki/MIT_License).

* **OpenCV**.
BSD license.

* **Eigen3**.
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.




