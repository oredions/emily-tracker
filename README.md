# Visual Navigation of Unmanned Surface Vehicle from Unmanned Aerial Vehicle to Safe Drowning Victims
This project uses the video from a small unmanned aerial system (UAS) to navigate an unmanned surface vehicle (USV) covered in a flotation jacket to reach drowning victims. The teleoperated [EMILY](https://www.emilyrobot.com/) USV has been used by the Hellenic Coast Guard since January of 2016 to allow lifeguards to rapidly deploy flotation to refugees attempting to cross the Mediterranean Sea. While EMILY has been credited with the successful rescue of at least one boat load of refugees, there are three problems. First, teleoperation takes lifeguard time and energy that would be better spent on directly rescuing high risk victims. Second, the lifeguards have trouble teleoperating the USV because their viewing angle of the USV heading away from them is unfavorable; as a result the USV often fish-tails and takes a sub-optimal path. Third, the lifeguards quickly lose depth perception and may accidentally hit the victim with the USV. This project addresses these deficiencies by using the output from a small UAS to direct the USV.

The implementation relies on a robust vision-based algorithm for position and orientation estimation of the USV based on the CamShift algorithm. The CamShift algorithm is applied to the USV’s histogram backprojection to estimate the position and projection of the USV’s trajectory filtered by Douglas-Peucker algorithm to estimate the orientation. Rudder and throttle control signals for the USV to reach the target selected in the video are computed using the line-of-sight and PID control. Current work is enabling the operator to select victims in the video feed provided by a tethered UAS called a [Fotokite Pro](https://fotokite.com/fotokite-pro/) and the system then navigates the USV to the victims autonomously. The Fotokite can operate from a shore or from a boat without the need to worry about the control. The UAS controlling the USV was tested with several first responder agencies such as United States Cost Guard, Italian Cost Guard, Los Angeles County Fire Department Lifeguards, and Department of Homeland Security during several exercises. A tracking error, progression of error angle to the target, distance to the target, and cross track error were measured. The mean tracking error was 0.3 %. The mean cross track error for the latest version was 1 m. The experimental results indicated feasibility of the system. The results are expected to be made available to lifeguards in Greece and the Frontex agencies assisting with the rescues.

## Publications

Details about the project can be found in the following IEEE publications:

[Visual pose estimation of USV from UAV to assist drowning victims recovery](http://ieeexplore.ieee.org/document/7784291/)

[UAV assisted USV visual navigation for marine mass casualty incident response](http://ieeexplore.ieee.org/document/8206510/)

[Visual pose stabilization of tethered small unmanned aerial system to assist drowning victim recovery](http://ieeexplore.ieee.org/document/8088149/)

[Visual servoing of unmanned surface vehicle from small tethered unmanned aerial vehicle](https://arxiv.org/abs/1710.02932)

## Installation on macOS

1. Install Homebrew:

    /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

2. Install OpenCV with FFmpeg support:

    brew tap homebrew/science

    brew install opencv3 --with-contrib --with-ffmpeg --with-tbb --with-qt5

3. Link OpenCV:

    brew link --force --override opencv3

4. Install CMake:

    https://cmake.org

5. Delete the following files from the root directory if they exist (those are system specific files that you will have to generate again on your system):

    cmake_install.cmake

    CMakeCache.txt

    CMakeFiles

    EMILYTracker

    Makefile

6. In terminal, change directory into the root directory of the project and run the following command to generate makefile:

    cmake .

3. Compile the project:

    make

8. Set up network for USB ethernet adapter:

    IP Address: 192.168.1.3

    Subnet Mask: 255.255.255.0

## Manual

The graphical user interface has the following functionality:

* Select the USV by holding CTRL key and making the selection using left mouse button.

* Select the target by left mouse button double click.

* Zoom with mouse wheel or touchpad scroll.

* Drag with left mouse button.

* Show histogram backprojection view by pressing b key. Switch back by pressing b again.

* Pause the video feed by pressing p key.

* Press escape key to exit.

* Use the sliders to adjust program parameters.

As soon as both the USV and the target are selected the USV will start navigating autonomously to the target. Both the USV and the target can be reselected online.