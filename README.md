# ofxPCL

Windows VS is not supported as PCL does not support latest VS.

As file tree is too large, currently, it is not recommended to use the project generator, so please follow the instructions below; otherwise, copy `example_empty` to your app directory.

## Setup instruction ( Visual studio 2015 )

1. Get dependencies (69.9 MB) and extract them to ofxPCL folder.

https://app.box.com/s/gdxfvpeuc23wjqk4utb5fn1hc34o3ivu

2. Add folder ofxPCL/src and add files in your solution in Visual Studio

3. 
In Property Pages > C/C++ > Additional Include Directories add

		..\..\..\addons\ofxPCL\libs\pcl\include\;
		..\..\..\addons\ofxPCL\libs\pcl\include\eigen3;
		..\..\..\addons\ofxPCL\libs\pcl\include\pcl-1.7;
		..\..\..\addons\ofxPCL\src;

4.
In Property Pages > Linker add 	
		..\..\..\addons\ofxPCL\libs\pcl\lib\vs;

5.
In Property Pages > C/C++ > Preprocessor add
		BOOST_ALL_DYN_LINK;

6. Select Configuration Debug
In Property Pages > Linker > Input > Additional Dependencies add
		pcl_common_debug.lib;pcl_features_debug.lib;pcl_filters_debug.lib;pcl_io_debug.lib;pcl_io_ply_debug.lib;pcl_kdtree_debug.lib;pcl_keypoints_debug.lib;pcl_octree_debug.lib;pcl_recognition_debug.lib;pcl_registration_debug.lib;pcl_sample_consensus_debug.lib;pcl_search_debug.lib;pcl_segmentation_debug.lib;pcl_surface_debug.lib;pcl_tracking_debug.lib;qhull.lib;

Select Configuration Release
In Property Pages > Linker > Input > Additional Dependencies add
		pcl_common_release.lib;pcl_features_release.lib;pcl_filters_release.lib;pcl_io_release.lib;pcl_io_ply_release.lib;pcl_kdtree_release.lib;pcl_keypoints_release.lib;pcl_octree_release.lib;pcl_recognition_release.lib;pcl_registration_release.lib;pcl_sample_consensus_release.lib;pcl_search_release.lib;pcl_segmentation_release.lib;pcl_surface_release.lib;pcl_tracking_release.lib;qhull.lib;

7.
		xcopy /e /i /y "$(PROJECT_DIR)..\..\..\addons\ofxPCL\libs\pcl\bin\vs\*.dll" "$(PROJECT_DIR)bin"
		xcopy /e /i /y "$(ProjectDir)..\..\..\export\vs\*.dll" "$(ProjectDir)bin"


## Setup instruction (OS X)

1. Get dependencies (96.9 MB) and extract them to ofxPCL folder.

		$ curl -O http://structor.jp/dist/ofxpcl_16_libs.zip
		$ unzip ofxpcl_16_libs.zip


1. Change `Project.xcconfig` like

		OFXPCL_PATH = $(OF_PATH)/addons/ofxPCL

		OFXPCL_OTHER_LDFLAGS = -L$(OFXPCL_PATH)/libs/pcl/lib/osx -lpcl_common -lpcl_features -lpcl_filters -lpcl_geometry -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_tracking -lqhull

		OFXPCL_HEADER_SEARCH_PATHS = $(OFXPCL_PATH)/libs/pcl/include/ $(OFXPCL_PATH)/libs/pcl/include/eigen3 $(OFXPCL_PATH)/libs/pcl/include/pcl-1.6

		OFXPCL_LD_RUNPATH_SEARCH_PATHS = @executable_path/../../../../../../../addons/ofxPCL/libs/pcl/lib/osx @executable_path/../../../data/pcl/lib

		LD_RUNPATH_SEARCH_PATHS = $(OFXPCL_LD_RUNPATH_SEARCH_PATHS)

		OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OFXPCL_OTHER_LDFLAGS)
		HEADER_SEARCH_PATHS = $(OF_CORE_HEADERS) $(OFXPCL_HEADER_SEARCH_PATHS)

1. Add `$(OFXPCL_HEADER_SEARCH_PATHS)` to Build Settings -> Search Paths -> Header Search Paths **(add above the OpenCV paths when applicable)**


1. Add `$(OFXPCL_OTHER_LDFLAGS)` to Build Settings -> Linking -> Other Linker Flags


1. Add `ofxPCL/src` folder to Xcode project.


## Setup instruction (Windows Code::Blocks)

1. Get dependencies (<http://cim.mcgill.ca/~nhieda/dist/ofxpcl_1.7.1_libs_win_cb.zip> 212MB) and extract them to ofxPCL folder.

1. Project -> Add files to add `ofxPCL\src\*` to the project.

1. In Project -> Build options -> Search directories, select the project name on the top left, add following paths (**add above the OpenCV paths when applicable**)

		..\..\..\addons\ofxPCL\libs\pcl\include
		..\..\..\addons\ofxPCL\libs\pcl\include\eigen3
		..\..\..\addons\ofxPCL\libs\pcl\include\pcl-1.7
		..\..\..\addons\ofxPCL\src

1. In Linker settings, add Link libraries:

		pcl_common;pcl_features;pcl_filters;pcl_io;pcl_io_ply;pcl_kdtree;pcl_keypoints;pcl_octree;pcl_recognition;pcl_registration;pcl_sample_consensus;pcl_search;pcl_segmentation;pcl_surface;pcl_tracking;qhull

1. and Other linker options:

		-L..\..\..\addons\ofxPCL\libs\pcl\lib\win_cb

1. In Pre/post build steps, append:

		xcopy /e /i /y "$(PROJECT_DIR)..\..\..\addons\ofxPCL\libs\pcl\bin\win_cb\*.dll"  "$(PROJECT_DIR)bin"


## Setup instruction (Ubuntu)

1. Get dependencies by apt-get

		sudo apt-get install libboost-all-dev
		sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
		sudo apt-get update
		sudo apt-get install libpcl-all

1. Add ofxPCL to the project **using projectGenerator**.

1. Add the following lines to Makefile before "call the project makefile":

		PROJECT_ADDONS_INCLUDES += /usr/include/pcl-1.7/
		PROJECT_ADDONS_INCLUDES += /usr/include/eigen3/
		PROJECT_ADDONS_LDFLAGS += -lboost_date_time
		PROJECT_ADDONS_LDFLAGS += -lboost_filesystem
		PROJECT_ADDONS_LDFLAGS += -lboost_iostreams
		PROJECT_ADDONS_LDFLAGS += -lboost_system
		PROJECT_ADDONS_LDFLAGS += -lboost_thread
		PROJECT_ADDONS_LDFLAGS += -lpcl_common
		PROJECT_ADDONS_LDFLAGS += -lpcl_features
		PROJECT_ADDONS_LDFLAGS += -lpcl_filters
		PROJECT_ADDONS_LDFLAGS += -lpcl_io
		PROJECT_ADDONS_LDFLAGS += -lpcl_io_ply
		PROJECT_ADDONS_LDFLAGS += -lpcl_kdtree
		PROJECT_ADDONS_LDFLAGS += -lpcl_keypoints
		PROJECT_ADDONS_LDFLAGS += -lpcl_octree
		PROJECT_ADDONS_LDFLAGS += -lpcl_recognition
		PROJECT_ADDONS_LDFLAGS += -lpcl_registration
		PROJECT_ADDONS_LDFLAGS += -lpcl_sample_consensus
		PROJECT_ADDONS_LDFLAGS += -lpcl_search
		PROJECT_ADDONS_LDFLAGS += -lpcl_segmentation
		PROJECT_ADDONS_LDFLAGS += -lpcl_surface
		PROJECT_ADDONS_LDFLAGS += -lpcl_tracking
