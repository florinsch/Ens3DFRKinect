CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
PROJECT(face_tracker)

# The version number.
SET(face_tracker_VERSION_MAJOR 0)
SET(face_tracker_VERSION_MINOR 1)

SET(CMAKE_CXX_COMPILER /usr/local/bin/g++-4.7)

# pcl
FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

#FIND_PACKAGE(Boost 1.45.0 COMPONENTS filesystem regex)
#IF(Boost_FOUND)
#    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#ENDIF()

#FIND_PACKAGE(CUDA)
#INCLUDE(FindCUDA)
#CUDA_ADD_EXECUTABLE( face_app openni_face_detection.cpp)

#ADD_EXECUTABLE( face_app openni_face_detection.cpp )
#TARGET_LINK_LIBRARIES( face_app ${PCL_LIBRARIES})

#ADD_EXECUTABLE( folder_viewer folder_viewer.cpp )
#TARGET_LINK_LIBRARIES( folder_viewer ${PCL_LIBRARIES})

#ADD_EXECUTABLE( make_voxel_patches make_voxel_patches.cpp )
#TARGET_LINK_LIBRARIES( make_voxel_patches ${PCL_LIBRARIES} )

ADD_EXECUTABLE( compute_features compute_features.cpp )
TARGET_LINK_LIBRARIES( compute_features ${PCL_LIBRARIES} )

# cuda

#FIND_PACKAGE(CUDA)
#INCLUDE(FindCUDA)
#cuda_add_executable ( ppl_app ../people_src/people_app.cpp)
#cuda_add_executable ( ppl_app ../people_src/people_tracking.cpp)
#target_link_libraries( ppl_app ${PCL_LIBRARIES})
