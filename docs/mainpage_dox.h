/** \mainpage SRBA

\section description 1. Description

Sparser Relative Bundle Adjustment (SRBA): a C++ framework for relative SLAM: 

See https://github.com/MRPT/srba and "<i>The SRBA guide</i>" (<a href="srba-guide.pdf" >PDF</a>) for references and further theoretical and coding information.

\section api_ref 2. API reference

The main classes are:

- srba::RbaEngine

\section cpp_ex 3. C++ Examples

All demos can be found under the directory `examples/cpp`. Compile them by enabling the CMake flag `BUILD_EXAMPLES`.

- Range-Bearing 2D SLAM: cpp/tutorial-srba-range-bearing-se2.cpp
- Range-Bearing 3D SLAM: cpp/tutorial-srba-range-bearing-se3.cpp
- Cartesian sensor 2D SLAM: cpp/tutorial-srba-cartesian2d-se2.cpp
- Cartesian sensor 3D SLAM: cpp/tutorial-srba-cartesian2d-se2.cpp
- Relative graph-SLAM 2D: cpp/tutorial-srba-relative-graph-slam-se2.cpp
- Relative graph-SLAM 3D: cpp/tutorial-srba-relative-graph-slam-se3.cpp
- Stereo visual SLAM 2D: cpp/tutorial-srba-stereo-se2.cpp
- Stereo visual SLAM 3D: cpp/tutorial-srba-stereo-se3.cpp
- How to recover the global map: cpp/tutorial-srba-how-to-recover-global-map.cpp

\section ex1 4. Example SLAM code explained

- See full source code in: cpp/tutorial-srba-range-bearing-se2.cpp

First, include SRBA headers and declare the intention to use symbols under namespace `srba`:

\snippet cpp/tutorial-srba-range-bearing-se2.cpp includes_namespaces

Now, configure your SLAM problem by defining all the required template arguments:

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_typedef

Next, declare the SRBA object instance and set up the main algorithm parameters:

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_setup

Once you want to create a new Keyframe, first create the corresponding sensory observations:

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_fill_observation

then call srba::RbaEngine::define_new_keyframe() to create the new KeyFrame and optimize the local area:

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_define_kf0

Repeat the last two steps (create observations, define keyframes) for each new Keyframe. 

At any moment you can create a 3D view of the SLAM state: 

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_3dview

or save the graph in Graphviz DOT format for debugging: 

\snippet cpp/tutorial-srba-range-bearing-se2.cpp srba_save_dot

*/



