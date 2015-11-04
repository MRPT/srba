[![Build Status](https://travis-ci.org/MRPT/srba.png?branch=master)](https://travis-ci.org/MRPT/srba)

TL;DR: **Sparser Relative Bundle Adjustment (SRBA)** is a header-only C++ library for solving SLAM/BA in relative coordinates with flexibility for different submapping strategies and aimed at constant time local graph update. BSD 3-Clause License.

Related papers: 
* Moreno, F.A. and Blanco, J.L. and Gonzalez, J. **A constant-time SLAM back-end in the continuum between global mapping and submapping: application to visual stereo SLAM**, International Journal of Robotics Research, 2016. (In Press)
* Blanco, J.L. and Gonzalez, J. and Fernandez-Madrigal, J.A. **Sparser Relative Bundle Adjustment (SRBA): constant-time maintenance and local optimization of arbitrarily large maps**, IEEE International Conference of Robotics and Automation (ICRA), 2013. ([PDF](http://ingmec.ual.es/~jlblanco/papers/blanco2013rba.pdf)), ICRA slides ([PDF](http://ingmec.ual.es/~jlblanco/papers/blanco2013rba_ICRA_slides.pdf)), [BibTeX](http://ingmec.ual.es/aigaion2/index.php/export/publication/233/bibtex)

# 1. Compile

Requisites: 
* MRPT >= 1.3.0  ([web](http://www.mrpt.org/), [github](https://github.com/MRPT/mrpt), [Ubuntu PPA](http://www.mrpt.org/MRPT_in_GNU/Linux_repositories))
* gcc or clang (any version supported by Eigen 3) or MS Visual C++ 2008 or newer.
* CMake >=2.8

In Ubuntu, install requisites with:  
```
sudo apt-get install build-essential cmake libmrpt-dev
```

Clone, configure and build as usual with CMake:

```
git clone https://github.com/MRPT/srba.git 
cd srba
mkdir build && cd build
cmake .. 
make 
make test
```

# 2. Theoretical bases

[Bundle adjustment](http://en.wikipedia.org/wiki/Bundle_adjustment) is the name given to one solution to visual SLAM based on maximum-likelihood estimation (MLE) over the space of map features and camera poses. However, it is by no way limited to visual maps, since the same technique is also applicable to maps of pose constraints (graph-SLAM) or any other kind of feature maps not relying on visual information.

The framework of Relative Bundle Adjustment (RBA) was introduced in a series of works by G. Sibley and colleagues:
* Sibley, G. **Relative bundle adjustment**. Department of Engineering Science, Oxford University, Tech. Rep, 2009. ([PDF](http://www.robots.ox.ac.uk/~gsibley/Personal/Papers/rba.pdf))
* Sibley, G. and Mei, C. and Reid, I. and Newman, P. **Adaptive relative bundle adjustment**. Robotics Science and Systems Conference. 2009. ([PDF](http://homepages.laas.fr/~cmei/uploads/Main/gsibley-RSS2009.pdf))

**Sparser RBA (SRBA)** is the name of the generic and extensible framework for RBA implemented in this C++ library, and introduced in the ICRA 2013 paper ([PDF](http://ingmec.ual.es/~jlblanco/papers/blanco2013rba.pdf), see full citation above).

# 3. Programming guide and documentation

* The official [user guide](http://reference.mrpt.org/devel/srba-guide.pdf)
* Doxygen C++ [API reference](http://mrpt.github.io/srba/)
* `srba-slam` [command-line reference](http://www.mrpt.org/Application%3Asrba-slam)

# 4. Run sample datasets

## 4.1. Monocular visual SLAM with synthetic dataset

https://www.youtube.com/watch?v=ZXti4GxqkUg


## 4.2. Relative 2D graph-SLAM

* Download and compile RWT, a small tool for generating synthetic datasets.
* Create an empty directory and copy there the files:
  * datasets/srba-demos/world-2d-30k-rel-graph-slam.cfg (The configuration file for the synthetic dataset)
  * datasets/srba-demos/world-2d-30k-landmarks.wrl (The geometrical description of the synthetic scenario)
* Generate the dataset, executing:
```rwt-dataset-simulator world-2d-30k-rel-graph-slam-cfg```

Now you can run RBA on the dataset with:
```
srba-slam --se2 --graph-slam -d dataset_30k_rel_graph_slam_SENSOR.txt \\
--submap-size 10 --max-spanning-tree-depth 3 --max-optimize-depth 3 \\
--verbose 1 --noise 0.001 --noise-ang 0.2 --add-noise \\
--gt-map dataset_30k_rel_graph_slam_GT_MAP.txt \\
--gt-path dataset_30k_rel_graph_slam_GT_PATH.txt # --step-by-step
```

