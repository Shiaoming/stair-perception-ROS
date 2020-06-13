

# Introduction

This ROS package is based on [the previous stair-perception kit](https://github.com/Shiaoming/stair-perception) with the following upgrades:

- ROS package
- using a faster organized point cloud segmentation method: [**PEAC** (Plane Extraction using Agglomerative Clustering)](https://github.com/symao/PEAC)
- and stair modeling optimization

# Hardware

- Kinect V2
- IMU

# Software
- Ubuntu16.04
- ROS kinetic
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)
- opencv(ROS included)
- pcl(ROS included)

# Build

```shell
cd ~/catkin_ws/src
git clone https://github.com/Shiaoming/stair-perception-ROS
cd ~/catkin_ws
catkin_make --pkg plane_msg stair_info_msg -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Release
```

For more details about this algorithm, please refer:

```
@article{zhao_adaptive_2019,
	title = {An adaptive stair-ascending gait generation approach based on depth camera for lower limb exoskeleton},
	volume = {90},
	issn = {0034-6748},
	url = {https://doi.org/10.1063/1.5109741 http://aip.scitation.org/doi/10.1063/1.5109741},
	doi = {10/ggsxh2},
	number = {12},
	journal = {Review of Scientific Instruments},
	author = {Zhao, Xiaoming and Chen, Wei-Hai and Li, Bing and Wu, Xingming and Wang, Jianhua},
	month = dec,
	year = {2019},
	note = {Publisher: AIP Publishing, LLC},
	pages = {125112}
}

@inproceedings{zhao2018real,
	title={Real-Time Stairs Geometric Parameters Estimation for Lower Limb Rehabilitation Exoskeleton},
	author={Zhao, Xiaoming and Chen, Weihai and Yan, Xing and Wang, Jianhua and Wu, Xingming},
	booktitle={2018 Chinese Control And Decision Conference (CCDC)},
	pages={5018--5023},
	year={2018},
	organization={IEEE}
}
```

