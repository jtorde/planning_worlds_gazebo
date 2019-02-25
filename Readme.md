
# Worlds for Gazebo #

Gazebo worlds used in the paper **Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments** (ICRA 2019) ([pdf](https://arxiv.org/abs/1810.01035), [video](https://www.youtube.com/watch?v=E4V2_B8x-UI))

```
@article{tordesillas2018real,
  title={Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and Carter, John and Ware, John and How, Jonathan P},
  journal={arXiv preprint arXiv:1810.01035},
  year={2018}
}
```

The three worlds available in this repo are these ones:


Office (.world + .bag)     |  Bugtrap (.world + .bag)  |  Forest (x10) (.world )
:-------------------------:|:-------------------------:|:-------------------------:
![](./imgs/office.png)     |  ![](./imgs/bugtrap.png)  |  ![](./imgs/forest.png) 

### Instructions:
Clone this repository:
```
git clone https://github.com/jtorde/planning_worlds_gazebo.git
```

If you want to generate your own forest:
```
cd planning_worlds_gazebo
catkin build
source devel/setup.bash 
```
Then, in one terminal:
```
roslaunch gazebo_ros empty_world.launch
```
And in other one:
```
rosrun worlds_project create_random_forest
```

The cylinders will start to polulate the world with the density specified in `create_random_forest.cpp`.


### License and Credits
Academic license - for non-commercial use only

Some parts of the code of `create_random_forest.cpp` has been take from [here](https://github.com/ethz-asl/mav_voxblox_planning/blob/master/mav_planning_benchmark/src/local_planning_benchmark.cpp).

The file `gazebo_cube_spawner.h` is from [here](https://github.com/JenniferBuehler/gazebo-pkgs/blob/master/gazebo_test_tools/include/gazebo_test_tools/gazebo_cube_spawner.h)

The office world is the `willow_garage.world` provided by Gazebo