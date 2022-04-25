## Badges
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Authors
- [Sameep Pote](https://github.com/Sameep2808) - Graduate student at University of Maryland pursuing M.Eng. Robotics.Loves to watch animes.

URL: https://github.com/Sameep2808/A_star_ROS.git

## License
```
MIT License

Copyright (c) 2022 Sameep Pote

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Steps to run the package
1. Download the package and place it in your 'catkin_ws/src' folder.
2. Open terminal and type,
```
cd ~/catkin_ws
catkin_make clean && catkin_make
source devel/setup.bash
```
3. After the catkin workspace is build and set, we will run the code. The code takes five input arguments which are the starting 2D coordinates, starting orientation and the goal nodes. These values are already defaulted to the extreme coordinates so the launch file can run without this input.
4. The coordinates input is given from -5 to 5 range and the angle takes a range of 0 to 360. Below are the arguments:
- x_pos: Initial x-coordinate
- y_pos: Initial y-coordinate
- theta: Initial orientation
- gx: Goal's x-coordinate
- gy: Goal's y-coordinate
5. To run the code, type in the terminal,
```
roslaunch A_star_ROS pro.launch # x_pos:=-3.5 y_pos:=-3.5 gx:=4.5
```
You can uncomment and edit the input arguments as shown above.
