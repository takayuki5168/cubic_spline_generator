Cubic Spline Generator
======================
Generator of Cubic Spline in two-dimensional plane

## How to Use
You can execute `example/main.cpp` as follwing.
```bash
mkdir build
cd build
cmake ..
make
./example
```

Also you can execute below and plot spline with a following script.
```bash
./scripts/debug.sh
```

## Cubic Spline
Necessary parameters are belows
- points of spline curve
- angle of start
- angle of goal
- velocity of start
- velocity of goal

Constructor of Spline class is shown as bellow.
```cpp
Spline(std::vector<std::array<double, 2>> via_pos_vec, double start_angle, double goal_angle, double start_vel=5, double goal_vel=5)
```
