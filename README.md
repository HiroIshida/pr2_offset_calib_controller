# pr2 offset calibration controller
Hacky update the offset value of specific joint with specific offset without actual calibration run.

## Usage
Inside PR2 machine:
```
catkin bt  # build the package
```
PR2 machine or locally:
```
python3 calibrate.py --joint l_elbow_flex_joint --offset 20
```
**Note**: The `--offset` value's unit is radian but not of the encoder value, rather (probably) **motor axis** rotation, so you probably need to set it large value than you imagined.
