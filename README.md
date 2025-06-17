# pr2 offset calibration controller
Hacky update the offset value of specific joint with specific offset without actual calibration run.

## Usage
Inside PR2 machine:
```bash
catkin bt  # build the package
```
then restart the controller manager. For the PR2 in JSK we'll use the following command:
```bash
sudo systemctl stop robot
sudo systemctl start jsk-pr2-startup
```

PR2 machine or locally:
```bash
python3 calibrate.py --joint l_elbow_flex_joint --offset 20
```
**Note**: The `--offset` value's unit is radian but not of the encoder value, rather (probably) **motor axis** rotation, so you need to set it large value than you imagined.
