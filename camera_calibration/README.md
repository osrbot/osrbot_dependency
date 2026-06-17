# camera_calibration

This is a local ROS 2 `camera_calibration` variant used for the OSRacer `usb_cam`
RGB camera calibration workflow.

## Local Changes

- Supports `camera:=/rgb` by resolving the remapped camera namespace before calling `/rgb/set_camera_info`.
- Derives the default YAML `camera_name` from the remapped camera namespace when `--camera_name` is not set.
  - `camera:=/rgb` writes `camera_name: rgb`.
- Adds `--calib_yaml_path` for writing the monocular calibration YAML directly to a requested path.
- Keeps the upstream `/tmp/calibrationdata.tar.gz` output and also extracts YAML files when saving.
- Uses ROS 2 async service calls for `set_camera_info`.
- Improves the OpenCV display window size so the right-side buttons are visible without manual resizing.
- Enables line-buffered output for cleaner logs under `ros2 launch`.

## Command-Line Usage

Recommended command:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.03 \
  --calib_yaml_path ~/osracer_ws/src/osracer/osracer_bringup/param/camera_info/rgb.yaml \
  --service-check-timeout 10.0 \
  --ros-args \
  --remap camera:=/rgb \
  --remap image:=/rgb/image_raw
```

The old remap syntax still works, but ROS 2 prints a deprecation warning:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.03 camera:=/rgb image:=/rgb/image_raw
```

## Launch Usage

This package provides an RGB camera calibration launch file:

```bash
ros2 launch camera_calibration rgb_camera_calibration.launch.py
```

It is equivalent to:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.03 \
  --calib_yaml_path ~/osracer_ws/src/osracer/osracer_bringup/param/camera_info/rgb.yaml \
  --service-check-timeout 10.0 \
  --ros-args \
  --remap camera:=/rgb \
  --remap image:=/rgb/image_raw
```

If this launch file is copied into another package, keep `emulate_tty=True` in
the `Node` action. Without it, Python `print()` output may be buffered and only
appear after Ctrl+C.

The launch file waits for `/rgb/set_camera_info` at startup. This avoids a race
where the camera driver is running but its service has not been registered yet.

## Calibration Workflow

1. Check that the camera topic and service exist:

   ```bash
   ros2 topic list | grep rgb
   ros2 service list | grep rgb
   ```

   You should see at least:

   ```text
   /rgb/image_raw
   /rgb/set_camera_info
   ```

2. Start the calibration tool.

3. Move the checkerboard through enough positions to collect samples.

4. Click `CALIBRATE` to compute the calibration.

5. Click `SAVE` to write calibration files.

   Default outputs:

   ```text
   /tmp/calibrationdata.tar.gz
   /tmp/ost.yaml
   ```

   With `--calib_yaml_path`, the YAML is also written to the requested path:

   ```text
   ~/osracer_ws/src/osracer/osracer_bringup/param/camera_info/rgb.yaml
   ```

6. Click `COMMIT` to call `/rgb/set_camera_info` and send the calibration to the camera driver.

## SAVE vs COMMIT

- `SAVE` writes calibration files to disk.
- `COMMIT` calls the camera node's `set_camera_info` service.

If you only need a YAML file for `camera_info_url`, clicking `SAVE` is enough.
Whether `COMMIT` persists the calibration depends on the camera driver and its
camera info manager configuration.

## Common Issues

### YAML File Not Found

Upstream `camera_calibration` writes `/tmp/calibrationdata.tar.gz`; the YAML is
inside the tarball. This local variant extracts YAML files automatically.

To inspect the tarball manually:

```bash
tar -tzf /tmp/calibrationdata.tar.gz
```

For monocular calibration, the YAML file is usually:

```text
ost.yaml
```

### COMMIT Cannot Find the Service

Check the available services:

```bash
ros2 service list | grep set_camera_info
```

When using `camera:=/rgb`, this service must exist:

```text
/rgb/set_camera_info
```

If the camera driver and calibrator start at nearly the same time, the service
may take a few seconds to appear. Increase the wait if needed:

```bash
--service-check-timeout 15.0
```

If you only want to collect samples and write a YAML file, `--no-service-check`
can still be used, but `COMMIT` requires the matching `/rgb/set_camera_info`
service before clicking `COMMIT`.

### Logs Appear Only After Ctrl+C Under ros2 launch

Use this in the launch `Node` action:

```python
emulate_tty=True
```

The included `rgb_camera_calibration.launch.py` already does this.

## Build

```bash
colcon build --symlink-install --allow-overriding camera_calibration
source install/setup.bash
```
