## OpenCV C++ Stereo Camera Calibration

A small tool for setreo camera calibration for both pinhole modle and fisheye model.
this repo is modified form [this repo](https://github.com/sourishg/stereo-calibration) and [this repo](https://github.com/sourishg/fisheye-stereo-calibration) and merge them together.

### Dependencies

- OpenCV
- popt
  - if not installed, do `apt-get install libpopt-dev`

### Compilation

Compile all the files using the following commands.

```bash
mkdir build && cd build
cmake ..
make
```

Make sure your are in the `build` folder to run the executables.


### Intrinsic and extrinisics calibration of stereo camera

This can do lenses with pinhole model and fisheye model in one step. The calibration saves the camera matrix and the distortion coefficients in a YAML file. The datatype for these matrices is `Mat`.

Once you have compiled the sources run the following command to calibrate the intrinsics.

```bash
./calibrate_stereo [-P / -F](pinhole or fisheye)
-w [board_width_corner_number] -h [board_height_corner_number] -n [num_imgs]
-s [square_size(m)] -L [left_img_dir] -R [right_img_dir]
-l [left_img_prefix] -r [right_img_prefix] -e [extension]
(-S [scaling. Ingnore this then automatic scale]) -o [output_calib_file]
```

For example, the command for the test fisheye images in `calib_imgs/left/` and `calib_imgs/right/` with file name `left1.jpg ...` would be

```bash
./calibrate_stereo -F -w 8 -h 6 -n 27 -s 0.082 -L ../calib_img/left/ -R ../calib_imgs/right/ -l left -r right -o cam_stereo.yml
```

### Undistortion and Rectification

Once you have the stereo calibration data, you can remove the distortion and rectify any pair of images so that the resultant epipolar lines become scan lines.

```bash
./undistort_rectify [-P / -F](pinhole or fisheye) -l [left_img_path]
-r [right_img_path] -c [stereo_calib_file] -L [output_left_img] -R [output_right_img]
```

For example

```bash
./undistort_rectify -P -l ../calib_imgs/1/left1.jpg -r ../calib_imgs/1/right1.jpg -c cam_stereo.yml -L left.jpg -R right.jpg
```
