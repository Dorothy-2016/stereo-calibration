#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point3d > > object_points_64;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;
vector< vector< Point2d > > left_img_points_64, right_img_points_64;

Mat img1, img2, gray1, gray2;
Size im_size;

void setup_calibration(int board_width, int board_height, int num_imgs,
                       float square_size,
                       char* leftimg_dir, char* rightimg_dir,
                       char* leftimg_filename, char* rightimg_filename,
                       char* extension) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%d.jpg", leftimg_dir, leftimg_filename, i ,extension);
    sprintf(right_img, "%s%s%d.jpg", rightimg_dir, rightimg_filename, i ,extension);
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found1)
    {
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
    }
    if (found2)
    {
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
    }

    vector< Point3f > obj;
    vector< Point3d > obj_64;
    for (int ii = 0; ii < board_height; ii++)
      for (int jj = 0; jj < board_width; jj++)
      {
        obj.push_back(Point3f((float)jj * square_size, (float)ii * square_size, 0));
        obj_64.push_back(Point3d(double((float)(jj+1) * square_size  ), double((float)(ii+1) *
                                                                       square_size ), 0));
      }
    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
      object_points_64.push_back(obj_64);
    }
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    vector< Point2d > v1_64, v2_64;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));

      v1_64.push_back(Point2d((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2_64.push_back(Point2d((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);

    left_img_points_64.push_back(v1_64);
    right_img_points_64.push_back(v2_64);
  }
}
double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints_out;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints_out);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints_out), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}
double computeReprojectionErrors(const vector< vector< Point3d > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints_out;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());
  for (i = 0; i < (int)objectPoints.size(); ++i) {
      //Problem here
    cv::fisheye::projectPoints(Mat(objectPoints[i]), imagePoints_out, rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints_out), CV_L2);
    int n ;
    n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}
int main(int argc, char const *argv[])
{
  bool pinhole_mode=false;
  bool fisheye_mode=false;
  char* leftimg_dir;
  char* rightimg_dir;
  char* leftimg_filename;
  char* rightimg_filename;
  char* extension;
  char* out_file;
  int num_imgs;
  int board_width, board_height;
  float square_size;
  float alpha=-1;

  static struct poptOption options[] = {
    { "pinehole", 'P', 0, 0, 'P', "use pinehole modle", NULL },
    { "fisheye", 'F', 0, 0, 'F', "use fisheye modle", NULL },
    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
    { "leftimg_dir",'L',POPT_ARG_STRING,&leftimg_dir,0,"Directory containing left images","STR" },
    { "rightimg_dir",'R',POPT_ARG_STRING,&rightimg_dir,0,"Directory containing right images","STR" },
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left image prefix","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image prefix","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "scaling",'S',POPT_ARG_FLOAT,&alpha,0,"Free scaling parameter","NUM" },
  POPT_AUTOHELP
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
      switch (c) {
          case 'P':
            pinhole_mode = true;
            fisheye_mode=false;
            break;
          case 'F':
            pinhole_mode = false;
            fisheye_mode=true;
            break;
       }
  }

  setup_calibration(board_width, board_height, num_imgs, square_size,
                   leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename,extension);

  printf("Starting Calibration\n");
  Mat K1, K2, R, F, E;
  Vec3d T;
  Mat D1, D2;
  vector< Mat > rvecs, tvecs;
  if(pinhole_mode)
  {
      int flag_mono = 0;
      flag_mono |= CV_CALIB_FIX_K4;
      flag_mono |= CV_CALIB_FIX_K5;
      calibrateCamera(object_points, imagePoints1, img1.size(), K1, D1, rvecs, tvecs, flag_mono);
      cout << "Calibration 1 error: " <<
              computeReprojectionErrors(object_points, imagePoints1, rvecs, tvecs, K1, D1) << endl;

      rvecs.clear();
      tvecs.clear();

      calibrateCamera(object_points, imagePoints2, img2.size(), K2, D2, rvecs, tvecs, flag_mono);
      cout << "Calibration 2 error: " <<
              computeReprojectionErrors(object_points, imagePoints2, rvecs, tvecs, K2, D2) << endl;

      int flag_stereo = 0;
      flag_stereo |= CV_CALIB_FIX_INTRINSIC;



      stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F,flag_stereo);
    }
  else if(fisheye_mode)
  {

      int flag_mono = 0;
      flag_mono |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
      flag_mono |= cv::fisheye::CALIB_CHECK_COND;
      flag_mono |= cv::fisheye::CALIB_FIX_SKEW;
      //flag_mono |= cv::fisheye::CALIB_FIX_K4;

      cv::fisheye::calibrate(object_points_64, imagePoints1, img1.size(), K1, D1, rvecs, tvecs, flag_mono);
//      cout << "Calibration 1 error: " <<
//              computeReprojectionErrors(object_points_64, imagePoints1, rvecs, tvecs, K1, D1) << endl;

      rvecs.clear();
      tvecs.clear();
      cv::fisheye::calibrate(object_points_64, imagePoints2, img2.size(), K2, D2, rvecs, tvecs, flag_mono);
//      cout << "Calibration 2 error: " <<
//              computeReprojectionErrors(object_points_64, imagePoints2, rvecs, tvecs, K2, D2) << endl;
      int flag_stereo = 0;
      flag_stereo |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
      flag_stereo |= cv::fisheye::CALIB_CHECK_COND;
      flag_stereo |= cv::fisheye::CALIB_FIX_SKEW;
      //flag |= cv::fisheye::CALIB_FIX_K2;
      //flag |= cv::fisheye::CALIB_FIX_K3;
      //flag |= cv::fisheye::CALIB_FIX_K4;
      cv::fisheye::stereoCalibrate(object_points_64, left_img_points_64, right_img_points_64,
          K1, D1, K2, D2, img1.size(), R, T, flag_stereo);
  }
  else
  {
      cout<<"Please choose camera mode!"<<endl;
      return -1;
  }
  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat R1, R2, P1, P2, Q;
  if (pinhole_mode)
  {
    stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, 0, alpha);
  }
  else if(fisheye_mode)
  {
    cv::fisheye::stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2,
    Q, CV_CALIB_ZERO_DISPARITY, img1.size(), 0.0, 1.0);

  }

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}
