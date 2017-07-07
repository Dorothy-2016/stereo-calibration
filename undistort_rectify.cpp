#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
  char* leftimg_filename;
  char* rightimg_filename;
  char* calib_file;
  char* leftout_filename;
  char* rightout_filename;

  static struct poptOption options[] = {
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left imgage path","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image path","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
    { "leftout_filename",'L',POPT_ARG_STRING,&leftout_filename,0,"Left undistorted imgage path","STR" },
    { "rightout_filename",'R',POPT_ARG_STRING,&rightout_filename,0,"Right undistorted image path","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  Mat R1, R2, P1, P2, Q;
  Mat K1, K2, R;
  Vec3d T;
  Mat D1, D2;
  cout<<"111111"<<endl;
  Mat img1 = imread(leftimg_filename, CV_LOAD_IMAGE_COLOR);
  Mat img2 = imread(rightimg_filename, CV_LOAD_IMAGE_COLOR);
    cout<<"22222"<<endl;
  cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
  cout<<"222233333"<<endl;
  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  cout<<"a"<<endl;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  cout<<"a"<<endl;
  fs1["R"] >> R;
  cout<<"a"<<endl;
  //fs1["T"] >> T;

  cout<<"33333"<<endl;

  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;

  cout<<"444444"<<endl;

  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat imgU1, imgU2;
  cout<<"before"<<endl;
//  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
//  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);
  
  imwrite(leftout_filename, imgU1);
  imwrite(rightout_filename, imgU2);

  return 0;
}
