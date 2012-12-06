#ifndef __CLOUD_H
#define __CLOUD_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <selforg/sox.h>
#include <selforg/matrix.h>


/**
 * A struct for a controller state
 */
class MatrixSet {

public :
  matrix::Matrix A;
  matrix::Matrix C;
  matrix::Matrix h;
  
  MatrixSet(matrix::Matrix A_, matrix::Matrix C_, matrix::Matrix h_)
  {
    A = A_;
    C = C_;
    h = h_;
  }
  
};

/**
 * This class clusterise the matrices of the Sox controller
 */
class Cloud{

public:
  // number_motors and numbers_sensors are the one of the Sos/Sox controller
  // cluster_count is the number of clusters we want to obtain
  Cloud();

  virtual ~Cloud();

  void configure(int number_motors, int number_sensors, int cluster_count);
  
  int number_motors;
  int number_sensors;
  int cluster_count;
  
  void addControllerState(matrix::Matrix A, matrix::Matrix C, matrix::Matrix h);
  void clusterize();

private:
  std::vector<MatrixSet> statesets;
//  std::vector<double> features; // Raw feature array. 

  cv::Mat centers;
  cv::Mat labels;
};

#endif