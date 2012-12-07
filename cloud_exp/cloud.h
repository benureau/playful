#ifndef __CLOUD_H
#define __CLOUD_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <selforg/inspectable.h>
#include <selforg/matrix.h>

/**
 * A struct for a controller state
 */
class MatrixSet {

public :
  matrix::Matrix A;
  matrix::Matrix C;
  matrix::Matrix h;

  MatrixSet()
  {
  }
  
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
class Cloud : public Inspectable {

public:
  // number_motors and numbers_sensors are the one of the Sos/Sox controller
  // cluster_count is the number of clusters we want to obtain
  Cloud();

  virtual ~Cloud();

  void configure(int number_motors, int number_sensors);
  
  int number_motors;
  int number_sensors;
  int cluster_count;

  int clustering_rate;
  int capture_rate;

  void addControllerState(matrix::Matrix A, matrix::Matrix C, matrix::Matrix h);
  void clusterize();

  void matrixSetFromCenterIndex(int center_index, MatrixSet& ms) const;
  
private:
  // Internal data store
  unsigned int row_length;
  std::vector<MatrixSet> statesets;

  // Misc
  int framecount;
  matrix::Matrix m;

  // OpenCV clustering data structures
  cv::Mat points;
  cv::Mat centers;
  cv::Mat labels;

  // Conversion functions
  void matrixSet2featureVector(const MatrixSet& ms, cv::Mat& features) const;
  void featureVector2matrixSet(const cv::Mat& features, MatrixSet& ms) const;
  
  
};

#endif