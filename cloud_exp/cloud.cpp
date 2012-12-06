#include <iostream>
#include <selforg/stl_adds.h>

#include "cloud.h"
using namespace matrix;
using namespace cv;
using namespace std;


Cloud::Cloud()
{
  addInspectableMatrix("Ce", &m, false);
}

Cloud::~Cloud()
{
}

void Cloud::configure(int number_motors_, int number_sensors_, int cluster_count_, int frequency_) {
  number_motors = number_motors_;
  number_sensors = number_sensors_;
  cluster_count = cluster_count_;
  frequency = frequency_;
  framecount = 0;

  row_length = number_motors*number_sensors + number_motors;
  m.set(cluster_count, row_length);
}

void Cloud::addControllerState(Matrix A, Matrix C, Matrix h) {

  
  if (framecount >= frequency) {
    MatrixSet* ms = new MatrixSet(A, C, h);
    statesets.push_back(*ms);
    clusterize();
    framecount = 0;
  } else {
    framecount += 1;
  }

//   for (unsigned int i = 0; i < C.getM(); i++) {
//     for (unsigned int j = 0; j < C.getN(); j++) {
//       features.push_back(C.val(i, j));
//     }
//   }
//   for (unsigned int i = 0; i < h.getM(); i++) {
//       features.push_back(h.val(i, 0));
//   }
  
}

void Cloud::clusterize() {
  
  int state_count = statesets.size();

  if (state_count >= cluster_count) {

    assert( row_length == statesets[0].C.getM()*statesets[0].C.getN() + statesets[0].h.getM());


    // TODO: reuse features.data pointer
    Mat points(state_count, row_length, CV_32F);
    for (unsigned int s_i = 0; s_i < statesets.size(); s_i++) {
      list<double> values = statesets[s_i].C.convertToList();
      values += statesets[s_i].h.convertToList();

      int j = 0;
      for (list<double>::iterator it = values.begin(); it != values.end(); it++, j++) {
        points.at<float>(s_i, j) = *it;
      }
    }

    centers.create(cluster_count, 1, points.type());

    kmeans(points, cluster_count, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

    for (int i = 0; i < cluster_count; i++) {
      for (int j = 0; j < row_length; j++) {
        m.val(i, j) = centers.at<float>(i, j);
      }
    }
    //cout << centers << endl;
      
  }
  
}

MatrixSet* Cloud::matrixset4center(int center_index) const {
  return feature2matrixset(centers.row(center_index));
}

MatrixSet* Cloud::feature2matrixset(const Mat& features) const {

  Matrix A(number_sensors, number_motors);
  Matrix C(number_motors , number_sensors);
  Matrix h(number_motors, 1);

  for (int i = 0; i < number_motors; i++) {
    for (int j = 0; j < number_sensors; j++) {
      C.val(i, j) = features.at<float>(0, i*number_sensors + j);
    }
  }
  for (int i = 0; i < number_motors; i++) {
    h.val(i, 0) = features.at<float>(0, number_sensors*number_motors + i);
  }
  
  MatrixSet* ms = new MatrixSet(A, C, h);

  return ms;
}
