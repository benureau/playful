#include <iostream>
#include <algorithm>
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

void Cloud::configure(int number_motors_, int number_sensors_) {

  number_motors = number_motors_;
  number_sensors = number_sensors_;

  framecount = 0;

  row_length = number_motors*number_sensors + number_motors;
  m.set(cluster_count, row_length);

  points.create(0, row_length, CV_32F);

}

void Cloud::addControllerState(Matrix A, Matrix C, Matrix h) {

  framecount += 1;
  
  if (framecount % capture_rate == 0) {
    MatrixSet ms = MatrixSet(A, C, h);
    statesets.push_back(ms);

    Mat features;
    matrixSet2featureVector(ms, features);
    points.push_back(features);
    points.reshape(statesets.size(), row_length);
    
  }
  if (framecount % clustering_rate == 0) {
    if (statesets.size() >= 2) {
      clusterize();
    }
    
  }
  
}

bool partialsort(vector<double> a,vector<double> b){
  return (a[1]<b[1]);
}

void Cloud::clusterize() {

  kmeans(points, cluster_count, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.01), 3, KMEANS_PP_CENTERS, centers);

  vector<vector<double> > centers_vector;
  for (int i = 0; i < cluster_count; i++) {
    vector<double> new_center;
    for (unsigned int j = 0; j < row_length; j++) {
      new_center.push_back(centers.at<float>(i, j));
    }
    centers_vector.push_back(new_center);
  }
  sort(centers_vector.begin(), centers_vector.end(), partialsort);
  
  if ((int)m.getM() != cluster_count) {
    m.set(cluster_count, row_length);
  }
  for (int i = 0; i < cluster_count; i++) {
    for (unsigned int j = 0; j < row_length; j++) {
      m.val(i, j) = centers_vector[i][j];
    }
  }
  
}

/* 
 * Convert a MatrixSet into an vector of features.
 * Currently, the C and h matrix elements only are used, flattened as a list.
 * @param features will be recreated.
 */
void Cloud::matrixSet2featureVector(const MatrixSet& ms, Mat& features) const {

  features.create(1, row_length, CV_32F);

  int k = 0;
  for (unsigned int i = 0; i < ms.C.getM(); i++) {
    for (unsigned int j = 0; j < ms.C.getN(); j++) {
      features.at<float>(0, k) = ms.C.val(i, j);
      k += 1;
    }
  }

  for (unsigned int i = 0; i < ms.h.getM(); i++) {
    features.at<float>(0, k) = ms.h.val(i, 0);
    k += 1;
  }
}

/*
 * Convert a vector of features into a MatrixSet.
 * Since the feature vector only contains information about C and h, A is the null matrix.
 */
void Cloud::featureVector2matrixSet(const Mat& features, MatrixSet& ms) const {

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

  ms.A = A;
  ms.C = C;
  ms.h = h;
}

void Cloud::matrixSetFromCenterIndex(int center_index, MatrixSet& ms) const {
  assert(center_index < centers.rows);
  return featureVector2matrixSet(centers.row(center_index), ms);
}


