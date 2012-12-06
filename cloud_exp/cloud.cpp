#include <iostream>
#include <selforg/stl_adds.h>

#include "cloud.h"
using namespace matrix;
using namespace cv;
using namespace std;


Cloud::Cloud()
{
}

Cloud::~Cloud()
{
}

void Cloud::configure(int number_motors_, int number_sensors_, int cluster_count_) {
  number_motors = number_motors_;
  number_sensors = number_sensors_;
  cluster_count = cluster_count_;  
}

void Cloud::addControllerState(Matrix A, Matrix C, Matrix h) {
  
  MatrixSet* ms = new MatrixSet(A, C, h);
  statesets.push_back(*ms);

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

    int row_length = statesets[0].C.getM()*statesets[0].C.getN() + statesets[0].h.getM();

    // TODO: reuse features.data pointer
    cout << state_count << endl;
    cout << row_length << endl;
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

    cout << centers << endl;
      
  }
  
}
