#include "cloud.h"
using namespace matrix;
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
  
  statesets.push_back(MatrixSet(A, C, h));
}

void Cloud::clusterise() {
  
  
}
