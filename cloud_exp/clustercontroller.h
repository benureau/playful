#ifndef __NEIGHBORCONTROLLER_H
#define __NEIGHBORCONTROLLER_H

#include "cloud.h"
#include <selforg/sox.h>

/**
 * This controller implements clustering routines on top of the Sox
 * controller.
 */
class ClusterController : public Sox {

public:
  /// constructor
  ClusterController(const SoxConf& conf = getDefaultConf());
  ClusterController(double init_feedback_strength, bool useExtendedModel=true, bool useTeaching=false);

  virtual ~ClusterController();

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
  void step(const sensor* x_, int number_sensors,
                      motor* y_, int number_motors);
private:
  Cloud cloud;

  // Simulation parameters
  int control;
  int center;
  int clusteringrate;
  
};

#endif

