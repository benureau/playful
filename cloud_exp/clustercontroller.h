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
  ClusterController(Cloud cloud, const SoxConf& conf = getDefaultConf());
  ClusterController(Cloud cloud, double init_feedback_strength, bool useExtendedModel=true, bool useTeaching=false);

  virtual ~ClusterController();

  void stepNoLearning(const sensor* x_, int number_sensors,
                      motor* y_, int number_motors);
private:
  Cloud cloud;
};

#endif

