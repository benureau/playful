#ifndef __NEIGHBORCONTROLLER_H
#define __NEIGHBORCONTROLLER_H

#include <sox.h>

/**
 * This controller implements clustering routines on top of the Sox
 * controller.
 */
class ClusterController : public Sox {

public:
  /// constructor
  ClusterController(const SoxConf& conf = getDefaultConf());

  /// constructor provided for convenience, use conf object to customize more
  ClusterController(double init_feedback_strength, bool useExtendedModel = true,
      bool useTeaching = false );

  virtual ~ClusterController();
};

#endif

