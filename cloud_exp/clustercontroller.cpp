/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

#include "clustercontroller.h"
using namespace matrix;
using namespace std;

ClusterController::ClusterController(const SoxConf& conf)
  : Sox(conf)
{

}

ClusterController::ClusterController(double init_feedback_strength, bool useExtendedModel, bool useTeaching )
  : Sox(init_feedback_strength, useExtendedModel, useTeaching)
{
   addParameterDef("control", &control, 0, 0, 1,
                   "enable cluster control.");
   addParameterDef("center", &center, 0, 0, 20, // TODO look if bounds can be dynamic
                   "center number for control.");
   addParameterDef("clusteringrate", &(cloud.clustering_rate), 1000, 0, 1000000,
                   "number of steps between two clustering routines.");
   addParameterDef("clustercount", &(cloud.cluster_count), 10, 0, 100,
                   "number of clusters to compute.");
   addParameterDef("capturerate", &(cloud.capture_rate), 10, 0, 100,
                   "number of steps between two controller capture.");

   addInspectable(&cloud);
}

void ClusterController::init(int sensornumber, int motornumber, RandGen* randGen) {
  cloud.configure(motornumber, sensornumber);
  Sox::init(sensornumber, motornumber, randGen);
}

ClusterController::~ClusterController(){
}

void ClusterController::step(const sensor* x_, int number_sensors,
                                    motor* y_, int number_motors){
  
  if (control == 0) {
    cloud.addControllerState(getA(), getC(), geth());
    Sox::step(x_, number_sensors, y_, number_motors);

  } else {
    MatrixSet ms;
    cloud. matrixSetFromCenterIndex(center, ms);
    setC(ms.C);
    seth(ms.h);
    
    Sox::stepNoLearning(x_, number_sensors, y_, number_motors);
  }

  
                                         
}
