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

ClusterController::ClusterController(Cloud cloud_, const SoxConf& conf)
  : Sox(conf)
{
  cloud = cloud_;
}

ClusterController::ClusterController(Cloud cloud_, double init_feedback_strength, bool useExtendedModel, bool useTeaching )
  : Sox(init_feedback_strength, useExtendedModel, useTeaching)
{
  cloud = cloud_;
}

void ClusterController::init(int sensornumber, int motornumber, RandGen* randGen) {
  cloud.configure(motornumber, sensornumber, 20);
  Sox::init(sensornumber, motornumber, randGen);
}

ClusterController::~ClusterController(){
}

void ClusterController::stepNoLearning(const sensor* x_, int number_sensors,
                                       motor* y_, int number_motors){
  Matrix A = this->getA();
  Matrix C = this->getC();
  Matrix h = this->geth();
  
  cloud.addControllerState(A, C, h);
  cloud.clusterize();
  Sox::stepNoLearning(x_, number_sensors, y_, number_motors);
  
                                         
}
