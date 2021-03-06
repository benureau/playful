/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <cmath>
#include <assert.h>
#include "groupController.h"
#include <selforg/controller_misc.h>
#include "ESN.h"

using namespace std;
using namespace matrix;


GroupController::GroupController(AbstractController* controller, int nbContextSensors)
  : AbstractController("GroupController", "$Id$"),
    controller(controller), nbContextSensors(nbContextSensors)
{
  addConfigurable(controller);
  addParameterDef("esnCtrl", &esnCtrl,0,0,1,"0: Normal control, 1: ESN control");
  
  addParameterDef("contextCtrl", &contextCtrl,0,0,1,"0: no control, 1: context control");
  addParameterDef("blueAxis", &blueAxis,0,0,1,"Blue axis value");
  addParameterDef("redAxis", &redAxis,0,0,1,"Red axis value");
  addParameterDef("greenAxis", &greenAxis,0,0,1,"Green axis value");
};

/** initialisation of the controller with the given sensor/ motornumber
    Must be called before use.
*/
void GroupController::init(int sensornumber, int motornumber, RandGen* randGen){
  // the controller does not get the context sensors
  controller->init(sensornumber-nbContextSensors, motornumber, randGen);
  esn = new ESN(30);
  esn->init(sensornumber, motornumber);
  addInspectable(esn);
  addConfigurable(esn);
};

void GroupController::step(const sensor* sensors, int sensornumber,
			  motor* motors, int motornumber) {
double lr;
  Matrix s(sensornumber,1,sensors);
  if(contextCtrl)
  {
    if(((s.val(1,0) > 0.55)) and ((s.val(0,0) > -0.55)) )
    {
     lr=1;
    }

    else
    {
      lr= 0;
    }
  }

  if(esnCtrl){ // let ESN control robot
    const Matrix& m = esn->process(s);
    m.convertToBuffer(motors, motornumber);
  }else{
    controller->step(sensors, sensornumber-nbContextSensors, motors, motornumber);
    //ESN controller from here
    Matrix m(motornumber,1,motors);
    if(((s.val(1,0) > 0.55)) and ((s.val(0,0) > 0.55)) )
    {
     lr=1;
    }

    else
    {
      lr= 0;
    }
    esn->learn(s, m, lr);
  }
};

void GroupController::stepNoLearning(const sensor* sensors, int number_sensors,
				    motor* motors, int number_motors) {

  controller->stepNoLearning(sensors, number_sensors-nbContextSensors,
			     motors, number_motors);
};



