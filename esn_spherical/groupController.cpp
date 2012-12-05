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


GroupController::GroupController(AbstractController* controller)
  : AbstractController("GroupController", "$Id$"), 
  controller(controller)
{
  addConfigurable(controller);
/*
  addParameterDef("period", &period,50);
  addParameterDef("phaseshift", &phaseShift, 1);
  
  addParameterDef("amplitude", &amplitude, 1);

  number_sensors=0;
  number_motors=0;  
*/

};

/** initialisation of the controller with the given sensor/ motornumber 
    Must be called before use.
*/
void GroupController::init(int sensornumber, int motornumber, RandGen* randGen){
  controller->init(sensornumber, motornumber, randGen);
  esn = new ESN(30);
  esn->init(sensornumber, motornumber);
  addInspectable(esn);
  addConfigurable(esn);
};
  
void GroupController::step(const sensor* sensors, int sensornumber, 
			  motor* motors, int motornumber) {
  controller->step(sensors, sensornumber, motors, motornumber);
  //ESN controller from here	
  Matrix s(sensornumber,1,sensors);
  Matrix m(motornumber,1,motors);
  esn->learn(s, m);
  
};

void GroupController::stepNoLearning(const sensor* sensors, int number_sensors, 
				    motor* motors, int number_motors) {
  
  controller->stepNoLearning(sensors, number_sensors, motors, number_motors);
};
 


