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
#ifndef __GROUPCONTROLLER_H
#define __GROUPCONTROLLER_H


#include <stdio.h>
#include <selforg/abstractcontroller.h>
#include <selforg/matrix.h>
#include "ESN.h"

/**
 * class for robot control with sine, sawtooth and impuls
 *
 * period is the length of the period in steps and
 * phaseshift is the phase difference between channels given in Pi/2
 */
class GroupController : public AbstractController {
public:

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  GroupController(AbstractController* controller, int nbContextSensors = 0);

  /** initialisation of the controller with the given sensor/ motornumber
      Must be called before use.
  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  /** @return Number of sensors the controller was initialised
      with or 0 if not initialised */
  virtual int getSensorNumber() const {return controller->getSensorNumber();}


  /** @return Number of motors the controller was initialised
      with or 0 if not initialised */
  virtual int getMotorNumber() const {return controller->getMotorNumber();}

  /** performs one step ( the same as StepNoLearning).
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
		    motor* motors, int motornumber);
  /** performs one step.
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
			      motor* , int number_motors);


  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    esn->store(f);
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    esn->restore(f);
    Configurable::parse(f);
    return true;
  }


protected:

  AbstractController* controller;
  ESN* esn;
  int nbContextSensors;

  int esnCtrl;
  int contextCtrl;
  int blueAxis;
  int redAxis;
  int greenAxis;
  double Axis3;
  double Axis4;
  double Axis5;
};

#endif

