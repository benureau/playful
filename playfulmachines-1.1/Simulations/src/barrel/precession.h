/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
#ifndef __PRECESSION_H
#define __PRECESSION_H

#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/abstractcontroller.h>


class PrecessionSim : public lpzrobots::Simulation {
public:
  AbstractController *controller;
  lpzrobots::OdeRobot* robot;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, 
	     lpzrobots::GlobalData& global);

  virtual void addCallback(lpzrobots::GlobalData& globalData, 
			   bool draw, bool pause, bool control);
  
  // add own key handling stuff here, just insert some case values
  virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, 
		       lpzrobots::GlobalData& globalData, int key, bool down);
  
};

#endif
