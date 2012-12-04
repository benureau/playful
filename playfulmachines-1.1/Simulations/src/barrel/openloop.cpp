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

#include "openloop.h"

#include <ode_robots/playground.h>

#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/motornoisewiring.h>

#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;

// starting function (executed once at the beginning of the simulation loop)
void OpenLoopSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			  GlobalData& global) {
  int num_barrels=1;
  
  setCameraHomePos(Pos(-1.29913, 6.06201, 1.36947),  Pos(-165.217, -9.32904, 0));
  setCameraMode(Follow);
  // initialization
  global.odeConfig.setParam("noise",0);
  // the simulation runs with 100Hz and we control every second
  global.odeConfig.setParam("controlinterval",2); 

  // add a new parameter to be configured on the console
  global.odeConfig.addParameterDef("friction", &friction, 0.0, "rolling friction coefficient");
  global.odeConfig.addParameterDef("color", &color, 0, "color of noise (correlation length)");
  
  /* * * * BARRELS * * * */
  for(int i=0; i< num_barrels; i++){
    //****************
    Sphererobot3MassesConf conf = Barrel2Masses::getDefaultConf();  
    conf.pendularrange  = 0.3;//0.15; 
    conf.motorpowerfactor  = 200;//150; 
    conf.motorsensor=false;
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
    conf.spheremass   = 1;
    robot = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
				  conf, "Barrel", 0.4); 

    robot->place (osg::Matrix::rotate(M_PI/2, 1,0,0) /* rotate to laying position */ 
		  * osg::Matrix::rotate(M_PI/4, 0,1,0) /* both axis at 45Deg*/ 
		  * osg::Matrix::translate(0,0,.15+2*i)); // place at 0,0 and in some height
    
    controller = new SineController();  
    controller->setParam("period", 300);  
    controller->setParam("phaseshift", 1);
    
    noisegen = new ColorUniformNoise();
    wiring = new MotorNoiseWiring(noisegen, 0);
    
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , robot , wiring );
    
    global.agents.push_back ( agent );
    global.configs.push_back ( agent );
  }
 
}

void OpenLoopSim::addCallback(GlobalData& global, bool draw, bool pause, bool control) {
  if(!pause){
    if(friction>0){
      Pos vel = robot->getMainPrimitive()->getAngularVel();
      robot->getMainPrimitive()->applyTorque(-vel*friction*5);
    }
    // apply the color noise parameter
    if(color>=1)
      noisegen->setTau(1/color);
    else
      noisegen->setTau(1);    
  }
}

void OpenLoopSim::bindingDescription(osg::ApplicationUsage & au) const {
  au.addKeyboardMouseBinding("Barrel: y","add force to the left");
  au.addKeyboardMouseBinding("Barrel: Y","add force to the right");
  au.addKeyboardMouseBinding("Barrel: x","add torque counter-clockwise");
  au.addKeyboardMouseBinding("Barrel: X","add torque clockwise");
  au.addKeyboardMouseBinding("Barrel: i","decrease period");
  au.addKeyboardMouseBinding("Barrel: I","incrase period");
};


  
// add own key handling stuff here, just insert some case values
bool OpenLoopSim::command(const OdeHandle&, const OsgHandle&, GlobalData& global, 
                          int key, bool down){
  if (down) { // only when key is pressed, not when released
    switch ( (char) key )
      {
      case 'y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
      case 'Y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
      case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 10 , 0 ); break;
      case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , -10 , 0 ); break;
      case 'I' : controller->setParam("period", controller->getParam("period")*1.2); 
	printf("period : %g\n", controller->getParam("period"));
        return true;
	break;
      case 'i' : controller->setParam("period", controller->getParam("period")/1.2); 
	printf("period : %g\n", controller->getParam("period"));
        return true;
	break;
      default:
	return false;
	break;
      }
  }
  return false;
}
  

