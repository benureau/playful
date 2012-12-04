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

#include "precession.h"

#include <ode_robots/playground.h>

#include <selforg/sinecontroller.h>
#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>

#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

#include "ctrloutline.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;


// starting function (executed once at the beginning of the simulation loop)
void PrecessionSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			  GlobalData& global) {
  int num_barrels=1;
  
  setCameraHomePos(Pos(-8.47437, 8.17268, 4.75058),  Pos(-134.083, -19.4807, 0));
  // initialization
  global.odeConfig.setParam("noise",0.01);
  global.odeConfig.setParam("controlinterval",1);

  double widthground=10;
  double heightground=1;
  Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, .208, heightground)); 
  playground->setColor(Color(1.,1.,1.,1)); 
  playground->setPosition(osg::Vec3(0,0,0));
  Substance substance;
  //  substance.toRubber(.5);
  //      substance.toMetal(1);
  substance.toPlastic(10);
  playground->setGroundSubstance(substance);
  global.obstacles.push_back(playground);
    
  /* * * * BARRELS * * * */
  for(int i=0; i< num_barrels; i++){
    //****************
    Sphererobot3MassesConf conf = Barrel2Masses::getDefaultConf();  
    conf.pendularrange  = 0.3;//0.15; 
    conf.motorpowerfactor  = 200;//150; 
    conf.motorsensor=false;
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
    conf.irAxis1=false;
    conf.irAxis2=false;
    conf.irAxis3=false;
    conf.spheremass   = 1;
    robot = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
				  conf, "Barrel1", 0.4); 

    robot->place (// osg::Matrix::rotate(M_PI/2, 1,0,0) /* rotate to laying postion */ 
		  // * osg::Matrix::rotate(M_PI/4, 0,1,0) /* both axis at 45Deg*/ 
		  osg::Matrix::translate(0,0,.15+2*i));


    controller = new CtrlOutline();        
    controller->setParam("epsC", 0.2);       
    controller->setParam("epsA", 0.2);       
    controller->setParam("creativity", 0.9);       

    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise());
    
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , robot , wiring );
    
    // controller is now initialized and we can modify the matrices
    Matrix C(2,2);
    // try different parameters. 
    // -1.8, -1.8, 2, -1.8  causes the barrel to flip over quickly
    // C.val(0,0)= -1.8; // 5;//-2;//1.8;
    // C.val(0,1)= -1.8; //2; //1.8;
    // C.val(1,0)=  2;
    // C.val(1,1)= -1.8; //-5; // -2;//1.8;
    C.val(0,0)= -.1; 
    C.val(0,1)= -0.05;//random_minusone_to_one(0)*.1; 
    C.val(1,0)= 0.05;//random_minusone_to_one(0)*.1;
    C.val(1,1)= -.1; 
    if(dynamic_cast<CtrlOutline*>(controller)){
      dynamic_cast<CtrlOutline*>(controller)->setC(C);
    }

    //  agent->setTrackOptions(TrackRobot(true, false, false, "", 50));
    global.agents.push_back ( agent );
    global.configs.push_back ( agent );
  }
  
}

void PrecessionSim::addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
}
  
// add own key handling stuff here, just insert some case values
bool PrecessionSim::command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, 
			    int key, bool down){
  if (down) { // only when key is pressed, not when released
    switch ( (char) key )
      {
      case 'y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
      case 'Y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;        
      default:
        break;
      }
  }
  return false;
}
  

