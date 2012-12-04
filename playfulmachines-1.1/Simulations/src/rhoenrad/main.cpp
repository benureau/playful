/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *
 ***************************************************************************/
#include <stdio.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// controller
#include <selforg/sox.h>
#include <selforg/soml.h>
#include <selforg/motorbabbler.h>
#include <selforg/sinecontroller.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/forceboostwiring.h>

#include <ode_robots/joint.h>

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>

#include <ode_robots/addsensors2robotadapter.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
// used robot
#include "rhoenrad.h" // if robot is local

#include <ode_robots/operators.h>
#include <ode_robots/tmpprimitive.h>
// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation {
public:
  
  // playground parameter
  const static double widthground = 1000; //1025.85;// 100; //1.3;
  const static double heightground = .8;// 1.2;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");

  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos (Pos(6.81498, 7.18378, 2.74255),  Pos(140.344, -8.23676, 0));
    setCameraMode(Follow);

    int humanoids = 1;

    bool fixedInAir = true;

    global.odeConfig.addParameterDef("forwardforce", &forwardforce, 0.0);

    forcepoint = 0;

    fixator=0;

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("noise",0.0); 
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.01);//0.004);
    global.odeConfig.setParam("gravity", -6);
    global.odeConfig.setParam("controlinterval", 4);


    //************************* SELECT PLAYGROUND HERE ******************?
    setupPlaygrounds(odeHandle, osgHandle, global);
    
    for (int i=0; i< humanoids; i++){ //Several humans      
      // normal servos
      //RhoenradConf conf = Rhoenrad::getDefaultConf();
      // velocity servos
      RhoenradConf conf = Rhoenrad::getDefaultConfVelServos();
      conf.relWheelmass= 4;
      conf.wheelSize=0.98;
      conf.wheelWidth = .8;
      conf.powerFactor = .5;
      conf.onlyPrimaryFunctions = false;

      conf.useOrientationSensor=false;
      conf.dampingFactor=0; 
      //conf.onlyMainParameters=false;
      //  conf.useBackJoint = false;
      conf.jointLimitFactor = 1.5; 

      OdeHandle skelHandle=odeHandle;
      // skelHandle.substance.toMetal(1);
      //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
      // skelHandle.substance.toRubber(5.00);//TEST sonst 40
      Rhoenrad* human = new Rhoenrad(skelHandle, osgHandle,conf, "Humanoid in Rhoenrad");           
      robot=human;
      human->place(osg::Matrix::rotate(M_PI_2,1,0,0)*osg::Matrix::rotate(M_PI,0,0,1)
                   //   *osg::Matrix::translate(-.2 +2.9*i,0,1));
                   *osg::Matrix::translate(.2*i,2*i,.841/*7*/ +2*i));
      //      global.configs.push_back(human);
      
      
      if( fixedInAir){
        Primitive* trunk = human->getMainPrimitive();
      
        fixator = new FixedJoint(trunk, global.environment);
        //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) , 		   Axis(0,0,1), Axis(0,1,0));
        fixator->init(odeHandle, osgHandle, true, 0.25);
      }

      // attention: This sox here is now called SoML in selforg/controller
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr=true;
      sc.useHiddenModel=true;
      sc.someInternalParams=true;
      sc.useS=true;
      //   AbstractController* controller = new SoML(sc);
      AbstractController* controller = new Sox(1.0,true);

      controller->setParam("epsC",0.05);
      controller->setParam("epsA",0.05);
      controller->setParam("sense",3);
      controller->setParam("harmony",0.0);
      controller->setParam("causeaware",0.0);
      controller->setParam("Logarithmic",1.0);
      controller->setParam("damping",.001);

      //     controller = new SineController(1<<8 | 1<<9 | 1<<12); // only left arm
      // controller = new MotorBabbler();
      //AbstractController* controller = new SineController(0,SineController::Impulse);
            
      // create pointer to one2onewiring
      //     One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
      //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
      ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise());
      wiring->setParam("booster",.1);
      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, human, wiring);
      //agent->startMotorBabblingMode(5000);
      //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps

    // agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), 
    //                                                 M_PI*0.1, 90));
      global.agents.push_back(agent);
      global.configs.push_back(agent);      
    }// Several humanoids end
    
    
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control &&!pause){
      if(forwardforce!=0){
	// force forwards
	FOREACH(vector<OdeAgent*> , globalData.agents, a){
	  Primitive* body = (*a)->getRobot()->getMainPrimitive();
	  osg::Matrix pose = body->getPose();
	  // transform a local point ahead of robot into global coords
	  // note that the internal corrd of the main primitive has z towards the front
	  Pos point = (Pos(0,0,1)*pose ); 
	  point.z()=pose.getTrans().z();  // only use x,y component (this can be commented out)
	  Pos d = (point - pose.getTrans());
	  d.normalize();

	  dBodyAddForce(body->getBody(), 
			d.x()*forwardforce, d.y()*forwardforce, d.z()*forwardforce);
	  if(!forcepoint){
	    forcepoint = new Sphere(0.1);
	    forcepoint->init(odeHandle, 0, osgHandle /*osgHandle.changeAlpha(0.4)*/, 
			     Primitive::Geom | Primitive::Draw);
          }
	  forcepoint->setPosition(point);
	  forcepoint->update();
	}
      }
    }
    if(globalData.time>5 && fixator){
      if(fixator) delete fixator;
      fixator=0;	 
    }
  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    Substance s;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'x': 
	  if(fixator) delete fixator;
	  fixator=0;	 
	  return true;
	  break;
	case 'r': 
	  if(robot) {	    
	    Primitive* wheel = robot->getAllPrimitives().front();
	    Axis a(0,0,100);
	    wheel->applyTorque(Pos(wheel->toGlobal(a)));
	  }
	  return true;
	  break;
	case 'R': 
	  if(robot) {
	    Primitive* wheel = robot->getAllPrimitives().front();
	    Axis a(0,0,-100);
	    wheel->applyTorque(Pos(wheel->toGlobal(a)));
	  }
	  return true;
	  break;
	default:
	  return false;
	  break;
	}
    }
    return false;
  }

  void setupPlaygrounds(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){

    playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground)); 
    playground->setColor(Color(1.,1.,1.)); 
    //playground->setGroundTexture("Images/really_white.rgb");
    //        playground->setGroundTexture("Images/desert.jpg");
    //    playground->setGroundColor(Color(1.,1.,1.)); 
    //    playground->setGroundTexture("Images/sand.jpg");
    playground->setGroundTexture("Images/sand_bw.jpg");
    playground->setPosition(osg::Vec3(0,0,0));
    Substance substance;
    substance.toRubber(15);
    //   substance.toMetal(1);
    playground->setGroundSubstance(substance);
    global.obstacles.push_back(playground);
  }

  Joint* fixator;

  AbstractGround* playground; 
  OdeRobot* robot; 

  double forwardforce;

  Primitive* forcepoint;


};


int main (int argc, char **argv)
{ 
  ThisSim sim;
  sim.setCaption("lpzrobots Simulator             playfulmachines.com");
  return sim.run(argc, argv) ? 0 : 1;

}
 
