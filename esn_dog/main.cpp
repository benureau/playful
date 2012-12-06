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
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/forceboostwiring.h>
#include <selforg/wiringsequence.h>
#include <selforg/derivativewiring.h>
// used controller
#include <selforg/sox.h>
#include <selforg/sinecontroller.h>

#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

//#include <ode_robots/vierbeiner.h>
#include "vierbeiner.h"

#include <ode_robots/joint.h>
#include <ode_robots/operators.h>
#include "groupController.h"
#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;
using namespace matrix;

bool barriers = false;
bool hippo    = false;
bool air      = false;

class ThisSim : public Simulation {
public:
  Joint* fixator;
  AbstractGround* playground; 
  double hardness;
  AbstractController *teachcontroller;
  Env env;

  ThisSim()
    : env(barriers ? Env::Stacked : Env::Normal){
    setCaption("Simulator by Martius et al");

    if(hippo){
      setTitle("HippoDog");
    }else if(air){      
      setTitle("Dog with weak forces");
    } else {
      setTitle("Dog on ground");    
    }
    
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-1.64766, 4.48823, 1.71381),  Pos(-158.908, -10.5863, 0));

    // initialization
    // - set noise to 0.0
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("noise",0.01); 
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("gravity", -6);
    //    global.odeConfig.setParam("cameraspeed", 250);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    bool useDerivativeWiring=false;

    // environemnt (type is set in constructor)
    env.numSpheres  = 0; 
    env.numBoxes    = 0;   
    env.numCapsules = 0;
    env.numSeeSaws  = 0;
    env.roughness   = 1.0; 

    // use Playground as boundary:
    if(barriers){
      env.widthground = 4;      
      env.distance    = 6;
      env.numgrounds  = 5;
      env.height      = 0.1;
      env.heightincrease =0.1;      
    }else {
      env.widthground = 32;            
      env.height      = 1.0;
    }

    env.create(odeHandle, osgHandle, global);
    global.configs.push_back(&env);

    
    // Boxpile* boxpile = new Boxpile(odeHandle, osgHandle, Pos(10,10,0.2), 
    //                                100,0, Pos(0.3,0.3,0.05), Pos(0.3,0.3,0.05)
    //                                );
    // boxpile->setColor("wall"); 
    // boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(0, 0 ,0.2));
    // global.obstacles.push_back(boxpile);
    


    for (int i=0; i< 1/*2*/; i++){ //Several dogs 

      //    VierBeinerConf conf = VierBeiner::getDefaultConf();
    VierBeinerConf conf = VierBeiner::getDefaultConfVelServos();

    conf.dampingFactor = .0; 
    conf.powerFactor = 1.3;
    if(air) conf.powerFactor = 0.3;
    if (hippo) conf.powerFactor = 1.5;
    
    conf.hipJointLimit = M_PI/3.0;              
    if ( barriers)  conf.hipJointLimit = M_PI/2.5;
    
    conf.kneeJointLimit = M_PI/3;        
    conf.legNumber = 4; /* for the dog's sake use only even numbers */
    conf.useBigBox = false;
    if(hippo) conf.useBigBox = false;
    conf.drawstupidface=true;
    conf.hippo = hippo;
    //    conf.onlyMainParameters = false; // all parameters

    OdeHandle doghandle = odeHandle;
    doghandle.substance.toRubber(10);
    VierBeiner* dog = new VierBeiner(doghandle, osgHandle,conf, "Dog");     
    //dog->place(osg::Matrix::translate(0,0,0.15));  
    dog->place(osg::Matrix::translate(0,0,.5 + 4*i));

    if(air || barriers){
      Primitive* trunk = dog->getMainPrimitive();
      fixator = new FixedJoint(trunk, global.environment);
      fixator->init(odeHandle, osgHandle);
    }
    
    // create pointer to one2onewiring
    //AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    
    double booster = 0.05;
    if(air) booster=0;

    AbstractWiring* wiring;
    if(!useDerivativeWiring){
      wiring = new ForceBoostWiring(new ColorUniformNoise(0.1), booster); 
    }else{
      //////////////// 
      AbstractWiring* fbw = new ForceBoostWiring(new NoNoise(), booster);
      DerivativeWiringConf dc = DerivativeWiring::getDefaultConf();
      dc.useId=true;
      dc.useFirstD=true;
      // dc.derivativeScale = 1.0;
      AbstractWiring* drw = new DerivativeWiring(dc, new ColorUniformNoise(0.1));  
      if(!useDerivativeWiring){ wiring = new WiringSequence(fbw, drw); // TODO booster bei derivative wiring ???
     }
    }

    AbstractController *controller = new Sox(.7, false);    
    //AbstractController *controller = new SineController();
    controller->setParam("Logarithmic", 1);
    controller->setParam("epsC",0.05);
    if ( hippo) controller->setParam("epsC",0.1);
    controller->setParam("epsA",0.01);
    controller->setParam("damping",0.0001);

    controller->setParam("period",300);
    controller->setParam("phaseshift",0);

    AbstractController* cont= new GroupController(controller);

    OdeAgent* agent = new OdeAgent(global);
    agent->init(cont, dog, wiring);
    if(!hippo){
    // add an operator to keep robot from falling over
      agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), 
                                                      M_PI*0.35, 10));
    }

    //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
    global.agents.push_back(agent);
    global.configs.push_back(agent);
  
    }// Several dogs end
        
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: x","release robot");
  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'x': 
	  if(fixator) delete fixator;
	  fixator=0;	 
	  return true;
	  break;
	default:
	  return false;
	  break;
	}
    }
    return false;
  }

  virtual void usage() const {
    printf("\t-barriers\tdog in barriers\n");
    printf("\t-hippo\thippo dog\n");
    printf("\t-air\tdog in air with weak forces\n");
  };

};


int main (int argc, char **argv)
{ 
  if (Simulation::contains(argv, argc, "-barriers")) {
    barriers=true;
  }
  if (Simulation::contains(argv, argc, "-hippo")) {
    hippo=true;
  }
  if (Simulation::contains(argv, argc, "-air")) {
    air=true;
  }


  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}
 
