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

// #include <selforg/sinecontroller.h>
#include "sinecontroller.h"

// #include <selforg/replaycontroller.h>
#include "replaycontroller.h"

// include simulation environment stuff
//#include <simulation.h>
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
//#include <ode_robots/skeleton.h>
#include "skeleton.h" // use local version

#include <ode_robots/joint.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/addsensors2robotadapter.h>

#include <ode_robots/operators.h>
#include <ode_robots/tmpprimitive.h>

#include <ode_robots/speedsensor.h>
#include <ode_robots/addsensors2robotadapter.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>

#include <selforg/soml.h>
#include <selforg/sox.h>
#include "groupController.h"
//#include "sox.h"

#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;
using namespace matrix;


#define ROBOTSTOREFILE "humanoid_initial.rob"

enum SimType { Normal, Rescue, Fight, Reck, Bungee };

class ThisSim : public Simulation {
public:
  SimType type;
  Env env;
  bool useSine;
  char* filename;
  int teachTime;

  Joint* fixator;
  Joint* reckLeft;
  Joint* reckRight;
  PassiveCapsule* reck;
  Playground* playground;
  //  AbstractObstacle* playground;
  double hardness;
  Substance s;

  ThisSim(SimType type, bool useSine, char* filename, int teachTime)
    : type(type), useSine(useSine), filename(filename), teachTime(teachTime) {
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    if(type==Rescue)
      addColorAliasFile("overwriteGroundColor.txt");
    setGroundTexture("Images/whiteground.jpg");

    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");

  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos (Pos(3.96562, 9.00244, 2.74255),  Pos(157.862, -12.7123, 0));

    setCameraMode(Static);

    int humanoids = 1;
    bool useExtendedModel = false;

    bool fixedInAir = false;
    env.type=Env::Normal;
    global.odeConfig.setParam("noise",0.01); //for more variety
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize",0.01);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity", -9.81);

    switch(type){
    case Normal:
      fixedInAir = true;
      env.numSeeSaws = 0;
      env.roughness  = 2.5;
      break;
    case Bungee:
      break;
    case Reck:
      env.height=2.0;
      break;
    case Rescue:
      setCameraHomePos (Pos(1.97075, 4.99419, 2.03904),  Pos(159.579, -13.598, 0));
      //  env.type       = Env::OpenPit;
      env.type       = Env::Pit;
      env.pitsize    = 1.0;//.9;
      env.thickness  = .1;
      env.height     = 1.4;
      env.roughness  = 2.0;
      env.hardness   = 30;
      env.numSeeSaws = 0;

      // global.addTmpObject(new TmpDisplayItem(new OSGText("plasdpaldss",16),
      //                                        TRANSM(20,30,0),
      //                                        osgHandle.getColor("hud"))
      //                     , 60);

      break;
    case Fight:
      env.type        = Env::Normal;
      global.odeConfig.setParam("gravity", -4);
      //      global.odeConfig.setParam("noise",0.0);
      env.widthground = 5;
      env.height      = 0.5;
      env.roughness   = 3;
      humanoids       = 2;
      env.numSeeSaws  = 0;
      break;
    }

    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0;
    env.numBoxes    = 0;
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);

    global.configs.push_back(&env);
    //    global.configs.push_back(this);

    fixator=0;
    reckLeft = reckRight = 0;
    reck = 0;

   for (int i=0; i< humanoids; i++){ //Several humans
     bool reckturner = (type==Reck);
     if (i>0) reckturner=false;

     // normal servos
     // SkeletonConf conf = Skeleton::getDefaultConf();
     // velocity servos
     SkeletonConf conf = Skeleton::getDefaultConfVelServos();

     OsgHandle skelOsgHandle=osgHandle.changeColorSet(i);
     double cInit = 0.8;
     double initHeight=0.8;

     conf.useBackJoint = true;
     conf.backSideBend = true;
     conf.movableHead  = true;
     conf.backJointLimit=M_PI/4;

     //Crawling power factor
     //Create a new type Crawling? How to use the type?
     conf.powerFactor = 1;
     conf.hipPower    = 90;
     conf.hip2Power   = 90;
     conf.neckPower    = 2;
     conf.kneePower    = 45;
     conf.anklePower    = 5;
     conf.armPower    = 50;
     conf.elbowPower    = 40;
     conf.pelvisPower    = 90;
     conf.backPower    = 10;

     switch(type){
     case Normal:
       conf.powerFactor = 1.5;
       conf.dampingFactor = .0;
       useExtendedModel = false;
       initHeight = 0.5;
       break;
     case Reck:
       conf.powerFactor = 0.3;
       conf.relForce = 2;
       conf.dampingFactor = .0;
       conf.handsRotating = true;
       initHeight = 0.45;
       //       conf.armPower = 30;
       break;
     case Rescue:
       conf.powerFactor = 1.25;
       conf.dampingFactor = .0;
       cInit = 0.99;
       break;
     case Bungee:
       conf.powerFactor = .2;
       conf.dampingFactor = .0;
       break;
     case Fight:
       conf.powerFactor = 1.0;
       conf.useGripper=true;
       conf.dampingFactor = .0;
       conf.gripDuration = 10;
       conf.releaseDuration = 5;
       break;
     }

     OdeHandle skelHandle=odeHandle;
     // skelHandle.substance.toMetal(1);
     //     skelHandle.substance.toPlastic(.5);//TEST sonst 40
     // skelHandle.substance.toRubber(5.00);//TEST sonst 40


     Skeleton* human0 = new Skeleton(skelHandle, skelOsgHandle,conf, "Humanoid" + itos(i));
     int numberContext=0; // adjust if you add additional sensor.
     //to add sensors use these lines
     // std::list<Sensor*> sensors;
     // //additional sensor
     //sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
     // speed sensor
     // sensors.push_back(new SpeedSensor(1,SpeedSensor::TranslationalRel));
     //  AddSensors2RobotAdapter* robot =
     // OdeRobot* robot = new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);
     OdeRobot* robot=human0;

     OdeRobot* human = human0;
     robot->place( ROTM(M_PI_2,1,0,0)*ROTM( i%2==0 ? M_PI : 0,0,0,1)
		  //*TRANSM(.2*i,2*i,.841/*7*/ +2*i));
                   * TRANSM(1*i, 0.10*i, initHeight));

     if( fixedInAir){
       Primitive* trunk = human->getMainPrimitive();

       fixator = new FixedJoint(trunk, global.environment);
       //       // fixator = new UniversalJoint(trunk, global.environment, Pos(0, 1.2516, 0.0552) , 		   Axis(0,0,1), Axis(0,1,0));
       fixator->init(odeHandle, osgHandle);
     }else if(reckturner){
       Primitive* leftHand = human->getAllPrimitives()[Skeleton::Left_Hand];
       Primitive* rightHand = human->getAllPrimitives()[Skeleton::Right_Hand];

       createOrMoveReck(odeHandle, osgHandle.changeColor("wall"), global,
                        leftHand->getPosition().z());

       reckLeft = new SliderJoint(leftHand, reck->getMainPrimitive(), leftHand->getPosition(), Axis(1,0,0));
       reckLeft->init(odeHandle, osgHandle,false);
       reckRight = new SliderJoint(rightHand, reck->getMainPrimitive(), rightHand->getPosition(), Axis(1,0,0));
       reckRight->init(odeHandle, osgHandle,false);

       //       reck = new OSGCapsule(0.02,env.widthground/2);
       //       reck->init(osgHandle.changeColor("Silbergrau"), OSGPrimitive::Low);
       //       reck->setMatrix(ROTM(M_PI_2,0,1,0) * TRANSM((leftHand->getPosition() + rightHand->getPosition())*0.5));
     }

     // create pointer to one2onewiring
     One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));


     AbstractController* controller;
     if(useSine){
       controller = new SineController();
       controller->setParam("period",200);
     } else if (filename != 0 && teachTime==0) {
       // No teaching: if we teach then we use the replay as motorbabbler, see below
       AbstractController* replay;
       replay = new ReplayController(filename, true);
       controller = new GroupController(replay,numberContext);
     } else {
       AbstractController* sox;
       sox = new Sox(cInit, useExtendedModel);

       sox->setParam("Logarithmic",1);
       sox->setParam("epsC",0.005);
       sox->setParam("epsA",0.01);
       sox->setParam("s4avg",1);
       sox->setParam("s4delay",1);
       sox->setParam("sense",4);

       // controller = sox; // or
       controller = new GroupController(sox,numberContext);
     }

     switch(type){
     case Normal:
       controller->setParam("epsC",0.1);
       break;
     case Reck:
       controller->setParam("damping",0.0001);
       break;
     case Rescue:
       break;
     case Bungee:
       break;
     case Fight:
       controller->setParam("damping",0.0003);
       controller->setParam("epsC",0.1  );
       global.odeConfig.setParam("noise",0.0);
       break;
     }


     // create pointer to agent
     // initialize pointer with controller, robot and wiring
     // push agent in globel list of agents
     OdeAgent* agent = new OdeAgent(global, i==0 ? plotoptions : list<PlotOption>());
     agent->init(controller, robot, wiring);
     //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps

     if (!useSine && filename != 0 && teachTime) { // use the replay as motorbabbler
       AbstractController* replay;
       replay = new ReplayController(filename, true);
       agent->startMotorBabblingMode(teachTime, replay);
     }


    // agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1),
    //                                                 M_PI*0.4, 1));
     switch(type){
     case Normal:
       break;
     case Reck:
       break;
     case Rescue:
       break;
     case Bungee:
       agent->addOperator(new PullToPointOperator(Pos(0,0,3),30,true,
                                                  PullToPointOperator::Z,
                                                  0, 0.1, true));
       break;
     case Fight:
       //agent->addOperator(new BoxRingOperator(Pos(0,0,2), 1.5, 0.2, 200, true));
       agent->addOperator(new BoxRingOperator(Pos(0,0,1.2), env.widthground/2.0,
                                              0.2, 200, false));
       // agent->addOperator(new PullToPointOperator(Pos(0,0,0.5),3,false,
       //                                            PullToPointOperator::XYZ,
       //                                            1, 0.0));

       break;
     }

     // save robot
     if(i==0) human->storeToFile(ROBOTSTOREFILE);

     global.configs.push_back(agent);
     global.agents.push_back(agent);

     //
   }// Several humans end

   // connect grippers
   if(type==Fight){
     Skeleton* h1 = dynamic_cast<Skeleton*>(global.agents[0]->getRobot());
     Skeleton* h2 = dynamic_cast<Skeleton*>(global.agents[1]->getRobot());
     if(h1 && h2){
        FOREACH(GripperList, h1->getGrippers(), g){
          (*g)->addGrippables(h2->getAllPrimitives());
        }
        FOREACH(GripperList, h2->getGrippers(), g){
          (*g)->addGrippables(h1->getAllPrimitives());
        }
     }else{
       fprintf(stderr,"Cannot convert Humanoids!");
     }
   }
  };


  void createOrMoveReck(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                GlobalData& global, double amount){
    if(type==Reck) {
      if(fixator) delete fixator;
      if(!reck){
        reck = new PassiveCapsule(odeHandle, osgHandle,
                                  0.02,env.widthground, 1.0);
        reck->setPose(ROTM(M_PI_2,0,1,0));
        global.obstacles.push_back(reck);

      }
      Primitive* r = reck->getMainPrimitive();
      r->setPosition(r->getPosition()+Pos(0.,0,amount));
      fixator = new FixedJoint(r, global.environment);
      fixator->init(odeHandle, osgHandle, false);
    }
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    if(reck){
      au.addKeyboardMouseBinding("Sim: b/B","move high bar down/up");
    }
  }


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                       GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'X':
	case 'x':
	  if(fixator) delete fixator;
	  fixator=0;
          //          globalData.agents[0]->setParam("pelvispower",0.0);
	  return true;
	  break;
	case 'b':
          createOrMoveReck(odeHandle, osgHandle, globalData, -0.1);
	  return true;
	  break;
	case 'B':
          createOrMoveReck(odeHandle, osgHandle, globalData,  0.1);
	  return true;
	  break;
	case 'l':
          {
            globalData.agents[0]->getRobot()->restoreFromFile(ROBOTSTOREFILE);
            globalData.agents[0]->getWiring()->reset();
          }
	  return true;
	  break;
	case 'i':
	  if(playground) {
	    s.hardness*=1.5;
	    cout << "hardness " << s.hardness << endl;
	    playground->setSubstance(s);
	  }
	  return true;
	  break;
	case 'j':
	  if(playground) {
	    s.hardness/=1.5;
	    cout << "hardness " << s.hardness << endl;
	    playground->setSubstance(s);
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

  virtual void usage() const {
    printf("\t-sine\tuse Sine controller\n");
    printf("\t-replay\treplay a file (e.g. from human data)\n");
    printf("\t-teach TIME\tuse the replay to teach the Sox controller for TIME steps, otherwise only ESN \n");
    printf("\t-rescue\thumanoid in a pit\n");
    printf("\t-reck\thumanoid at a reck bar\n");
    printf("\t-fight\ttwo humanoid in a fighting arena\n");
    printf("\t-bungee\ta weak humanoid attached to bungee\n");
    printf("\t-soml ratio\tuse multiplayer controller with ratio hidden units\n");
  };

};

int main (int argc, char **argv)
{
  SimType type=Normal;
  int index;
  if (Simulation::contains(argv, argc, "-rescue")) {
    type= Rescue;
  }
  if (Simulation::contains(argv, argc, "-reck")) {
    type= Reck;
  }
  if (Simulation::contains(argv, argc, "-fight")) {
    type= Fight;
  }
  if (Simulation::contains(argv, argc, "-bungee")) {
    type= Bungee;
  }
  bool useSine;
  useSine = Simulation::contains(argv, argc, "-sine");

  int teachTime = 0;
  index = Simulation::contains(argv, argc, "-teach");
  if (index>0 && index<argc) {
    teachTime=atoi(argv[index]);
  }


  index = Simulation::contains(argv, argc, "-replay");
  char* filename=0;
  if (index>0 && index<argc) {
    filename=argv[index];
  }



  ThisSim sim(type, useSine, filename, teachTime);
  return sim.run(argc, argv) ? 0 : 1;

}

