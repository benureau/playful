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
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>

#include <ode_robots/passivebox.h>

#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>

//#include "sox.h"
#include <selforg/sox.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/onecontrollerperchannel.h>

#include <ode_robots/substance.h>

#include <stdio.h>
#include <string.h>


using namespace lpzrobots;
using namespace osg;
using namespace std;


class ThisSim : public Simulation
{
public:
  Nimm2* nimm2;
  bool track;
  double noise;
  double eps;
  double cInit;
  bool couplingChanged;
  bool noH;
  AbstractObstacle* trailer;
  FixedJoint* fj; // trailer to robot joint
  
  double friction;

  ThisSim(bool nolearning, bool noH)
  {
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
    noise = 0.1;
    trailer = 0;
    this->noH = noH;
    if(nolearning){
      noise = 0.1;    
      cInit = 1.1;
      eps   = 0;
    }else{
      cInit = 0.5;
      eps   = 1.0;
    }
    addParameter("coupling",&cInit,0,10,"change coupling constant of controller");
    track    = false;    
    couplingChanged=false;
    addParameterDef("friction", &friction, 1.8, 0, 5, "friction of trailer with floor");
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {

    setCameraHomePos(Pos(-26.6849, 17.3789, 16.7798),  Pos(-120.46, -24.7068, 0));
    setCameraMode(Static);

    global.odeConfig.setParam("noise",noise);
    global.odeConfig.setParam("realtimefactor",2);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("simstepsize",0.01);
    global.odeConfig.setParam("gravity",-9.81);


    Configurable::setName("Pulling Trailer Simulation");
    global.configs.push_back(this);
      

    Playground* playground=0;
    //bool noSlip=false;

    Substance substance;
    substance.toPlastic(1);
    substance.slip=0.000;          
    OdeHandle oh=odeHandle;
    oh.substance=Substance(0,0.05,40,.1);
    playground = new Playground(oh, osgHandle,osg::Vec3(30, .5, .5),1.0/17.0);
    playground->setPosition(osg::Vec3(0,0,0));
    playground->setGroundSubstance(substance);
    global.obstacles.push_back(playground);

    OdeAgent* agent;
    AbstractWiring* wiring;
    AbstractController *controller;
    OdeRobot* robot;

    /// 2 wheeled
    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1.6;
    nimm2conf.force = 6;
    nimm2conf.speed=20;
    nimm2conf.cigarMode=true;
    nimm2conf.sphereWheels=false;
    nimm2conf.cigarLength=3;
    nimm2conf.singleMotor=true;
    nimm2conf.boxMode=true;
    nimm2conf.visForce =true;
    nimm2conf.bumper=true;
    nimm2conf.wheelTexture="Images/tire_stripe.rgb";
    //    wiring = new One2OneWiring(new WhiteNormalNoise());
    wiring = new One2OneWiring(new ColorUniformNoise(.5));
    //wiring = new One2OneWiring(new WhiteUniformNoise());
    
    controller = new Sox(cInit, false);
    controller->setParam("epsC",eps);
    controller->setParam("epsA",eps/4);
    controller->setParam("s4avg",2);
    controller->setParam("sense",1);
    robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "TwoWheeled (single wheel mode)");
    
    robot->setColor(Color(.9,.9,0.0));        
    agent = new OdeAgent(global);    
    agent->init(controller, robot, wiring);
    if(track) 
	  agent->setTrackOptions(TrackRobot(true,true,false,true,"split_control",10));      
    robot->place(Pos(2.,0.,.2));
    global.agents.push_back(agent);
    global.configs.push_back(agent);
  }

  Substance getTrailerSubstance(){
    return Substance(friction, 0.005, 30, 0);
  }


  void removeMoveableBox(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                         GlobalData& global){
    if(trailer){
      ObstacleList::iterator i = find(global.obstacles.begin(), global.obstacles.end(), 
                                      trailer);
      if(i!=global.obstacles.end())
        global.obstacles.erase(i);
      delete trailer;
      delete fj;
    };
    trailer=0;
    fj=0;
    
  }
  
  void addMoveableBox(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                      GlobalData& global){
    if(trailer) return;
    if(global.agents.size()<1) return;
    Primitive* p = global.agents[0]->getRobot()->getMainPrimitive();
    // Position p = global.agents[0]->getRobot()->getPosition();
    double mass = 20;
    double length = 3;
    OdeHandle oh = odeHandle;
    //    oh.substance.toFoam(30);
    oh.substance = getTrailerSubstance();
    trailer = new PassiveBox(oh, osgHandle, osg::Vec3(length,1.5,.6), mass);
    trailer->setTexture("Images/light_chess.rgb");
    trailer->setPose(osg::Matrix::translate(2+length/2,0,0) * 
                     osg::Matrix::rotate(M_PI/2,osg::Vec3(0,1,0)) * p->getPose());
    global.obstacles.push_back(trailer);
    fj = new FixedJoint(trailer->getMainPrimitive(), p);    
    // HingeJoint* j = new HingeJoint(trailer->getMainPrimitive(), p, 
    //                                p->getPosition(),
    //                                Axis(0,1,0) * p->getPose());    
    fj->init(odeHandle, osgHandle,false);
    p->limitLinearVel(0); // stop robot
    // AbstractObstacle* o1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(.5,4,.5), mass);
    // AbstractObstacle* o2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(.5,4,.5), mass);
    // o1->setTexture("Images/light_chess.rgb");
    // o1->setPosition(Pos(p) + Pos(1.45,0,0));
    // global.obstacles.push_back(o1);
    // o2->setTexture("Images/light_chess.rgb");
    // o2->setPosition(Pos(p) + Pos(-1.45,0,0));
    // global.obstacles.push_back(o2);        
    //    FixedJoint* j = new FixedJoint(o1->getMainPrimitive(), o2->getMainPrimitive());    
    //    j->init(odeHandle, osgHandle);
  }

  /** optional additional callback function which is called every simulation step.
  Called between physical simulation step and drawing.
  @param draw indicates that objects are drawn in this timestep
  @param pause indicates that simulation is paused
  @param control indicates that robots have been controlled this timestep
   */
  void addCallback( GlobalData& globalData, bool draw, bool pause, bool control) {    
      FOREACH(OdeAgentList, globalData.agents, a){
        Sox* s = dynamic_cast<Sox*>((*a)->getController());
        if(s){
          if(couplingChanged){
            cout << " agent " << (*a)->getName() <<endl;
            matrix::Matrix C = s->getC();
            C.toMapP(cInit,constant);
            s->setC(C);
            matrix::Matrix h = s->geth();
            h.toMapP(0,constant);
            s->seth(h);
            cout << "  changed coupling of " << s->getName() << endl; 
            couplingChanged=false;
          } else if (noH){
            matrix::Matrix h = s->geth();
            h.toMapP(0,constant);
            s->seth(h);                       
          }         
        }else
          cerr << "Controller is not Sox!" << endl; 
      }
      if(globalData.agents.size()<1) return;
      // Primitive* p = globalData.agents[0]->getRobot()->getMainPrimitive();      
      // if(p){
      //   const double* vel = dBodyGetAngularVel( p->getBody() );
      //   dBodySetAngularVel(p->getBody(), vel[0], 0, vel[2]);        
      // }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                       GlobalData& globalData, int key, bool down) { 
    if (down)
      { // only when key is pressed, not when released
        switch ( (char) key ) {
        case 'b': addMoveableBox(odeHandle, osgHandle, globalData);
          return true;
          break;
        case 'B': removeMoveableBox(odeHandle, osgHandle, globalData);
          return true;
          break;
        default:
            return false;
          break;
        }
      }
    return false;
  }


  virtual void notifyOnChange(const paramkey& key){    
    if(key == "coupling"){
      couplingChanged=true;
    }
    if(key == "friction"){
      if(trailer){
        trailer->getMainPrimitive()->setSubstance(getTrailerSubstance());
      }

    }

  }

  virtual void usage() const {
    printf("\t-nolearning\tdisable learning (-eps 0, -cinit 1.1)\n");
    printf("\t-noH\tdisable h learning\n");
    printf("\t-track\tenable tracking (trajectory is written and shown)\n");
    printf("\t-cinit\tinitial value of C\n");
    printf("\t-eps\tlearning rate\n");
    printf("\t-noise\tnoise strength\n");
  };

};


int main (int argc, char **argv){
  bool nolearning = (Simulation::contains(argv,argc,"-nolearning") != 0); 
  bool noH = (Simulation::contains(argv,argc,"-noH") != 0); 
  

  ThisSim sim(nolearning, noH);
  sim.track = Simulation::contains(argv,argc,"-track") != 0;
  
  int index;
  index= sim.contains(argv,argc,"-cinit");
  if(index>0)
    sim.cInit = atof(argv[index]);  
  index= sim.contains(argv,argc,"-eps");  
  if(index>0){
    if(nolearning) {
      std::cerr << "-eps cannot be used with -nolearning -- ignored (press a key and enter to continue)\n"; 
      char c;
      std::cin >> c;
    } else sim.eps = atof(argv[index]);
  }
  
  index= sim.contains(argv,argc,"-noise");
  if(index>0)
    sim.noise = atof(argv[index]);    
  
  sim.setCaption("LpzRobots         www.playfulmachines.com");
  
  return sim.run(argc, argv) ? 0 : 1;
}
