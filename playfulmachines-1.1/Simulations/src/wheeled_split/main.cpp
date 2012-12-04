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

struct ControlGen : public ControllerGenerator {
  ControlGen(double epsC, double epsA, double cInit) 
    : epsC(epsC), epsA(epsA), cInit(cInit) {}
  virtual ~ControlGen() {}
  virtual AbstractController* operator()( int index) { 
    AbstractController* c;
    c= new Sox(cInit);    
    c->setName("Sox " + itos(index));
    c->setParam("epsC",epsC);
    c->setParam("epsA",epsA);
    if(epsC==0) c->setParam("creativity",0);
    else  c->setParam("creativity",0.1);
    return c; 
  }
  double epsC;
  double epsA;
  double cInit;
};

struct BoxesDef {
  double xboxes;
  double yboxes;
  double boxdis;
  double xsize;
  double ysize;
  double zsize;
};

class ThisSim : public Simulation
{
public:
  enum SimType { SINGLE, THREE, LONGINMAZE, ROBOTCHAIN };

  StatisticTools* stats;
  Nimm2* nimm2;
  std::list<Joint*> joints;
  bool noslip;
  SimType simtype;
  bool track;
  double noise;
  double eps;
  double cInit;
  bool couplingChanged;

  ThisSim(SimType simtype, bool nolearning)
    : simtype(simtype)
  {
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");

    noise = 0.05; // simtype == ROBOTCHAIN ? 0.01 : 0.05;    
    if(nolearning){
      noise = 0.1;    
      cInit = 1.2;
      eps   = 0;
    }else{
      cInit = simtype == ROBOTCHAIN ? 1.0  : 0.8;    
      eps   = simtype == ROBOTCHAIN ? 0.01 : 0.1;
    }
    track    = false;    
    couplingChanged=false;
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {

    setCameraHomePos(Pos(-26.6849, 17.3789, 16.7798),  Pos(-120.46, -24.7068, 0));
    if(simtype == ROBOTCHAIN) setCameraMode(Follow);
    else setCameraMode(Static);
    bool plotOnlyOne=false;

    global.odeConfig.addParameter("coupling",&cInit,0,10,"coupling constant of controllers");
    global.odeConfig.setParam("noise",noise);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("gravity",-9.81);

    if(eps==0){
      Configurable::setName("Wheeled Robots Simulation");
      global.configs.push_back(this);
    }

    Playground* playground=0;
    bool boxes = false;
    BoxesDef b = {0,0,0,0,0,0};
    int num_robots=5;
    bool noSlip=false;

    Substance substance;
    switch(simtype){
    case THREE:
    case SINGLE:
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(15, 0.2, .5));
      break;
    case LONGINMAZE:
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(20, 0.2, .5));
      substance = Substance(2, 0 , 40, .5);
      playground->setGroundSubstance(substance);
      b.xboxes=5.0;
      b.yboxes=5.0;
      b.boxdis=4;
      b.xsize=2.0;
      b.ysize=2.0;
      b.zsize=.5;
      boxes=true;
      break;
    case ROBOTCHAIN:      
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(50, 0.2, 2.0));
      if(noSlip){
	substance.toPlastic(1);
	substance.slip=0;          
      } else {
	substance.toPlastic(2);
      }      
      b.xboxes=8.0;
      b.yboxes=8.0;
      b.boxdis=6;
      b.xsize=1.5;
      b.ysize=1.5;
      b.zsize=.8;
      boxes=true;
    }
    playground->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(playground);

    if(boxes){
      for (double j=0.0;j<b.xboxes;j++)
        for(double i=0.0; i<b.yboxes; i++)
          {
            PassiveBox* bb =
              new PassiveBox(odeHandle,
                             osgHandle.changeColor(Color(1.0f,0.2f,0.2f)),
                             osg::Vec3(b.xsize,b.ysize,b.zsize),0.0);
            bb->setTexture("Images/light_chess.rgb");
            bb->setPosition(Pos(b.boxdis*(i-(b.xboxes-1)/2.0),b.boxdis*(j-(b.yboxes-1)/2.0), 0.01));
            global.obstacles.push_back(bb);
          }
      
    }

    OdeAgent* agent;
    AbstractWiring* wiring;
    AbstractController *controller;
    OdeRobot* robot;

    switch (simtype){
    case SINGLE:
    case THREE:
      /// 2 wheeled CIGAR (LongVehicle)
      {
	Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
	nimm2conf.size = 1;
	nimm2conf.force = 5;
	nimm2conf.speed=20;
	//      nimm2conf.speed=15;
	nimm2conf.cigarMode=true;
        nimm2conf.cigarLength= simtype==SINGLE ? 2.0 : 3.0;
	nimm2conf.singleMotor=false;
	nimm2conf.boxMode=true;
	nimm2conf.boxWidth=1.5;      
	//      nimm2conf.visForce =true;
	nimm2conf.bumper=true;
	wiring = new One2OneWiring(new WhiteNormalNoise());
      
	controller = new OneControllerPerChannel(new ControlGen(eps,0.01,cInit),"OnePerJoint - LongVehicle",2);
        OdeHandle odeHandleR = odeHandle;
        if(simtype==SINGLE){
          odeHandleR.substance.toFoam(20);
        }
	robot = new Nimm2(odeHandleR, osgHandle, nimm2conf, "LongVehicle");

	robot->setColor(Color(.1,.1,.8));        
	if(plotOnlyOne)
	  agent = new OdeAgent(global, PlotOption(NoPlot));
	else
	  agent = new OdeAgent(global);
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[1]); 
	agent->init(controller, robot, wiring);
	if(track) 
	  agent->setTrackOptions(TrackRobot(true,true,false,true,"split_control",10));      
	robot->place(Pos(0,-2,.2));
	global.agents.push_back(agent);
	global.configs.push_back(agent);
      }
      if(simtype == SINGLE) break;
      /// 2 wheeled
      {        //      robot = new Nimm2(odeHandle);
	Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
	nimm2conf.size = 1;
	nimm2conf.force = 5;
	nimm2conf.speed=20;
	//      nimm2conf.speed=15;
	//      nimm2conf.cigarMode=true;
	nimm2conf.singleMotor=false;
	//      nimm2conf.boxMode=true;
	//      nimm2conf.visForce =true;
	//      nimm2conf.bumper=true;
	wiring = new One2OneWiring(new WhiteNormalNoise());
      
	controller = new OneControllerPerChannel(new ControlGen(eps,0.01,cInit),"OnePerJoint - TwoWheeled",2);
	robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "TwoWheeled");

	robot->setColor(Color(.9,.9,0.0));        
	agent = new OdeAgent(global);
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[1]); 
	agent->init(controller, robot, wiring);
	if(track) 
	  agent->setTrackOptions(TrackRobot(true,true,false,true,"split_control",10));      
	robot->place(Pos(2.,0.,.2));
	global.agents.push_back(agent);
	global.configs.push_back(agent);
      }
      /// 4 wheeled
      {
	wiring = new One2OneWiring(new WhiteNormalNoise());      
	controller = new OneControllerPerChannel(new ControlGen(eps,0.01,cInit),"OnePerJoint - FourWheeled",4);
	robot = new Nimm4(odeHandle, osgHandle, "FourWheeled",1.0,3.0,30.0);
	robot->setColor(Color(.9,.0,.0));        

	if(plotOnlyOne)
	  agent = new OdeAgent(global, PlotOption(NoPlot));
	else
	  agent = new OdeAgent(global);
	agent = new OdeAgent(global,PlotOption(NoPlot));
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[1]); 
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[2]); 
	agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[3]); 
	agent->init(controller, robot, wiring);
	if(track) 
	  agent->setTrackOptions(TrackRobot(true,true,false,true,"split_control",10));      
	robot->place(Pos(-2.,0.,.5));
	global.agents.push_back(agent);
	global.configs.push_back(agent);
      }
      break;
    case LONGINMAZE:
      {    
	for(int i=0; i<num_robots; i++){
	  Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
	  nimm2conf.size = 1;
	  nimm2conf.force = 5;
	  nimm2conf.speed=20;
	  //      nimm2conf.speed=15;
	  nimm2conf.cigarMode=true;
	  nimm2conf.cigarLength=2+(i*0.8);
	  nimm2conf.singleMotor=false;
	  nimm2conf.boxMode=true;
	  nimm2conf.boxWidth=1.5;
	  nimm2conf.wheelTexture="Images/tire_stripe.rgb";
	  //      nimm2conf.visForce =true;
	  nimm2conf.bumper=true;
	  wiring = new One2OneWiring(new WhiteNormalNoise());
	
	  controller = new OneControllerPerChannel(new ControlGen(eps,0.01,cInit),"OnePerJoint - LongVehicle " + std::itos(i) ,2);
	  robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "LongVehicle " + std::itos(i) );
	
	  robot->setColor(Color(.9,.9,0.0));        
	  agent = new OdeAgent(global);
	  agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
	  agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[1]); 
	  agent->init(controller, robot, wiring);
	  if(track) 
	    agent->setTrackOptions(TrackRobot(true,true,false,true,"split_control",10));      
	  robot->place(Pos(-10+i*4,-1.7,.6));
	  global.agents.push_back(agent);
	  global.configs.push_back(controller);
	}
      }
      break;
    case ROBOTCHAIN:
      {
	double distance = 1.1;
	std::vector<OdeRobot*> robots(num_robots);
	for (int j=-0; j<num_robots; j++) {
	  Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
	  nimm2conf.size = 1.6;
	  nimm2conf.force = 6;
	  nimm2conf.speed=20;
	  nimm2conf.cigarMode=true;
	  nimm2conf.singleMotor=false;
	  nimm2conf.boxMode=true;
	  nimm2conf.visForce =true;
	  nimm2conf.bumper=true;
	  //        wiring = new One2OneWiring(new WhiteUniformNoise());
	  wiring = new One2OneWiring(new WhiteNormalNoise());
        
	  controller = new OneControllerPerChannel(new ControlGen(eps,eps,cInit),"OnePerJoint " + std::itos(j));
	  if (j==1) {
	    nimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Red_" + std::itos(j)); 
	    nimm2->setColor(Color(1.0,.2,0.2));        
	    agent = new OdeAgent(global);
	    agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
	    agent->init(controller, nimm2, wiring);
	    if(track) 
	      agent->setTrackOptions(TrackRobot(true,true,false,true, "",10));
	  } else {
	    nimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Yellow_" + std::itos(j));
	    nimm2->setColor(Color(1.0,1.0,0));        
	    agent = new OdeAgent(global, PlotOption(NoPlot));
	    agent->init(controller, nimm2, wiring);
	  }
	  global.agents.push_back(agent);
	  global.configs.push_back(agent);
	  
	  ((OdeRobot*)nimm2)->place(Pos(j*(1.5+distance),0,0.11));	 
	  robots[j]=nimm2;
	}
	for(int j=0; j<num_robots-1; j++) {
	  Primitive* p1 = robots[j]->getMainPrimitive();
	  Primitive* p2 = robots[j+1]->getMainPrimitive();
	  Joint* joint = new BallJoint(p1,p2,(p1->getPosition()+p2->getPosition())/2.0);
	  joint->init(odeHandle,osgHandle,true,distance/6);
	  joints.push_back(joint);
	}
      }	
      break;
    }

  }

  /** optional additional callback function which is called every simulation step.
  Called between physical simulation step and drawing.
  @param draw indicates that objects are drawn in this timestep
  @param pause indicates that simulation is paused
  @param control indicates that robots have been controlled this timestep
   */
  void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(draw) {
      FOREACH(std::list<Joint*>, joints,j) {
        (*j)->update();
      }
    }
    if(couplingChanged){
      FOREACH(OdeAgentList, globalData.agents, a){
        cout << " agent " << (*a)->getName() <<endl;
        OneControllerPerChannel* ocpc = 
          dynamic_cast<OneControllerPerChannel*>((*a)->getController());
        if(ocpc){
          vector<AbstractController*> cs = ocpc->getControllers();
          FOREACH(vector<AbstractController*>, cs, c){
            Sox* s = dynamic_cast<Sox*>(*c);
            if(s){
              matrix::Matrix C = s->getC();
              C.toMapP(cInit,constant);
              s->setC(C);
              matrix::Matrix h = s->geth();
              h.toMapP(0,constant);
              s->seth(h);
              cout << "  changed coupling of " << s->getName() << endl; 
            }else
              cerr << "Controller is not Sox!" << endl; 
          }
        }else
          cerr << "Controller is not OneControllerPerChannel!" << endl; 
      }
      couplingChanged=false;
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    // if (down)
    // { // only when key is pressed, not when released
    //   switch ( (char) key )
    //   {
    //   default:
    //     return false;
    //     break;
    //   }
    // }
    return false;
  }

  virtual void notifyOnChange(const paramkey& key){
    if(key == "coupling"){
      couplingChanged=true;
    }
  }

  virtual void usage() const {
    printf("\t-single\tSingle wheeled robots in square arena\n");
    printf("\t-three\tThree wheeled robots in square arena (default)\n");
    printf("\t-maze\tLongVehicle in maze\n");
    printf("\t-chain\tRobotChain in maze\n");
    printf("\t-nolearning\tdisable learning (-eps 0, -cinit 1.1)\n");
    printf("\t-track\tenable tracking (trajectory is written and shown)\n");
    //    printf("\t-noslip\tdisable slipping\n");
    printf("\t-cinit\tinitial value of C\n");
    printf("\t-eps\tlearning rate\n");
    printf("\t-noise\tnoise strength\n");
  };

};


int main (int argc, char **argv){
  ThisSim::SimType simtype = ThisSim::THREE;
  if(Simulation::contains(argv,argc,"-single") != 0)
    simtype = ThisSim::SINGLE;
  if(Simulation::contains(argv,argc,"-three") != 0)
    simtype = ThisSim::THREE;
  if(Simulation::contains(argv,argc,"-maze") != 0)
    simtype = ThisSim::LONGINMAZE;
  if(Simulation::contains(argv,argc,"-chain") != 0)
    simtype = ThisSim::ROBOTCHAIN;
  bool nolearning = (Simulation::contains(argv,argc,"-nolearning") != 0); 


  ThisSim sim(simtype, nolearning);
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
  
  return sim.run(argc, argv) ? 0 : 1;
}
