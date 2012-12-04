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
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>


#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/onecontrollerperchannel.h>
#include <selforg/forceboostwiring.h>
#include <selforg/crossmotorcoupling.h>

#include <selforg/sos.h>
#include <selforg/sox.h>
#include <selforg/soxexpand.h>
#include <selforg/soml.h>

#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/schlangeservo2.h>
#include <ode_robots/caterpillar.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/hexapod.h>
#include <ode_robots/vierbeiner.h>
#include <ode_robots/skeleton.h>
#include <ode_robots/robotchain.h>

#include "environment.h"
#include <ode_robots/operators.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

bool empty=false;

// generates controller for splitcontrol
struct ControlGen : public ControllerGenerator {
  ControlGen(double eps, double epsA,double cInit)
    :eps(eps),epsA(epsA), cInit(cInit) {}
  virtual ~ControlGen(){}
  virtual AbstractController* operator()( int index) { 
    AbstractController* c;
    c= new Sos(cInit);    
    c->setParam("epsC",eps);
    c->setParam("epsA",epsA);
    return c; 
  }
  double eps;
  double epsA;
  double cInit;
};

// generates controller for splitcontrol
struct ControlGenSox : public ControllerGenerator {
  ControlGenSox(double eps, double epsA,double cInit)
    :eps(eps),epsA(epsA), cInit(cInit) {}
  virtual ~ControlGenSox(){}
  virtual AbstractController* operator()( int index) { 
    AbstractController* c;
    c= new Sox(cInit);    
    c->setParam("epsC",eps);
    c->setParam("epsA",epsA);
    c->setParam("sense",4);
    c->setParam("Logarithmic",1);
    return c; 
  }
  double eps;
  double epsA;
  double cInit;
};

// coupling for robot chain
matrix::Matrix getCouplingTwo(double alpha, int index, int num){
  matrix::Matrix B(1,4);
  // index is wheel number: 0,1 left,right of front robot
  // num is number of wheels in total
  // front robot has front ir sensors (order: left(0) right(1))
  // rear robot has  back ir sensors  (order: right(2) left(3))
  if(index==0){
    B.val(0,0)=0.5;  // right front wheel gets a bit form left front IR
    B.val(0,1)=-1;   // left front wheel gets from right front IR
    B.val(0,2)=0;
    B.val(0,3)=0;
  } else if(index==1){
    B.val(0,0)=-1;   // right front wheel gets from left front IR
    B.val(0,1)=0.5;    
    B.val(0,2)=0;
    B.val(0,3)=0;    
  } else if(index==num-2){
    B.val(0,0)=0;
    B.val(0,1)=0;    
    B.val(0,2)=1;    // left rear wheel gets from right rear IR
    B.val(0,3)=-0.5;    
  }else if(index==num-1){
    B.val(0,0)=0;
    B.val(0,1)=0;    
    B.val(0,2)=-0.5;
    B.val(0,3)=1;   // right rear wheel gets from left rear IR
  }
  B*=alpha;
  return B;
}

// generates controller for splitcontrol
struct ControlGenSoxExpand : public ControllerGenerator {
  ControlGenSoxExpand(double eps, double epsA,double cInit, int num)
    :eps(eps),epsA(epsA), cInit(cInit), num(num) {}
  virtual ~ControlGenSoxExpand(){}
  virtual AbstractController* operator()( int index) { 
    AbstractController* c;
    SoxExpandConf sc = SoxExpand::getDefaultConf();
    sc.numberContextSensors=4;
    sc.initFeedbackStrength=cInit;
    sc.contextCoupling = getCouplingTwo(3.0, index, num);   
    
    c= new SoxExpand(sc);
    //    c->setName(std::string("SoxExpand ") + (index==0? "left" : "right"));
    c->setParam("epsC",eps);
    c->setParam("epsA",epsA);
    c->setParam("sense",4);
    c->setParam("Logarithmic",1);
    return c; 
  }
  double eps;
  double epsA;
  double cInit;
  int num;
};


// predicate that matches agents that have the same name prefix
struct agent_match_prefix : public unary_function<const OdeAgent*, bool> {
    agent_match_prefix(string nameprefix)
      : nameprefix(nameprefix) {
      len=nameprefix.length();
    }
    bool operator()(const OdeAgent* a) {
      if(!a || !a->getRobot()) return false;
      return nameprefix.compare(0,len, a->getRobot()->getName(),0,len) == 0;
    }

    string nameprefix;
    int len;
  };

class ThisSim : public Simulation {
public:

  Env env;
  double lastRobotCreation;
  bool useMultilayer;

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
    global.odeConfig.addParameterDef("multilayer",&useMultilayer, false, "use multilayer controller (SoML)");
    
    int numHexapods      = 0;
    int numSphericals    = 0;
    int numSnakes        = 0;
    int numDogs          = 0;
    int numHumanoids     = 0;
    int numSliderWheelie = empty ? 0 : 1;
    int numLongVehicle   = 0;
    int numRobotChains   = 0;
    int numCaterPillars  = 0;
    bool fighters        = empty ? false: true;

    setCameraHomePos(Pos(-1.14109, 20.0043, 8.34801),  Pos(-179.015, -24.8423, 0));
    setCameraMode(Static);
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.01);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("gravity",-6);

    // environemnt type
    env.type           = Env::BoxRing;
    env.useColorSchema = true;
    env.pitPosition    = Pos(-3,5,0.2);
    env.pitsize        = 3.5;
    env.heightInner    = 0.5;

    env.widthground  = 18;
    env.height       = 1; // 0;//1;
    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 5; 
    env.numBoxes    = 1;   
    env.numCapsules = 0;
    env.numSeeSaws  = 1;
    env.numBoxPiles = 2;

    env.hardness = 30;

    env.placeObstacles(odeHandle, osgHandle, global);
    global.configs.push_back(&env);
                
    // So wird das mit allen Robotern aussehen, createXXX, siehe unten
    for(int r=0; r < numHexapods ; r++) { 
      createHexapod(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,1+1*r));
    }    
    for(int r=0; r < numSphericals; r++) {
      createSpherical(odeHandle, osgHandle, global, osg::Matrix::translate(1,5,0+1*r));
    }
    for(int r=0; r < numSnakes; r++) {
      createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4-r,0.2));
    }
    for(int r=0; r < numDogs; r++) {
      createDog(odeHandle, osgHandle, global, osg::Matrix::translate(-4,4+r,0.2));
    }
    for(int r=0; r < numHumanoids; r++) {
      createHumanoid(odeHandle, osgHandle, global, osg::Matrix::translate(1,-1,1+1.5*r),
                     "Humanoid");
    }
    for(int r=0; r < numSliderWheelie; r++) {
      createArmband(odeHandle, osgHandle, global, osg::Matrix::translate(5+r,0,0.5));
    }
    for(int r=0; r < numLongVehicle; r++) {
      createLongVehicle(odeHandle, osgHandle, global, osg::Matrix::translate(2,-4-r,0.5));
    }
    for(int r=0; r < numRobotChains; r++) {
      createChain(odeHandle, osgHandle, global, osg::Matrix::translate(-2,-2-r,0.5));
    }
    for(int r=0; r < numCaterPillars; r++) {
      createCaterPillar(odeHandle, osgHandle, global, osg::Matrix::translate(-4-r,5+r,0.5));
    }
    if(fighters) {
      createFighters(odeHandle, osgHandle, global, osg::Matrix::translate(env.pitPosition));
    }
       

    lastRobotCreation=-.5;    
  }

  bool removeRobot(GlobalData& global, const string& nameprefix){
    OdeAgentList::reverse_iterator i = 
      find_if(global.agents.rbegin(), global.agents.rend(), agent_match_prefix(nameprefix));
    if(i!=global.agents.rend()){
      printf("Removed robot %s\n", (*i)->getRobot()->getName().c_str());
      OdeAgent* a = *i;
      global.agents.erase(i.base()-1);
      removeElement(global.configs, a);
      delete a;
      return true; 
    }
    return false;
  }
  
  OdeAgent* createHexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                          GlobalData& global, osg::Matrix pose, string name="Hexapod"){
    bool normal = (name == "Hexapod");
    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
    myHexapodConf.coxaPower          = 1.5;
    myHexapodConf.tebiaPower         = 0.8;
    myHexapodConf.coxaJointLimitV    = .9; // M_PI/8;  // angle range for vertical dir. of legs
    myHexapodConf.coxaJointLimitH    = 1.3; //M_PI/4;
    myHexapodConf.tebiaJointLimit    = 1.8; // M_PI/4; // +- 45 degree
    myHexapodConf.percentageBodyMass = .5;
    myHexapodConf.useBigBox          = false;
    myHexapodConf.tarsus             = true;
    myHexapodConf.numTarsusSections  = 1;
    myHexapodConf.useTarsusJoints    = true;
    

    OsgHandle rosgHandle=osgHandle.changeColorSet(num+normal);
    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);    
    OdeRobot* robot = new Hexapod(rodeHandle, rosgHandle, myHexapodConf, name);
    robot->place(osg::Matrix::rotate(M_PI*0,1,0,0)*pose);
    
    ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.1),0.05);
    //    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    AbstractController* controller;
    if(useMultilayer){
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr        = true;
      sc.useHiddenModel        = true;
      sc.hiddenContrUnitsRatio = 1.0;
      sc.hiddenModelUnitsRatio = 1.0;
      sc.useS = false;
      controller = new SoML(sc);
    }else{      
      controller = new Sox(1.2, false);
    }


    controller->setParam("epsC",0.1);
    controller->setParam("epsA",0.05);
    controller->setParam("Logarithmic",1);
    controller->setParam("sense",1.5);
    wiring->setParam("booster",.05);
 

    OdeAgent*       agent  = new OdeAgent( global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);
    
    // add an operator to keep robot from falling over
    agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), 
                                                    M_PI*0.3, 30));

    global.agents.push_back(agent);
    global.configs.push_back(agent);      
    return agent;
  }

  OdeAgent* createDog(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                      GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("Dog");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    VierBeinerConf conf = VierBeiner::getDefaultConf();
    conf.hipJointLimit = M_PI/2;        
    conf.kneeJointLimit = M_PI/3;        
    conf.legNumber = 4; /* for the dog's sake use only even numbers */
    conf.useBigBox = false;
    conf.drawstupidface=true;

    OsgHandle rosgHandle=osgHandle.changeColorSet(num);
    OdeHandle rodehandle = odeHandle;
    rodehandle.substance.toRubber(10);
    VierBeiner* robot = new VierBeiner(rodehandle, rosgHandle, conf, name);     
    robot->place(osg::Matrix::translate(-4,0,2)*pose);
    
    AbstractController *controller = new Sox(1.1, false);    
    controller->setParam("Logarithmic", 1);
    controller->setParam("epsC",0.05);
    controller->setParam("epsA",0.01);
    controller->setParam("damping",0.00001);

    AbstractWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.1),0.1);
    //    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*       agent  = new OdeAgent( global, PlotOption(NoPlot));
    agent->init(controller, robot, wiring);

    agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), 
                                                    M_PI*0.4, 1));

    //    wiring->setParam("booster",.2);

    global.agents.push_back(agent);
    global.configs.push_back(agent);      
    return agent;
  }


  OdeAgent* createSpherical(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                            GlobalData& global, osg::Matrix pose){

    // find robot and do naming
    string name("Spherical");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.motorsensor=false;
    conf.irAxis1=true;
    conf.irAxis2=true;
    conf.irAxis3=true;
    conf.motor_ir_before_sensors=false;
    conf.diameter = 1; 
    conf.drawIRs = RaySensor::drawRay;
    conf.pendularrange= 0.25;
    conf.motorpowerfactor = 150;     
    
    OdeRobot* sphere = 
      new Sphererobot3Masses ( odeHandle, osgHandle.changeColor("Green"), 
                               conf, name, 0.2); 
    sphere->place(pose);    
    AbstractController* controller = new Sox();
    controller->setParam("Logarithmic",1);
    controller->setParam("pseudo",3);
    One2OneWiring*      wiring     = new One2OneWiring ( new WhiteUniformNoise() );
    OdeAgent*           agent      = new OdeAgent ( global, PlotOption(NoPlot));
    agent->init ( controller, sphere, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);      
    return agent;
  }
  

  OdeAgent* createSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                        GlobalData& global, osg::Matrix pose, string name = "Snake"){    
    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), 
                       agent_match_prefix(name));
    bool outside = (name == "Snake");
    name += "_" + itos(num+1);        
    SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
    snakeConf.useServoVel  = true;
    snakeConf.segmNumber   = 9+3*num;
    snakeConf.segmDia      = 0.12;
    snakeConf.segmMass     = 0.2;    
    snakeConf.segmLength   = 0.8;
    snakeConf.jointLimit   = M_PI/3;
    snakeConf.motorPower   = 1;
    snakeConf.frictionJoint = 0;
    snakeConf.frictionRatio = 0.1;
    snakeConf.useSpaces = true;
    OdeHandle snakeHandle(odeHandle);
    if(outside)
      snakeHandle.substance.toPlastic(.1);
    else
      snakeHandle.substance.toPlastic(0.01);//TEST5);

    OsgHandle snakeOsgH = osgHandle.changeColorSet((num)%3);

    OdeRobot*    robot     = new SchlangeServo2 ( snakeHandle, snakeOsgH, snakeConf, name);
    if(outside){
      robot->place(pose*TRANSM(-1,-1, 1)); 
    }else{
      robot->place(pose*TRANSM(0,0, 0.8*snakeConf.segmNumber)); 
    }

    ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.01), 0.0);
    AbstractController* controller;
    //   useMultilayer=true;
    if(useMultilayer){
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr        = true;
      sc.useHiddenModel        = true;
      sc.hiddenContrUnitsRatio = 1.0;
      sc.hiddenModelUnitsRatio = 1.0;
      sc.useS = false;
      controller = new SoML(sc);
    }else{      
      controller = new Sox(.9, true);
    }

    controller->setParam("Logarithmic",1);
    controller->setParam("epsC",0.05);
    controller->setParam("epsA",0.01);
    controller->setParam("Logarithmic",1);
    controller->setParam("causeaware",0.0);
    wiring->setParam("booster",.07);

    OdeAgent*           agent      = new OdeAgent( global, PlotOption(NoPlot) );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }
  
  OdeAgent* createHumanoid(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                           GlobalData& global, osg::Matrix pose, 
                           string name, bool fighter = false){    

    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    // only 2 fighters allowed
    if(fighter && num > 1) return 0;
    name += "_" + itos(num+1);
        
    // normal servos
    //SkeletonConf conf   = Skeleton::getDefaultConf();
    // velocity servos
    SkeletonConf conf = Skeleton::getDefaultConfVelServos();
    
    conf.massfactor  = 1;
    conf.relLegmass  = 1;
    conf.relFeetmass = 1;
    conf.relArmmass  = 1;       //1.0;
    
    // conf.ankleJointLimit = 0.001; //!
    // conf.pelvisPower     = 20;
     
    conf.powerFactor  = 1;
    
    conf.useBackJoint     = true;
    conf.jointLimitFactor = 1.4;

    if(fighter){
      conf.powerFactor  = 1;
      conf.useGripper=true;
      conf.dampingFactor = .0;
      conf.gripDuration = 10; 
      conf.releaseDuration = 5;
    }else{
      conf.powerFactor  = 0.2;
      conf.dampingFactor = .0;
    }

    
    // conf.irSensors = true;    
    
    OdeHandle skelHandle=odeHandle;
    OsgHandle skelOsgHandle=osgHandle.changeColorSet(num);

    // skelHandle.substance.toMetal(1);
    // skelHandle.substance.toPlastic(.5);//TEST sonst 40
    // skelHandle.substance.toRubber(5.00);//TEST sonst 40
    
    
    Skeleton* human = new Skeleton(skelHandle, skelOsgHandle, conf, name);
    // // additional sensor
    // std::list<Sensor*> sensors;
    // // sensors.push_back(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
    // AddSensors2RobotAdapter* human = 
    //   new AddSensors2RobotAdapter(skelHandle, osgHandle, human0, sensors);
    // global.configs.push_back(human0);
    
    // place the robots such that they look at each other
    //  the fighters are also offset sidewards
    human->place( ROTM(M_PI_2,1,0,0)*ROTM( num%2==0 ? M_PI : 0,0,0,1)
                  * (fighter ? TRANSM(1*num, 0.1*num, 0) : TRANSM(0,0.4*num, 0)) * pose);

    
    AbstractController* controller;
    useMultilayer=false;
    if(useMultilayer){
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr        = true;
      sc.useHiddenModel        = false;
      sc.hiddenContrUnitsRatio = 1.0;
      sc.hiddenModelUnitsRatio = 1.0;
      sc.useS = false;
      controller = new SoML(sc);
    }else{      
      SoxConf sc = Sox::getDefaultConf();
      sc.useExtendedModel=false;
      if(!fighter)
        sc.initFeedbackStrength=1.1;
      controller = new Sox(sc);
    }

    if(fighter){
      controller->setParam("epsC",0.05);
      controller->setParam("epsA",0.1);
    }else{
      controller->setParam("epsC",0.2);
      controller->setParam("epsA",0.05);
    }
    controller->setParam("Logarithmic",1);
    controller->setParam("sense",1);
    controller->setParam("harmony",0);
    controller->setParam("causeaware",0.0);
              
    // One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.01), 0.0);
    OdeAgent*      agent  = new OdeAgent(global);
    agent->init(controller, human, wiring);
    if(!fighter){
      // // add an operator to keep robot up
      // LiftUpOperatorConf lc = LiftUpOperator::getDefaultConf();
      // lc.height = 1.5;
      // lc.force  = 10;
      // lc.intervalMode = true;    
      // agent->addOperator(new LiftUpOperator(lc));
      
      // like a bungee
      agent->addOperator(new PullToPointOperator(Pos(0,0,5),30,true, 
                                                 PullToPointOperator::Z,
                                                 0, 0.1, true));

      wiring->setParam("booster",0.01);
    }else{
      wiring->setParam("booster",0.07);
      controller->setParam("damping",0.0003);
      controller->setParam("epsC",0.1  );
    }

    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createFighters(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                           GlobalData& global, osg::Matrix pose){

    OdeAgent* f1=createHumanoid(odeHandle, osgHandle, global, TRANSM(0,0,1)*pose,
                                   "Fighter", true);
    if(!f1) {
      fprintf(stderr,"Only two fighters supported");
      return 0;
    }
    f1->addOperator(new BoxRingOperator(env.pitPosition+Pos(0,0,1.2),
                                        env.pitsize/2.0, 
                                        0.4, 200, false));

    OdeAgent* f2=createHumanoid(odeHandle, osgHandle, global, TRANSM(0,0,1)*pose,
                                   "Fighter", true);
    f2->addOperator(new BoxRingOperator(env.pitPosition+Pos(0,0,1.2),
                                        env.pitsize/2.0, 
                                        0.4, 200, false));
    // connect grippers
    Skeleton* h1 = dynamic_cast<Skeleton*>(f1->getRobot());
    Skeleton* h2 = dynamic_cast<Skeleton*>(f2->getRobot());
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
   
    return f2;    
  }

  OdeAgent* createArmband(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                          GlobalData& global, osg::Matrix pose){    
    // SPLIT CONTROLLED ARMBAND

    // find robot and do naming
    string name("SliderArmband");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);
    
    SliderWheelieConf mySliderWheelieConf  = SliderWheelie::getDefaultConf();
    mySliderWheelieConf.segmNumber         = 12;
    mySliderWheelieConf.segmLength         = .4;
    mySliderWheelieConf.segmDia            = .1; // thickness and width(*8) of segments
    mySliderWheelieConf.segmLength         = .6;
    mySliderWheelieConf.segmDia            = .2; // thickness and width(*8) of segments
    mySliderWheelieConf.jointLimitIn       = M_PI/3;
    mySliderWheelieConf.frictionGround     = 0.5;
    mySliderWheelieConf.motorPower         = 5;
    mySliderWheelieConf.motorDamp          = 0.05;
    mySliderWheelieConf.sliderLength       = 0.5;
    //    mySliderWheelieConf.texture            = "Images/whiteground.rgb";

    OdeRobot* robot = new SliderWheelie(odeHandle, /*osgHandle.changeColor(.5,.1,.2)*/
                                        osgHandle.changeColor("Renaissancegruen"),
                                        mySliderWheelieConf, name);
    robot->place(Pos(0,0,2.0)); 
    
    //controller = new Sos(1.0);
    AbstractController * controller = 
      new OneControllerPerChannel(new ControlGen(1,0.1,0.1),"OnePerJoint");
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); 
    OdeAgent* agent = new OdeAgent(global);
    // only the first controller is exported to guilogger and Co
    agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]);
    // think about configureable stuff since it clutters the console
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }


  OdeAgent* createLongVehicle(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                              GlobalData& global, osg::Matrix pose){    
    // find robot and do naming
    string name("LongVehicle");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    Nimm2Conf nimm2conf   = Nimm2::getDefaultConf();
    nimm2conf.size        = 0.7;
    nimm2conf.force       = 8/0.7/0.7/2.0;
    nimm2conf.speed       = 30;
    // nimm2conf.speed    = 15;
    nimm2conf.cigarMode   = true;
    nimm2conf.cigarLength = 3.0;
    nimm2conf.singleMotor = false;
    nimm2conf.boxMode     = true;
    nimm2conf.boxWidth    = 1.5;      
    nimm2conf.bumper      = true;
    
    OdeHandle odeHandleR = odeHandle;
    OdeRobot* robot = new Nimm2(odeHandleR, osgHandle.changeColor("robot2"), 
                                nimm2conf, name);
    //    robot->setColor(Color(.1,.1,.8));        
    robot->place(pose);
    SoxConf sc = Sox::getDefaultConf();
    sc.useExtendedModel=false;
    sc.useTeaching=true;
    Sox* sox = new Sox(sc);    
    sox->setParam("epsC",0.05);
    sox->setParam("epsA",0.05);
    sox->setParam("gamma",0.025);

    CrossMotorCoupling* controller = new CrossMotorCoupling( sox, sox, 0);    
    std::list<int> perm;
    perm.push_back(1);
    perm.push_back(0);
    CMC cmc = controller->getPermutationCMC(perm);
    controller->setCMC(cmc);


    AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());    
    OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot), 3.0);// noise 3 times stronger
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }    

  OdeAgent* createChain(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                        GlobalData& global, osg::Matrix pose){    
    // find robot and do naming
    string name("RobotChain");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));

    name = name + "_" + itos(num+1);

    RobotChainConf conf = RobotChain::getDefaultConf();
    conf.numRobots  = 5+num;
    conf.size       = 0.6;
    conf.distance   = 0.95;
    conf.force      = 2;
    conf.speed      = 50;
    conf.massFactor = 0.5;
    conf.wheelSlip  = 0.02;
    conf.color      = "robot3";        
    conf.useIR      = true;
    //        wiring = new One2OneWiring(new WhiteUniformNoise());
    AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());
      
    RobotChain* robotchain = new RobotChain(odeHandle, osgHandle.changeColorSet(num), 
                                            conf, name);     
    robotchain->place(pose);

    AbstractController* controller = 
      new OneControllerPerChannel(new ControlGenSoxExpand(0.001,0.01, 1.0, conf.numRobots*2),
                                  "OnePerJoint",1, robotchain->getIRSensorNum());

    OdeAgent* agent = new OdeAgent(global,PlotOption(NoPlot),1.0);
    agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]); 
    agent->init(controller, robotchain, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;    
  }    

  OdeAgent* createCaterPillar(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                              GlobalData& global, osg::Matrix pose){    
    // find robot and do naming
    string name("CaterPillar");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);
    
    CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
    myCaterPillarConf.segmNumber=3+num;
    myCaterPillarConf.jointLimit=M_PI/3;
    myCaterPillarConf.motorPower= 2;//5;
    myCaterPillarConf.frictionJoint=0.01;
    CaterPillar* robot=
      new CaterPillar ( odeHandle, osgHandle.changeColor(Color(.1f,.5,0.1)), 
                        myCaterPillarConf, name );
    robot->place(pose*TRANSM(-0,-11, 1)); 
      
    AbstractController* controller = new Sox();
    //  AbstractWiring* wiring = new One2OneWiring(new WhiteNormalNoise());    
    ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.01), 0.0);
    OdeAgent* agent = new OdeAgent(global, PlotOption(NoPlot)); 
    controller->setParam("Logarithmic",1);
    wiring->setParam("booster",.05);
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }    


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      // lower case (create robot) but last creation less than a second ago
      if(key >= 'a' && key <='z' && lastRobotCreation > (global.time-1)) { 
        printf("You have to wait one second between robot creations.\n");
        return false;
      }

      switch ( (char) key )
	{
        case 'k': // test
          env.widthground=15;
          env.create(odeHandle, osgHandle, global,true);
        case 'b':
          createSpherical(odeHandle, osgHandle, global, 
                          osg::Matrix::translate(1,5,2)); break;
        case 'B':
          removeRobot(global, "Spherical"); break;          
        case 'x':
          createHexapod(odeHandle, osgHandle, global, TRANSM(0,0,2)); break;
        case 'X':
          removeRobot(global, "Hexapod"); break; 
        case 'p' :
          createHexapod(odeHandle, osgHandle, global, TRANSM(5,-6,2),"HexaOnPile"); 
          break;
        case 'P':
          removeRobot(global, "HexaOnPile"); break; 
        case 's':
          createSnake(odeHandle, osgHandle, global, TRANSM(4,3,2)); break;
        case 'S':
          removeRobot(global, "Snake"); break; 
        case 'c':
          createChain(odeHandle, osgHandle, global, TRANSM(-2,-2,2)); break;
        case 'C':
          removeRobot(global, "RobotChain"); break;          
        // case 'd':
        //   createDog(odeHandle, osgHandle, global, TRANSM(0,0,2)); break;
        // case 'D':
        //   removeRobot(global, "Dog"); break;          
        case 'u':
          createHumanoid(odeHandle, osgHandle, global, TRANSM(0,0,2), "Humanoid"); break;
        case 'U':
          removeRobot(global, "Humanoid"); break;          
        case 'r':
          createFighters(odeHandle, osgHandle, global, TRANSM(env.pitPosition)); break;
        case 'R':
          removeRobot(global, "Fighter");
          removeRobot(global, "Fighter");
          global.removeExpiredObjects(global.time+1000);
          break;
        case 'a':
          createArmband(odeHandle, osgHandle, global, TRANSM(0,0,2)); break;
        case 'A':
          removeRobot(global, "SliderArmband"); break;          
        case 'l':
          createLongVehicle(odeHandle, osgHandle, global, TRANSM(0,0,2)); break;
        case 'L':
          removeRobot(global, "LongVehicle"); break;          
	default:
	  return false;
	  break;
	}
      if(key >= 'a' && key <='z') { // lower case -> created robot 
        lastRobotCreation = global.time;        
      }
      return true;
    }
    return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: b/B","add/remove Spherical");
    au.addKeyboardMouseBinding("Sim: x/X","add/remove Hexapod");
    au.addKeyboardMouseBinding("Sim: p/P","add/remove Hexapod on pile");
    au.addKeyboardMouseBinding("Sim: s/S","add/remove Snake");
    au.addKeyboardMouseBinding("Sim: c/C","add/remove Chain of robots");
    au.addKeyboardMouseBinding("Sim: u/U","add/remove Humanoid");
    au.addKeyboardMouseBinding("Sim: r/R","add/remove Humanoids in box ring");
    au.addKeyboardMouseBinding("Sim: a/A","add/remove SliderArmband");
    //    au.addKeyboardMouseBinding("Sim: d/D","add/remove Dog");
    au.addKeyboardMouseBinding("Sim: l/L","add/remove LongVehicle");
  }

  virtual void usage() const {
    printf("\t-empty\tstart the empty arena\n");
  };

  
};



int main (int argc, char **argv)
{ 
  empty=Simulation::contains(argv, argc, "-empty");
  

  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
