#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include "sos_avggrad.h"
#include <selforg/one2onewiring.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

using namespace std;
using namespace matrix;

bool stop=0;
double noise=.00;
double realtimefactor=1;

/** This robot emulates different systems based on the mode
    parameter. 
    This is usually some kind of short-circuit with inertia, 
     additional inputs/outputs ...
    */
class MyRobot : public AbstractRobot, public Inspectable {
public:
  MyRobot(const string& name, int dimension, AbstractController* controller)
    : AbstractRobot(name, "$Id: main.cpp,v 1.3 2009/01/23 09:26:39 martius Exp $"),
      controller(controller) {
    t=0;
    buffersize=20;
    motornumber  = dimension;
    sensornumber = dimension;

    x.set(sensornumber,1);
    y.set(motornumber,1);

    // initial conditions
    x.val(0,0) = 0.01;
    x.val(1,0) = 0.00001;
    
    //    addParameterDef("initS", &initS, false, "initial S (true) or C (false)");
    addParameterDef("rotation", &rotation, 90, "initial rotation angle");
    addParameterDef("factor",   &factor,   1, "factor u of rotation matrix");
    //    addParameterDef("magic",    &magic,    0, "initialize with magic circle");
    addParameterDef("staterand",    &staterandom,    false, "randomize state");
    addParameterDef("random",     &random,    false, "randomize controller matrix");
    
    addInspectableValue("phi",      &phi,   "rotation angle");
    addInspectableValue("omega",    &omega, "magic rotation angle");
    
    // parameter that defines the speed of the simulation
    addParameter("realtimefactor",&realtimefactor); // actually a global parameter 
    // noise level
    addParameter("noise", &noise);               // actually a global parameter 
  }

  ~MyRobot(){
  }

  // robot interface

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber){
    assert(sensornumber >= this->sensornumber);
    x.convertToBuffer(sensors,sensornumber);
    //    double* x_cur = x_buffer[(t-1 + buffersize)%buffersize];
    //    memcpy(sensors, x_cur, sizeof(sensor) * this->sensornumber);
    return this->sensornumber;
  }

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber){
    assert(motornumber >= this->motornumber);
    y.set(motors);
    normal();
    if(staterandom){
      x.toMap(random_minusone_to_one);
      staterandom=0;
    }
    t++;    
  } 

  /** returns number of sensors */
  virtual int getSensorNumber(){ return sensornumber; }

  /** returns number of motors */
  virtual int getMotorNumber() { return motornumber; }

  virtual Position getPosition()     const {return Position(0,0,0);}
  virtual Position getSpeed()        const {return Position(0,0,0);}
  virtual Position getAngularSpeed() const {return Position(0,0,0);}
  virtual matrix::Matrix getOrientation() const {
    matrix::Matrix m(3,3);
    m.toId();
    return m; 
  };

  // system with damping, correlation of subsequent channels (cyclic),
  // and delay (the delay is actually done by getSensors()
  void normal(){
    Matrix S;
    if(dynamic_cast<SosAvgGrad*>(controller)){      
      S = dynamic_cast<SosAvgGrad*>(controller)->getS();
    }
    x = y + S*x;

  }

  virtual void notifyOnChange(const paramkey& key){
    if(key=="rotation"){
      initSO2(getParam(key),factor);     
    }
    if(key=="magic"){
      initMagic(getParam(key),factor);     
    }
    if(key=="random"){
      initRandom(factor);     
    }
  }


  void initSO2(double rot, double factor){
    phi=rot*M_PI/180;
    Matrix M(2,2);
    M.val(0,0) = cos(phi)*factor;
    M.val(0,1) = sin(phi)*factor;
    M.val(1,0) = -sin(phi)*factor;
    M.val(1,1) = cos(phi)*factor;
    if(dynamic_cast<SosAvgGrad*>(controller)){
      if(initS)
        dynamic_cast<SosAvgGrad*>(controller)->setS(M);
      else
        dynamic_cast<SosAvgGrad*>(controller)->setC(M);
    }      
    omega=0;
  }

  void initRandom(double factor){
    Matrix M(2,2);			       
    M = M.map(random_minusone_to_one)*factor;
    if(dynamic_cast<SosAvgGrad*>(controller)){
      if(initS)
        dynamic_cast<SosAvgGrad*>(controller)->setS(M);
      else
        dynamic_cast<SosAvgGrad*>(controller)->setC(M);
    }
    random=false;
  }

  void initMagic(double omega_deg, double factor){
    Matrix M(2,2);
    omega=omega_deg * M_PI/180;
    M.val(0,0) = 1.;
    M.val(0,1) = -sin(omega);
    M.val(1,0) = sin(omega);
    M.val(1,1) = 1-sqr(sin(omega));
    M*=factor;
    if(dynamic_cast<SosAvgGrad*>(controller)){
      if(initS)
        dynamic_cast<SosAvgGrad*>(controller)->setS(M);
      else
        dynamic_cast<SosAvgGrad*>(controller)->setC(M);
    }      
    phi=0;
  }
  
private:
  int motornumber;
  int sensornumber;
  unsigned int buffersize;

  Matrix x;
  Matrix y;

  int rotation;
  paramval factor;
  bool initS;
  paramval magic;
  bool staterandom;
  bool random;

  double phi;  // acutally set angle
  double omega; // acutally set magic angle

  AbstractController* controller;

  int t;

}; 


void printRobot(MyRobot* robot){
  char line[81];
  memset(line,'_', sizeof(char)*80);
  line[80]=0;
  sensor s[20];
  int len = robot->getSensors(s,20);
  for(int i=0; i<len; i++){
    double x = s[i];
    x=clip(x,-1.0,1.0);
    line[int((x+1.0)/2.0*80.0)%80]='0'+ i;
  }
  
  printf("\033[1G%s",line);
  fflush(stdout);
  
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

void autoScanPhi(MyRobot* robot,long int t){
  const int interval = 2000;
  if(t%interval==0){
    robot->setParam("rotation",t/interval);
    printf("Rotation: %f\n",robot->getParam("rotation"));
  }
}

void autoChangePhi(MyRobot* robot,long int t){
  const int scale = 10000;
  switch (t){
  case 1*scale:
    robot->setParam("rotation",5);
    break;
  case 2*scale:
    robot->setParam("rotation",30);
    break;
  case 3*scale:
    robot->setParam("rotation",60);
    break;
  case 4*scale:
    robot->setParam("rotation",90);
    break;
  case 5*scale:
    robot->setParam("rotation",120);
    break;
  case 6*scale:
    robot->setParam("rotation",0);
    break;
  case 7*scale:
    robot->setParam("magic",30);
   break;
  case 8*scale:
    robot->setParam("magic",90);
    break;
  case 9*scale:
    robot->setParam("rotation",0);
    break;
  default:
    break;
  }
}

int main(int argc, char** argv){
  list<PlotOption> plotoptions;
  char modestr[1024];
  int dim = 2;

  double phi= 0; 
  double factor = 1.05;
  double eps = 0.01;
  double  magic = 0;
  long int steps = 0;
  bool autochange = false;
  bool autoscan   = false;

  plotoptions.push_back(PlotOption(GuiLogger,1));
  int index; 
  index = contains(argv,argc,"-f");
  if(index >0 && argc>index) {
    plotoptions.push_back(PlotOption(File,atoi(argv[index])));
  }
  index = contains(argv,argc,"-magic");
  if(index >0 && argc>index)
    magic=atof(argv[index]);
  index = contains(argv,argc,"-noise");
  if(index >0 && argc>index)
    noise=atof(argv[index]);

  // index = contains(argv,argc,"-d");
  // if(index >0 && argc>index)
  //   dim=atoi(argv[index]); 
  index = contains(argv,argc,"-a");
  if(index >0 && argc>index)
    phi=atof(argv[index]);
  index = contains(argv,argc,"-fac");
  if(index >0 && argc>index)
    factor=atof(argv[index]);
  index = contains(argv,argc,"-steps");
  if(index >0 && argc>index)
    steps=atoi(argv[index]);
  index = contains(argv,argc,"-eps");
  if(index >0 && argc>index)
    eps=atof(argv[index]);  
  autochange = contains(argv,argc,"-autophi") != 0;
  autoscan   = contains(argv,argc,"-scanphi") != 0;
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-f N] [-eps EPS] [-a PHI] [-fac FACTOR] [-magic PHI] [-steps STEPS] [-noise SIZE] [-initS] [-autophi] \n",argv[0]);
    printf("\t-f N\twrite logfile with interval N\n");
    //    printf("\t-d DIM\t dimensionality, default 2\n");
    printf("\t-eps eps\t learning rate, default 10\n");
    printf("\t-a phi\t rotation angle in degree, default 0\n");
    printf("\t-fac factor\t size of rotation matix, default 1\n");
    printf("\t-magic PHI\t magic initialization\n");
    printf("\t-steps steps\t number of simulation steps, default 0 (infinite)\n");
    printf("\t-initS\t the initialization is done in S (default is on C)\n");
    printf("\t-autophi\t automatic change of phi and magic (default is on C)\n");
    printf("\t-scanphi\t automatic run through phi (default is on C)\n");
    printf("\t-h\tdisplay this help\n");
    exit(0);
  }

  sprintf(modestr,"avg_n%i-%03i_e%i-%03i_",(int)noise,(int)(1000*noise), (int)eps, (int)(1000*eps));

  GlobalData globaldata;
  MyRobot* robot;
  Agent* agent;
  initializeConsole();
  
  
  AbstractController* controller = new SosAvgGrad();
  controller->setParam("epsC", eps);  
  controller->setParam("epsA", 0);
   
  robot         = new MyRobot(string("ShortCircuit_") + string(modestr), dim, controller);
  agent         = new Agent(plotoptions);
  AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),
                                             AbstractWiring::Controller 
                                             | AbstractWiring::Noise);  
  //AbstractWiring* wiring = new One2OneWiring(new SineWhiteNoise(0.1,1));  
  // AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise());  
  agent->init(controller, robot, wiring);
  // if you like, you can keep track of the robot with the following line. 
  //  this assumes that you robot returns its position, speed and orientation. 
  // agent->setTrackOptions(TrackRobot(true,false,false, false,"systemtest"));
  
  globaldata.agents.push_back(agent);
  globaldata.configs.push_back(robot);
  globaldata.configs.push_back(controller);
 
  robot->setParam("initS",contains(argv,argc,"-initS")!=0); 
  robot->setParam("factor",factor);
  robot->setParam("rotation", phi);
  if(magic!=0)
    robot->setParam("magic", magic);  

  showParams(globaldata.configs);
  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  printf("The output of the program is more fun then useful ;-).\n");
  printf(" The number are the sensors and the position there value.\n");
  printf(" You probably want to use the guilogger with e.g.: -g 10\n");

  cmd_handler_init();
  long int t=0;
  while(!stop && (steps==0  || t < steps)){

    agent->step(noise,t);
    
    if(autochange) autoChangePhi(robot,t);
    if(autoscan) autoScanPhi(robot,t);
    if(control_c_pressed()){      
      if(!handleConsole(globaldata)){
        stop=1;
      }
      cmd_end_input();
    }
    int drawinterval = 10000;
    if(realtimefactor!=0){
      drawinterval = max(1,int(6*realtimefactor));
    }
    if(t%drawinterval==0){
      printRobot(robot);
      usleep(60000);
    }    
    t++;
  };
  delete agent;
  closeConsole();
  fprintf(stderr,"terminating\n");
  // should clean up but what costs the world
  return 0;
}
