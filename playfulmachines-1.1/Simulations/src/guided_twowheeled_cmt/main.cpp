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
 *   $Log: main.cpp,v $
 *   Revision 1.5  2010/09/20 09:53:52  martius
 *   added quantitative plot Fig 4
 *
 *   Revision 1.4  2010/09/17 14:45:21  martius
 *   started to implement reviewers comments
 *
 *   Revision 1.3  2010/05/28 16:53:31  martius
 *   changed simulation
 *
 *   Revision 1.1  2010/02/24 14:08:30  martius
 *   initial version
 *
 *   Revision 1.1  2009/11/20 15:24:22  martius
 *   copied together from diss
 *
 *   Revision 1.2  2009/03/19 18:51:40  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/03/18 17:46:14  martius
 *   Teaching stuff
 *
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/tmpprimitive.h>

#include <selforg/semox.h>
#include <selforg/sox.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/stl_adds.h>

#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;

int useTeaching = 0;
bool useStraight = false;
bool useStaffel = false;
double arenaFactor=5;
double gamma_teach = 0;
bool track=false;

class ThisSim : public Simulation, public Inspectable {
public:

  //  SeMoX* controller;
  Sox* controller;
  OdeRobot* vehicle;
  Matrix teaching;
  AbstractGround* playground;
  
  double teacherror; 
  double cumteacherror; 
  long int cumteachersteps;

  ThisSim(){
    controller=0;
    vehicle=0;
    setCaption("Simulator by Martius et al");
    setTitle("Guided TwoWheeled");
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-4.91981, 6.44648, 2.29529),  Pos(-142.657, -13.0139, 0));
    setCameraMode(Follow);
    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 1);

    // use Playground as boundary:
//    playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
//     // playground->setColor(Color(0,0,0,0.8)); 
//     playground->setGroundColor(Color(2,2,2,1)); 
//     playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
//     global.obstacles.push_back(playground);    

    if(!useTeaching){
      playground = new ComplexPlayground(odeHandle, osgHandle, 
						     "cluttered.fig", arenaFactor, 0.02);
      playground->setTexture("really_white.rgb");
      playground->setColor(Color(236.0/255.0,233.0/255.0,24.0/255.0)); 
      playground->setPosition(osg::Vec3(0,-1*arenaFactor,0.0));      
      //      playground->setPosition(osg::Vec3(0,-,0.05));      
      global.obstacles.push_back(playground);
    }

    teacherror=0;
    cumteacherror = 0;
    cumteachersteps = 0;
    teaching.set(2,1);
    addInspectableValue("teacherror", &teacherror, "difference between teaching and output");

    for(int i=0; i<0; i++){ //20
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5,10);
      s->setPosition(osg::Vec3(-4+2*(i/5),-4+2*(i%5),2));
      global.obstacles.push_back(s);    
    }
    
    Nimm2Conf c = Nimm2::getDefaultConf();    
    c.sphereWheels=false;
    c.sphereWheels=false;

    vehicle = new Nimm2(odeHandle, osgHandle, c, std::string("Nimm2_") + 
			(useStraight ? "straight_" : (useTeaching? "teaching_" : "")) + 
			 std::itos(gamma_teach*10000));
    vehicle->place(Pos(0,0,0.1));    

    // create pointer to controller
    // SeMoXConf cc = SeMoX::getDefaultConf();    
    // cc.cInit=1.19;
    // cc.modelExt=false;
    // cc.someInternalParams=false;        
    // controller = new SeMoX(cc); 
 
    SoxConf cc = Sox::getDefaultConf();    
    cc.initFeedbackStrength=1.1;
    cc.useExtendedModel=false;
    cc.someInternalParams=false;
    cc.useTeaching = true;
    controller = new Sox(cc);  
    if(!useTeaching){
      controller->setParam("epsC", 0.01);
      controller->setParam("epsA", 0.01);
    }else{
      controller->setParam("epsC", 0.1);
      controller->setParam("epsA", 0.1);
    }

    //    controller->setParam("gamma_teach", gamma_teach);
    controller->setParam("gamma", gamma_teach);

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(global);
    agent->addInspectable(this);
    agent->init(controller, vehicle, wiring);
    if(track) agent->setTrackOptions(TrackRobot(true,true,false, true, "cl", 10));
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    
    this->getHUDSM()->setColor(osgHandle.getColor("hud"));
    this->getHUDSM()->setFontsize(16);    
    this->getHUDSM()->addMeasure(gamma_teach,"Gamma",ID,1);
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(controller){
      gamma_teach = controller->getParam("gamma");
    }    
    if(control){      
      Matrix last = controller->getLastMotorValues();	

      // calc mismatch between teaching and real output (last timestep)
      teacherror = sqrt((teaching - last).norm_sqr()); 
      cumteacherror+=teacherror;
      cumteachersteps++;

      if(useTeaching){
        if(useStaffel){
          if(globalData.time<60){ // normal
            return;
          }else if(globalData.time<120){ // sin
            teaching.toMapP(sin(globalData.time*1/20.0*M_PI)*.85, constant);
          }else{     // rectangular
            teaching.toMapP(sign(sin(globalData.time*1/20.0*M_PI))*0.65, constant);
          }
        }else{
          teaching.toMapP(sin(globalData.time*1/20.0*M_PI)*.85, constant);
        }
      }else if(useStraight){	
	teaching.val(0,0) = last.val(1,0);
	teaching.val(1,0) = last.val(0,0);	
      }

      if(useTeaching==1 || useTeaching==2 || useStraight){
	controller->setMotorTeaching(teaching);
      }
    }
    
  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& global, 
                       int key, bool down)
  {
    char file[256];
    char txt[256];
    FILE* f;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'g' :           
          // useTeaching = !useTeaching;
          // printf("Guidance %s\n", useTeaching ? "enabled" : "disabled");
          if(controller->getParam("gamma") <= 0 ) {
            controller->setParam("gamma", 0.02);
            sprintf(txt,"Guidance enabled  (gamma = %f)\n", controller->getParam("gamma"));
          } else {
            controller->setParam("gamma", 0.0);
            sprintf(txt,"Guidance disabled (gamma = %f)\n", controller->getParam("gamma"));
          }
          printf("%s\n",txt);
          global.removeExpiredObjects(global.time+15);
          global.addTmpObject(new TmpDisplayItem(new OSGText(std::string(txt),14), 
                                                 TRANSM(20,30,0), "hud", 0.8),15);
	  return true;
          break;
	case 'c' : 
	  sprintf(file,"contour.dat");
	  f= fopen(file,"w");
	  if(!f) std::cerr << "cannot open file: " << file << std::endl;
	  else {
	    if(playground) playground->printContours(f);
	    fclose(f);
	    std::cout << "saved contour to "<<  file << std::endl; 
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


  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: c","save contour");

  }

  virtual void end(GlobalData& globalData){
    FILE* f = fopen("teacherrors.dat","a");
    if(f){
      fprintf(f,"%f\t %f\n", gamma_teach, cumteacherror/cumteachersteps);
      fclose(f);
    }else{
      fprintf(stderr, "cannot write to file teacherrors.dat\n");
    }
  }

  virtual void usage() const {
    printf("\t-gamma gamma\tset teaching strength (direct motor teaching)\n");
    printf("\t-staffel\tchange teaching signal automatically\n");
    printf("\t-straight gamma\tenable cross motor teaching with strength gamma\n");
    printf("\t-arenasize size\tsets the size of the arena, (default: 50)\n");
    printf("\t-track\tenable tracking (write logfile)\n");
  };


};


int main (int argc, char **argv)
{ 
  int index = Simulation::contains(argv,argc,"-gamma");
  if(index >0 && argc>index){
    gamma_teach=atof(argv[index]); 
    useTeaching = 1;  
  }
  useStaffel = (Simulation::contains(argv,argc,"-staffel")!=0);
  track = (Simulation::contains(argv,argc,"-track")!=0);
  index = Simulation::contains(argv,argc,"-straight");
  if(index >0 && argc>index){
    gamma_teach=atof(argv[index]); 
    printf("GAMMA %f\n", gamma_teach);
    //    if(teacher>0)
    useStraight = true;  
  }
  index = Simulation::contains(argv,argc,"-arenasize");
  if(index >0 && argc>index){
    arenaFactor=atof(argv[index])/10; 
  }

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
