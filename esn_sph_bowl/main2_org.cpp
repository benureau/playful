/***************************************************************************
 *   Copyright (C) 2011 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>

// used robot
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivecapsule.h>

// used controller
#include <selforg/sox.h>
#include <selforg/semox.h>

// fetch all the stuff of lpzrobots and std into scope
using namespace lpzrobots;
using namespace std;

Sphererobot3Masses* sphere ;
//const double height = 6.5;
const double height = 2;

enum Env {ThreeBump, SingleBasin, ElipticBasin};
Env env = SingleBasin;
bool track=false;
const char* envnames[3] = {"ThreePot", "SingleBasin","ElipticBasin"};

class ThisSim : public Simulation {
public:


  ThisSim(){
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
  }

  void addRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int i){
    Color col;
    
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    conf.diameter=1.0;
    conf.pendularrange= 0.30; // 0.15;
    conf.motorpowerfactor  = 150;     
    conf.spheremass = 1;
    conf.motorsensor=false;
    
    conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));
    

    //SphererobotArms* sphere = new SphererobotArms ( odeHandle, conf);
    switch(i){
    case 0:
      col.r()=0;
      col.g()=1;
      col.b()=0.1;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 1", 0.4);
      sphere->place ( osg::Matrix::translate(9.5 , 0 , height+1 ));
      break;
    case 1:
      col.r()=1;
      col.g()=0.2;
      col.b()=0;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 2", 0.4);
      sphere->place ( osg::Matrix::translate( 2 , -2 , height+1 ));
      break;
    case 3:
      col.r()=0;
      col.g()=0;
      col.b()=1;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere" + 
					string(envnames[env]), 0.4);
      sphere->place ( osg::Matrix::translate( 0 , 0 , .5 ));
      break;
    default:
    case 2:
      col.r()=0;
      col.g()=1;
      col.b()=0.4;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere 3", 0.4);
      sphere->place ( osg::Matrix::translate( double(rand())/RAND_MAX*10 , 0 , height+1 ));
      break;
    }
  
    AbstractController *controller = new Sox(1.2,false);    
    controller->setParam("epsC", 0.2);
    controller->setParam("epsA", 0.2);        
    if(env==ElipticBasin){
      controller->setParam("epsC", 0.3);
      controller->setParam("epsA", 0.3);        
    }
    controller->setParam("Logarithmic", 0);
    controller->setParam("sense", 0.5);
    // 1297536669
    
    // SeMoXConf cc = SeMoX::getDefaultConf();    
    // cc.modelExt=true;
    // AbstractController* controller = new SeMoX(cc);  
    
    // controller->setParam("epsC", 0.05);
    // controller->setParam("epsA", 0.1);        
    // controller->setParam("rootE", 0);
    // controller->setParam("steps", 1);
    // controller->setParam("s4avg", 1);
    // controller->setParam("dampModel", 0.9e-5);
    // controller->setParam("discountS", 0.05);
    
    
  
    //    AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );   
  AbstractWiring* wiring = new SelectiveOne2OneWiring(new WhiteUniformNoise(), 
                                                      new select_from_to(0,2), 
                                                      AbstractWiring::Robot);
  
    OdeAgent* agent;
    if(i==0 || i==3 ){
      agent = new OdeAgent (global);
    }
    else
      agent = new OdeAgent (global, PlotOption(NoPlot));

    agent->init ( controller , sphere , wiring );
    if(track)
      agent->setTrackOptions(TrackRobot(true,true,true,false,"",2));

    global.agents.push_back ( agent );
    global.configs.push_back ( controller );  
    global.configs.push_back ( sphere);  
  }

  void removeRobot(GlobalData& global){
    if(!global.agents.empty()){
      OdeAgentList::iterator i =  global.agents.end()-1;
      delete (*i)->getRobot();
      delete (*i)->getController(); 
      delete (*i);
      global.agents.erase(i);    
    }
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-16.4509, 15.6927, 12.5683),  Pos(-133.688, -26.4496, 0));
    setCameraMode(Static);
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",2);
    //  global.odeConfig.setParam("gravity", 0);

    Playground* playground;
    if(env==ElipticBasin){
      playground = new Playground(odeHandle, osgHandle, 
				  osg::Vec3(20, 0.2, height+1.f), 2);
    }else{
      playground = new Playground(odeHandle, osgHandle, 
				  osg::Vec3(20, 0.2, height+0.3f), 1);
    }
    playground->setColor(Color(.1,0.7,.1));
    playground->setTexture("");
    playground->setPosition(osg::Vec3(0,0,0.01f)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    int numpassive=0;
    switch(env){
    case ThreeBump:
      {
        TerrainGround* terrainground = 
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0,1.0)),
                            "terrains/macrospheresLMH_64.ppm","terrains/macrospheresTex_256.ppm", 
                            20, 20, height, OSGHeightField::LowMidHigh);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground);
        addRobot(odeHandle, osgHandle, global, 0);
        addRobot(odeHandle, osgHandle, global, 1);
        numpassive=4;
      }
      break;
    case SingleBasin:
      // at Radius 3.92 height difference of 0.5 and at 6.2 height difference of 1 
      // ./start -single -track -f 2 -r 1297536669

    case ElipticBasin:
      // ./start -eliptic -f 5 -track -r 1297628680
      {
        TerrainGround* terrainground = 
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
			    //                        "terrains/dip128_flat.ppm","terrains/dip128_flat_texture.ppm", 
                            "terrains/dip128.ppm","terrains/dip128_texture.ppm", 
                            20, env == SingleBasin ? 20 : 40, height, OSGHeightField::Red);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground);
        addRobot(odeHandle, osgHandle, global, 3);
      }
      break;
    }
        

    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and 
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere 
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i< numpassive; i+=1){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5,0.1);
      s1->setPosition(osg::Vec3(-8+2*i,-2,height+0.5));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }
  
  }

  virtual void usage() const {
    printf("\t-single\tSingle basin (default)\n");
    printf("\t-three\tThree basins\n");
    printf("\t-eliptic\tAn elictic basin\n");
    printf("\t-track\tenable tracking (trajectory is written to file)\n");
    //    printf("\t-noslip\tdisable slipping\n");
  };


};


int main (int argc, char **argv)
{ 
  if(Simulation::contains(argv,argc,"-eliptic")>0)
    env=ElipticBasin;
  if(Simulation::contains(argv,argc,"-single")>0)
    env=SingleBasin;
  if(Simulation::contains(argv,argc,"-three")>0)
    env=ThreeBump;
  track = (Simulation::contains(argv,argc,"-track")>0);
  
  ThisSim sim;  
  return sim.run(argc, argv) ? 0 : 1;

}
 




