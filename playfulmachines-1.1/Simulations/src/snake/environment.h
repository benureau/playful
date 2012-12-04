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
 *                                                                 *
 ***************************************************************************/

#ifndef __ENVIRONMENT_H
#define __ENVIRONMENT_H

#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>
#include <ode_robots/seesaw.h>

using namespace lpzrobots;
using namespace std;


class Env : public Configurable {
public:
  enum EnvType { None, Normal, Octa, Pit, Uterus, Stacked };

  Env() : Configurable("Environment","1.0") {
    type         = None;
    widthground  = 45.85;
    heightInner  = 2;
    pitsize      = 1;
    height       = 0.8;
    uterussize   = 1;
    roughness    = .2;
    
    
    numSpheres  = 0; 
    numBoxes    = 0;   
    numCapsules = 0;
    numSeeSaws  = 0;

    addParameter("pitheight", &heightInner, 0, 10, "height of circular pit");
    addParameter("height",    &height,      0, 10, "height of square walls");
    addParameter("pitsize",   &pitsize,     0, 10, "size of the pit (diameter)");
    addParameter("roughness", &roughness,   0, 10, 
                 "roughness of ground and walls (friction parameters)");
  }

  EnvType type;
  std::list<AbstractGround*> playgrounds;
      
  // playground parameter
  double widthground;
  double heightground;
  double heightInner;
  double pitsize;
  double height;
  double uterussize;
  double roughness;

  // obstacles
  int numSpheres; 
  int numBoxes;   
  int numCapsules;
  int numSeeSaws;

  OdeHandle odeHandle;
  OsgHandle osgHandle;
  GlobalData* global ;
  
  
  /** creates the Environment   
   */
  void create(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
              GlobalData& global, bool recreate=false){
    this->odeHandle=odeHandle;
    this->osgHandle=osgHandle;
    this->global=&global;
    if(recreate && !playgrounds.empty()){
      FOREACH(std::list<AbstractGround*>, playgrounds, p){
        removeElement(global.obstacles, *p);
        delete (*p);
      }
      playgrounds.clear();
    }
    AbstractGround* playground;
    switch (type){
    case Octa:
      {
        playground = new OctaPlayground(odeHandle, osgHandle, 
                                        osg::Vec3(pitsize, 0.2, heightInner), 12, false);
        playground->setTexture("Images/really_white.rgb");
        Color c = osgHandle.getColor("Monaco");
        c.alpha()=0.15;
        playground->setColor(c);
        playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
      }
    case Normal:
      {
        playground = new Playground(odeHandle, osgHandle,
                                    osg::Vec3(widthground, 0.208, height)); 
        //     playground->setTexture("Images/really_white.rgb");
        //        playground->setGroundTexture("Images/yellow_velour.rgb");
        playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3)); // was: wall.rgb 
        playground->setPosition(osg::Vec3(0,0,.0));
        Substance substance(roughness, 0.0, 10, 0.95);
        playground->setGroundSubstance(substance);
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
        break;
      }
    case Pit:
      {
        // we stack two playgrounds in each other. 
        // The outer one is hard and the inner one is softer
        Substance soft = Substance::getRubber(5);
        soft.roughness = roughness;
        OdeHandle myHandle = odeHandle;
        myHandle.substance = soft;
        playground = new Playground(myHandle, osgHandle, 
                                    osg::Vec3(pitsize, 0.2, height),
                                    1, true); 
        playground->setTexture("Images/really_white.rgb");
        playground->setGroundSubstance(soft);
        
        Color c = osgHandle.getColor("Monaco");
        c.alpha()=0.15;
        playground->setColor(c);
        playground->setPosition(osg::Vec3(0,0,0.1)); // playground positionieren und generieren
        global.obstacles.push_back(playground);
        playgrounds.push_back(playground);
        break;
      }        
    case Uterus:
      {
        // we stack two playgrounds in each other. 
        // The outer one is hard (and invisible) and the inner one is soft
        int anzgrounds=2;
        // this is the utterus imitation: high slip, medium roughness, high elasticity, soft
        Substance uterus(0.2/*roughness*/, 0.1 /*slip*/, 
                         .5 /*hardness*/, 0.95 /*elasticity*/);
        double thickness = 0.4;
        for (int i=0; i< anzgrounds; i++){
          OdeHandle myHandle = odeHandle;
          if(i==0){
            myHandle.substance = uterus;
          }else{
            myHandle.substance.toMetal(.2);            
          }          
          Playground* playground = new Playground(myHandle, osgHandle, 
                                                  osg::Vec3(uterussize+2*thickness*i, 
                                                            i==0 ? thickness : .5, height),
                                                  1, i==0); 
          playground->setTexture("Images/dusty.rgb");
          if(i==0){ // set ground also to the soft substance
            playground->setGroundSubstance(uterus);
          }
          playground->setColor(Color(0.5,0.1,0.1,i==0? .2 : 0)); // outer ground is not visible (alpha=0)
          playground->setPosition(osg::Vec3(0,0,i==0? thickness : 0 )); // playground positionieren und generieren
          global.obstacles.push_back(playground);
          playgrounds.push_back(playground);        
        }        

        break;
      }
    case Stacked:
      {
        int anzgrounds=12;
        for (int i=0; i< anzgrounds; i++){
          playground = new Playground(odeHandle, osgHandle, osg::Vec3(8+4*i, .42, .95+0.15*i), 1, i==(anzgrounds-1));
          OdeHandle myhandle = odeHandle;
          //      myhandle.substance.toFoam(10);
          // playground = new Playground(myhandle, osgHandle, osg::Vec3(/*base length=*/50.5,/*wall = */.1, /*height=*/1));
          playground->setPosition(osg::Vec3(0,0,0.2)); // playground positionieren und generieren
          
          global.obstacles.push_back(playground);
          playgrounds.push_back(playground);
        }
        break;
      }
    default:
      break;
    }
  }    

  void placeObstacles(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                      GlobalData& global){

    
    for(int i=0; i<numSeeSaws; i++){
      Seesaw* seesaw = new Seesaw(odeHandle, osgHandle);
      seesaw->setColor("wall"); 
      seesaw->setPose(ROTM(M_PI/2.0,0,0,1)*TRANSM(4, -i,.0));
      global.obstacles.push_back(seesaw);
    }

    for(int i=0; i<numSpheres; i++){
      PassiveSphere* s = 
	new PassiveSphere(odeHandle, 
			  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0)); 
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);    
    }

    for(int i=0; i<numBoxes; i++){
      PassiveBox* b = 
	new PassiveBox(odeHandle, 
			  osgHandle, osg::Vec3(0.2+i*0.1,0.2+i*0.1,0.2+i*0.1));
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      b->setPosition(Pos(i*0.5-5, i*0.5, 1.0)); 
      global.obstacles.push_back(b);    
    }

    for(int i=0; i<numCapsules; i++){
      PassiveCapsule* c = 
	new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-1, -i, 1.0)); 
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);    
    }

  }

  virtual void notifyOnChange(const paramkey& key){
    create(odeHandle,osgHandle,*global,true);
  }

};


#endif

