/*
 *  Neuron.cpp
 *  IzhikevichSimulator
 *
 *  Created by Chrisantha Fernando on 03/03/2009.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "Neuron.h"

#define getrandom(max1) ((rand()%(int)((max1)))) // random integer between 0 and max-1

Neuron::Neuron(int CT, int idI, int layerI, int xposI, int yposI, int typeI, int numLayers, int layerSize, int layerSizeY){ 

	type = typeI; //1 = excitatory, -1 = inhibitory. 
	iden = idI; 	
	//INITIALIZE IZEKEVIACH NEURON PARAMETERS
	
	if(type == 1) {//excitatory  
		a = 0.02; 
		b = 0.2; 
		c = -65+15*pow((rand()/(RAND_MAX*1.0)),2); 
		d = 8-6*pow((rand()/(RAND_MAX*1.0)),2); 
	} 
	if(type == -1){//inhibitory
		a = 0.02 + 0.08*(rand()/(RAND_MAX*1.0)); 
		b = 0.25-0.05*(rand()/(RAND_MAX*1.0)); 
		c = -65; 
		d = 2; 
		
	}
	
	v = -65.0; 
	cI = 0; //This variable measures how much of the input into the neuron (in L1) is due to internal layer current. 
	cE = 0; //This variable measures how much of the input into the neuron (in L1) is due to external current from L0.  
	u = b * v; 
	//cout << u << "\n"; 
	fired = 0; 
	thalamicInput = 0; 
	xpos = xposI; 
	ypos = yposI; 
	z = 0; 
	inLayer = layerI; 
	toReplicate = 0; 
	
	phenotype = getrandom(2); 
	//cout << "phenotype = " << phenotype << "\n"; 
	inputSetNeuron = 0; 
	activationCountdown = CT; 
						
	
	
	
};


Neuron::~Neuron(){ 
	
}; 

