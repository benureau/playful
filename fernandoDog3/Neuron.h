/*
 *  Neuron.h
 *  IzhikevichSimulator
 *
 *  Created by Chrisantha Fernando on 03/03/2009.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */


#import <iostream>
#import <vector>
#import <fstream>
#import <math.h>
#import <stddef.h>
#import  <time.h>
#import <string.h>
#import <stdio.h>
#import <sstream>


using namespace std; 

class Neuron{
		
	public: 
			
	Neuron(int, int, int,int, int, int, int, int, int);  
	~Neuron();  
	
	int type;
	int iden; 

	vector <Neuron*> connectsTo; 	
	vector <int> connectsToWTAState; 	

	vector <Neuron*> connectsFrom; 
	vector <int> connectsFromWTAState; 
	
	double a; 
	double b; 
	double c; 
	double d; 
	double v;
	
	double cI; //Voltage component due to internal stimulation 
	double cE; //Voltage component due to external stimulation 
	
	double u;
	int fired;
	
	double thalamicInput; 
	double z; //A leaky integrator is filled by spikes of this neuron, tc = 100ms = 0.1s.  
	
	//Variables needed to define spatial connectivity 
	int inLayer; 
	int xpos; 
	int ypos; 
	
	
	int toReplicate; 
	Neuron* toReplicateTo; 
	
	int phenotype; 
	int inputSetNeuron; 
	
	int activationCountdown; 

	
	
	private: 
	
	
		
}; 
