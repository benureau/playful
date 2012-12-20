/*
 *  CuriosityLoop.cpp
 *
 *
 */

#include "CuriosityLoop.h"
#include <selforg/controller_misc.h>

#define getrandom(max1) ((rand()%(int)((max1)))) // random integer between 0 and max-1

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

CuriosityLoop::CuriosityLoop(int inputSize, int motornumber, int sensornumber, int trial_length){
	cout << "Making curiosity loop  of input size " << inputSize << "\n";

	   predictorWeights.set(inputSize+1,inputSize+1); // initialized 0
	   predictorWeights=predictorWeights.map(random_minusone_to_one)*1.0;

	   predictorMask.set(inputSize+1,inputSize+1);

	   for(int i = 0; i < predictorMask.getM(); i++){
		for(int j = 0; j < predictorMask.getN(); j++){
			if(rand()/(RAND_MAX*1.0) < 0.01)
				predictorMask.val(i,j) = 1;
			else
				predictorMask.val(i,j) = 0;
		}
	   }

           prediction.set(inputSize+1, 1);

        pInput.set(inputSize+1,1); 
	pOutput.set(inputSize+1,1); 

	for(int i = 0; i < inputSize+1; i++){ 
	 		if(rand()/(RAND_MAX*1.0) < 0.2)
				pInput.val(i,0) = 1;
			else
				pInput.val(i,0) = 0;

			if(rand()/(RAND_MAX*1.0) < 0.2)
				pOutput.val(i,0) = 1;
			else
				pOutput.val(i,0) = 0;

	}

	uPredictorWeights = predictorWeights;  
 	uPredictorMask = predictorMask; 
        uPrediction = prediction;   
	
	uPInput = pInput; 
	uPOutput = pOutput; 
	
	//Unrestrict upInput now to it gets input from all motors. 
	for(int i = 0; i < inputSize+1; i++){ 
		if(i > 1+sensornumber){ 

			uPInput.val(i,0) = 1; //i.e. gets input from all motors. 	
			
		}
	}


	   actorWeights.set(motornumber,sensornumber); // initialized 0
	   actorWeights=actorWeights.map(random_minusone_to_one)*1.0;

	   offspringActorWeights.set(motornumber,sensornumber); 
	   offspringActorWeights = actorWeights; 

	//Each loop has a fitness
	fitness = 0;
	prediction_error = 0;
	prediction_error_time_average = 0;
	old_prediction_error_time_average = 0;

	addInspectableMatrix("PredInLoop",&prediction,false);
	//addInspectableMatrix("PredError",&parent_error,false);
	//addInspectableMatrix("UPredError",&offspring_error,false);
	parent_error.set(trial_length,1); 
	
	offspring_error.set(trial_length,1); 
	

};

void CuriosityLoop::replicateMutateActor(void){ 



	//Mutate the offspring actor. 
	for(int i = 0; i < offspringActorWeights.getM(); i++){
	   for(int j = 0; j <offspringActorWeights.getN(); j++){
		
		offspringActorWeights.val(i,j) = offspringActorWeights.val(i,j) + fRand(-0.3,0.3); 

	   }
	}
 


}; 

void CuriosityLoop::replicateUnrestrictPredictor(int chosen_act, int inputSize, int sensornumber){ 

	
	uPredictorWeights = predictorWeights;  
 	uPredictorMask = predictorMask; 
        uPrediction = prediction;   
	uPInput = pInput; 
	uPOutput = pOutput; 
	//Unrestrict upInput now to it gets input from all motors. 
	for(int i = 0; i < inputSize+1; i++){ 
		if(i > 1+sensornumber){ 

			uPInput.val(i,0) = 1; //i.e. gets input from all motors. 	
			
		}
	}

}; 

double CuriosityLoop::updatePrediction(const matrix::Matrix& smHist, const matrix::Matrix& s, const matrix::Matrix& m, int phase){

        matrix::Matrix sm = s.above(m);
        matrix::Matrix f;
        f.set(1,1);
        f.val(0,0) = 1;
        sm = sm.above(f);

	//1. Go through the predictions of this predictor determining the prediction errors at each dimension.
	matrix::Matrix error;
	error.set(smHist.getM(), 1);

	prediction_error = 0;
	for(int i = 0; i < prediction.getM(); i++){
	 if(pOutput.val(i,0) == 1){ 
	  error.val(i,0) = prediction.val(i,0) - sm.val(i,0);
	  prediction_error = prediction_error + pow(error.val(i,0),2);
	//  cout << error << "predictionError\n";
	 }
	 else{
	//  cout << "This dimension is not predicted, and does not count towards the error\n";
	  error.val(i,0) = 0;
	  //prediction_error = prediction_error + error.val(i,0);
	 }

	}
	parent_error.val(phase,0) = prediction_error; 

	//2. Change the weights by the delta rule.
	for(int i = 0; i < prediction.getM(); i++){

		for(int j = 0; j < predictorWeights.getN(); j++){

			predictorWeights.val(i,j) = predictorWeights.val(i,j) - 0.01*error.val(i,0)*sm.val(i,0);

			if(predictorWeights.val(i,j) > 5)
				predictorWeights.val(i,j)  = 5; 
			else if(predictorWeights.val(i,j)  < -5)
				predictorWeights.val(i,j) = -5; 
			
		}

	}
	prediction_error_time_average = 0.9999*prediction_error_time_average + (1-0.9999)*prediction_error;  

	//Update the fitness of this predictor based on the instantaneous reduction / increase in prediction error. 
	this->fitness = 0.1 + 100*(prediction_error_time_average - old_prediction_error_time_average);
        old_prediction_error_time_average = prediction_error_time_average; 

	//cout << fitness << " "; 
 	
	//Improve the method of determining this gradient later! 


	//UPDATE THE UNRESTRICTED PREDICTOR NOW AS WELL, ALWAYS... 
	//1. Go through the predictions of this UNRESTRICTED predictor determining the prediction errors at each dimension.
	matrix::Matrix uError;
	uError.set(smHist.getM(), 1);

	uPrediction_error = 0;
	for(int i = 0; i < uPrediction.getM(); i++){
	 if(uPOutput.val(i,0) == 1){ 
	  uError.val(i,0) = uPrediction.val(i,0) - sm.val(i,0);
	  uPrediction_error = uPrediction_error + pow(uError.val(i,0),2);
	//  cout << error << "predictionError\n";
	 }
	 else{
	//  cout << "This dimension is not predicted, and does not count towards the error\n";
	  uError.val(i,0) = 0;
	  //prediction_error = prediction_error + error.val(i,0);
	 }

	}
	//cout << "phase = " << phase << "\n";  
	offspring_error.val(phase,0) = uPrediction_error; 
	//2. Change the weights by the delta rule.
	for(int i = 0; i < uPrediction.getM(); i++){

		for(int j = 0; j < uPredictorWeights.getN(); j++){

			uPredictorWeights.val(i,j) = uPredictorWeights.val(i,j) - 0.01*uError.val(i,0)*sm.val(i,0);

			if(uPredictorWeights.val(i,j) > 5)
				uPredictorWeights.val(i,j)  = 5; 
			else if(uPredictorWeights.val(i,j)  < -5)
				uPredictorWeights.val(i,j) = -5; 
			
		}

	}
	 
	//************************UNRESTRICTED PREDICTOR CODE ****************************
	


	return this->fitness; 
};

void CuriosityLoop::makePrediction(const matrix::Matrix& s, const matrix::Matrix& m){

     matrix::Matrix sm = s.above(m);
     matrix::Matrix f;
     f.set(1,1);
     f.val(0,0) = 1;
     sm = sm.above(f);

     matrix::Matrix pwMod = predictorWeights;

     for(int i = 0; i < predictorWeights.getM(); i++){
	for(int j = 0; j < predictorWeights.getN(); j++){
	    if(pInput.val(i,0) == 1 && pOutput.val(j,0) == 1)
		   pwMod.val(i,j) = predictorWeights.val(i,j); 
	    else
		   pwMod.val(i,j) = 0; 
		 //   pwMod.val(i,j) = predictorWeights.val(i,j)*predictorMask.val(i,j);
	}
     }

     matrix::Matrix a = pwMod*sm; //Make prediction here.
     this->prediction = a; //The prediction is stored here.


	//************************UNRESTRICTED PREDICTOR CODE ****************************
//ALSO MAKE PREDICTIONS FOR THE UNRESTRICTED PREDICTOR

     matrix::Matrix uPwMod = uPredictorWeights;

     for(int i = 0; i < uPredictorWeights.getM(); i++){
	for(int j = 0; j < uPredictorWeights.getN(); j++){
	    if(uPInput.val(i,0) == 1 && uPOutput.val(j,0) == 1)
		   uPwMod.val(i,j) = uPredictorWeights.val(i,j); 
	    else
		   uPwMod.val(i,j) = 0; 
		 //   pwMod.val(i,j) = predictorWeights.val(i,j)*predictorMask.val(i,j);
	}
     }

     matrix::Matrix uA = uPwMod*sm; //Make prediction here.
     this->uPrediction = uA; //The prediction is stored here.

	//************************UNRESTRICTED PREDICTOR CODE ****************************

//	cout << predictorWeights.getM() << " " << predictorWeights.getN() << "= pw \n";
//	cout << sm.getM() << " " << sm.getN() << " = sm\n";
//	cout << pwMod.getM() << " " << pwMod.getN() << " = a\n";
//     for(int i = 0; i < prediction.getM(); i++){
//	for(int j = 0; j < pwMod.getN(); j++){
//	  cout << prediction.val(i,0) << " ";
//    }
//	 cout << " = precd\n";
//   }

};

matrix::Matrix CuriosityLoop::generateAction(const matrix::Matrix& s, int parentActing){ 

matrix::Matrix a; 

if(parentActing == 1){ 
	a = actorWeights*s; 
        a = a.map(tanh); 
 }
else{
	a = offspringActorWeights*s; 
        a = a.map(tanh); 

}

return a; 


}; 


CuriosityLoop::~CuriosityLoop(){

};

