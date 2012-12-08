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

CuriosityLoop::CuriosityLoop(int inputSize){
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


	   actorWeights.set(inputSize+1,inputSize); // initialized 0
	   actorWeights=actorWeights.map(random_minusone_to_one)*1.0;

	//Each loop has a fitness
	fitness = 0;
	prediction_error = 0;
	addInspectableMatrix("PredInLoop",&prediction,false);

};

double CuriosityLoop::updatePrediction(const matrix::Matrix& smHist, const matrix::Matrix& s, const matrix::Matrix& m){

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
	 if(prediction.val(i,0) != 0){
	  error.val(i,0) = prediction.val(i,0) - sm.val(i,0);
	  prediction_error = prediction_error + error.val(i,0);
	//  cout << error << "predictionError\n";
	 }
	 else{
	//  cout << "This dimension is not predicted, and does not count towards the error\n";
	  error.val(i,0) = 0;
	  prediction_error = prediction_error + error.val(i,0);
	 }
	}

	//2. Change the weights by the delta rule.
	for(int i = 0; i < prediction.getM(); i++){

		for(int j = 0; j < predictorWeights.getN(); j++){

			predictorWeights.val(i,j) = predictorWeights.val(i,j) - 0.05*error.val(i,0)*sm.val(i,0);

		}

	}
	return prediction_error;
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

	    pwMod.val(i,j) = predictorWeights.val(i,j)*predictorMask.val(i,j);

	}
     }

     matrix::Matrix a = pwMod*sm; //Make prediction here.
     this->prediction = a; //The prediction is stored here.

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

CuriosityLoop::~CuriosityLoop(){

};

