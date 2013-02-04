/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                    DN c++ File      		                             *
 *                                                                           *
 *****************************************************************************/

#include "DN.h"
#include <selforg/controller_misc.h>
#include "CuriosityLoop.h"

#define TIMESCALE 0.1
#define CONNECTION_RATIO 0.1
#define INRATIO 0.2
#define OUTRATIO 0.2

using namespace std;
using namespace matrix;


#define getrandom(max1) ((rand()%(int)((max1)))) // random integer between 0 and max-1

double mean (const Matrix& array)
{
double sum = 0 ;
//cout << array.getM() << "\n"; 
for (int i = 0; i < array.getM(); i++)
sum = sum + array.val(i,0);
return sum/array.getM();
} // function calculating mean

double sd (const Matrix& array)
{
double sum = 0;
double STD_DEV = 0; // returning zero's

for (int i = 0; i < array.getM(); i++)
{
sum = sum + array.val(i,0);
STD_DEV = STD_DEV + pow(array.val(i,0), 2);
}
return sqrt ((STD_DEV/array.getM()) - (pow(sum/array.getM(),2)));
} // function calculating standard deviation



  /**
   * DN class constructor
  */
  DN::DN(int nbNeurons)
	:AbstractModel("ESN","0.1"), nbNeurons(nbNeurons)
  {

/*
        eps=0.01;
	addParameter("eps",&eps,0,1,"learning rate");
	//nothing
	addInspectableMatrix("OutputWeights",&outputWeights,false,"output weights");
  	addInspectableMatrix("ESNNeurons",&ESNNeurons,false,"internal state");
  	addInspectableMatrix("ESNWeights",&ESNWeights,false,"internal weights");
	addInspectableValue("error",&error,"Learning error");
        error = 0;


*/

  };


  void DN::init(unsigned int numNeurons, unsigned  int inputSize,   double unit_map, RandGen* randGen)
  {
	int trial_length = 10000; 
	chosen = 0; 
	//1. Construct a population of curiosity loops.
	for(int i = 0; i < numNeurons; i++){
		pop.push_back(new CuriosityLoop(inputSize, 12, 12, trial_length));
	}
	addInspectable(pop[0]);

	//2. Initialize the array to store the prediction errors of predictors.
	predictionErrors.set(pop.size(),1);
	parent_errors.set(trial_length,1); 	
	offspring_errors.set(trial_length,1); 
	

	addInspectableMatrix("fitness",&predictionErrors,false);
	addInspectableValue("chosen",&chosen,"ChosenActor");
	addInspectableMatrix("chosenActorParentError",&parent_errors,false);
	addInspectableMatrix("chosenActorOffspringError",&offspring_errors,false);
	
	fitnessF.open ("fitness.txt");



/*
	int nbInternalConnection;
	int nbInputConnectionPN;
	int nbOutputConnectionPN;

	nbInputs = inputDim;
	nbOutputs = outputDim;

	nbInternalConnection = nbNeurons*nbNeurons*CONNECTION_RATIO;
	nbInputConnectionPN = nbNeurons*INRATIO;
	nbOutputConnectionPN = nbNeurons*OUTRATIO;

	inputWeights.set(nbNeurons,inputDim);
	outputWeights.set(outputDim,nbNeurons);
	ESNNeurons.set(nbNeurons,1);
	ESNWeights.set(nbNeurons,nbNeurons);

	//inputWeights=inputWeights.map(random_minusone_to_one)*TIMESCALE;
	//outputWeights=outputWeights.map(random_minusone_to_one)*TIMESCALE;

	for(int count1 = 0; count1 < nbInputs; count1++)
	{
		for(int count2 = 0; count2 < nbInputConnectionPN; count2++)
		{
			int i = rand()%nbNeurons;
			inputWeights.val(i,count1) = random_minusone_to_one(0)*TIMESCALE;
		}
	}

	for(int count1 = 0; count1 < nbOutputs; count1++)
	{
		for(int count = 0; count < nbOutputConnectionPN; count++)
		{
			int i = rand()%nbNeurons;
			outputWeights.val(count1,i) = random_minusone_to_one(0)*TIMESCALE;
		}
	}

	for(int count = 0; count < nbInternalConnection; count++)
	{
		int i = rand()%nbNeurons;
		int j = rand()%nbNeurons;
		ESNWeights.val(i,j) = random_minusone_to_one(0)*TIMESCALE;
	}

*/



};

void DN::updateLoops (const Matrix& sensors, const Matrix& motors)
{
//  void DN::updateLoops (const sensor* sensors, const motor* motors)


};

void DN::updatePredictions (const Matrix& smHist, const Matrix& sensors, const Matrix& motors, int phase)
{
//Use the delta-rule to modify the predictions based on the actual new sensor values and the new motor values.

	//Go through each of the predictions, calculating the error.
	for(int i = 0; i < pop.size(); i++){
		//cout << "Update prediction " << i << "\n";
		predictionErrors.val(i,0) = pop[i]->updatePrediction(smHist, sensors, motors, phase);
	}
	//cout << "\n"; 
};

 void DN::makePredictions (const Matrix& sensors, const Matrix& motors)
{
//   Matrix sm = sensors.above(motors);

   //1.The predictor in each loop should make a prediction.

	for(int i = 0; i < pop.size(); i++){
		//cout << "Making prediction " << i << "\n";
		pop[i]->makePrediction(sensors, motors);
	}

};

  const int  DN::actionSelection (const matrix::Matrix& s){ 
  //This is really LOOP SELECTION on the basis of LOOP FITNESS. 
	Matrix a; 
	//1. Do Softmax selection on the basis of Loop level fitness. 	
	double totalFitness = 0; 
	for(int i = 0; i < pop.size(); i++){ 
		totalFitness = totalFitness + pop[i]->fitness; 
	}
	double randThresh = (rand()/(RAND_MAX*1.0))*totalFitness; 	
	//cout << randThresh << "\n"; 	
	double counter = 0;
	int chosen = -1;  
	for(int i = 0; i < pop.size(); i++){ 
		counter = counter + pop[i]->fitness; 
		if(counter >= randThresh){ 
			chosen = i; 
//			cout << "non random choice of action \n"; 
			break; 
		}
		if(chosen != -1)
		 break; 
	}
	if(chosen == -1){ 
		chosen = getrandom(pop.size()); 
		cout << "Random loop selected/n"; 
        	//Chosen action = chosen. So, this actor will be permitted to act. 
	}

	
  	this->chosen = chosen; 
	return chosen; 
};

 const matrix::Matrix  DN::executeAction (int chosen, const Matrix& sensors, int parentActing){ 

	Matrix a; 

	//Run the chosen loop's actor. 
	a = pop[chosen]->generateAction(sensors, parentActing);

	return a; 

}; 

void DN::replicateMutate(int chosen_actor){ 

        pop[chosen_actor]->replicateMutateActor(); 

}; 


void DN::replicateUnrestrictPredictor(int chosen_actor, int in, int sens ){ 

        pop[chosen_actor]->replicateUnrestrictPredictor(chosen_actor, in, sens); 

}; 

void DN::savePredictorWeights(int chosen_actor){ 

	pop[chosen_actor]->savePredictorWeights(); 

}; 

void DN::getErrorsFromChosenActor(int chosen_actor){

	parent_errors = pop[chosen_actor]->parent_error; 
	offspring_errors = pop[chosen_actor]->offspring_error; 


};  	

void DN::determineActorFitness(int chosen_actor){ 

  ofstream myfile;
  myfile.open ("restrictedErrors.txt");
 
//1.    Restricted predictor errors over time. 
	for(int i = 0; i < parent_errors.getM(); i++){
	//	cout << parent_errors.val(i,0) << " "; 
		myfile << parent_errors.val(i,0) << " \n";
  	}
	//cout << "\n"; 
  	myfile.close();

  ofstream myfile2;
  myfile2.open ("unrestrictedErrors.txt");

//2.    Unrestricted predictor errors over time. 
	for(int i = 0; i < offspring_errors.getM(); i++){
	//	cout << offspring_errors.val(i,0) << " "; 
		myfile2 << offspring_errors.val(i,0) << "\n ";
  	}
	//cout << "\n"; 
  	myfile2.close();

//3. Take first half of the data and get mean and s.d. 
	double meanRparent = mean(parent_errors.rows(0,parent_errors.getM()/2)); 
	double meanRoffspring = mean(parent_errors.rows(parent_errors.getM()/2+1, parent_errors.getM()-1)); 
	double meanUparent = mean(offspring_errors.rows(0,offspring_errors.getM()/2)); 
	double meanUoffspring = mean(offspring_errors.rows(offspring_errors.getM()/2+1, offspring_errors.getM()-1)); 

	double sdRparent = sd(parent_errors.rows(0,parent_errors.getM()/2)); 
	double sdRoffspring = sd(parent_errors.rows(parent_errors.getM()/2+1, parent_errors.getM()-1)); 
	double sdUparent = sd(offspring_errors.rows(0,offspring_errors.getM()/2)); 
	double sdUoffspring = sd(offspring_errors.rows(offspring_errors.getM()/2+1, offspring_errors.getM()-1)); 

	
//4. Fitness is a function of the above values. 
	//Reward a high LMS error ratio between restricted and unrestricted model, i.e. adding motor helps. 
	//Reward an unrestricted model that reduces the sd of the prediction errors.   

	//Typical Granger Causality type fitness function. Rewarding unrestricted models that reduce the variance of the prediction errors. 
	pop[chosen_actor]->parentActorFitness =  meanRparent/meanUparent;//+ (sdRparent/sdUparent); 
	//if(pop[chosen_actor]->parentActorFitness  < 0)
	//	pop[chosen_actor]->parentActorFitness  = 0; 
	pop[chosen_actor]->offspringActorFitness = meanRoffspring/meanUoffspring;// + (sdRoffspring/sdUoffspring);
	//if(pop[chosen_actor]->offspringActorFitness  < 0)	
	//	pop[chosen_actor]->offspringActorFitness  = 0; 
 
	cout << "parent = " << pop[chosen_actor]->parentActorFitness << " offspring = " << pop[chosen_actor]->offspringActorFitness << "\n"; 
	fitnessF <<  pop[chosen_actor]->parentActorFitness << " " << pop[chosen_actor]->offspringActorFitness << "\n"; 
	fitnessF.flush(); 
	//cout << parent_errors.rows(0,10) << " EMND OF ROWS\n"; ; 
	//cout << parent_errors.columns(0,1) << " EMND OF COLUMNS\n"; ; 
}; 

void DN::SHCstep(int chosen_actor){ 

	if(pop[chosen_actor]->parentActorFitness < pop[chosen_actor]->offspringActorFitness){
	   pop[chosen_actor]->overwriteParentActor(); 
	   cout << "Loop " << chosen_actor << " offspring to parent\n"; 
	} 
			
};  

//****************************OLD ESN FUNCTIONS*********************************

  const Matrix DN::process (const Matrix& input)
  {
	ESNNeurons = (inputWeights*input+ESNWeights*ESNNeurons).map(tanh);
	return outputWeights* ESNNeurons;
  };


  const Matrix DN::learn (const Matrix& input, const Matrix& nom_output, double learnRateFactor)
  {
	const Matrix& output = process(input);
	const Matrix& delta = nom_output - output;
	error = delta.norm_sqr();
	outputWeights += (delta * (ESNNeurons^T)) * (learnRateFactor * eps);
	return output;

  }

  void DN::damp(double damping)//Damp is Dumb
  {

  }

  unsigned int DN::getInputDim() const
  {
	return inputWeights.getM();
  }

  unsigned int DN::getOutputDim() const
  {
	return outputWeights.getN();
  }

  bool DN::store(FILE* f) const{
  // save matrix values

    inputWeights.store(f);
    outputWeights.store(f);
    outputWeights.store(f);
    ESNWeights.store(f);
    Configurable::print(f,0);
    return true;
  }

  /* loads the ESN values from a given file. */
  bool DN::restore(FILE* f){
    // save matrix values
    inputWeights.restore(f);
    outputWeights.restore(f);
    outputWeights.restore(f);
    ESNWeights.restore(f);
    Configurable::parse(f);
    return true;
  }

