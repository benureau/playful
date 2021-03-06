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


  void DN::init(unsigned int numNeurons, unsigned  int inputSize, double unit_map, RandGen* randGen)
  {

	//1. Construct a population of curiosity loops.
	for(int i = 0; i < numNeurons; i++){
		pop.push_back(new CuriosityLoop(inputSize));
	}
	addInspectable(pop[0]);

	//2. Initialize the array to store the prediction errors of predictors.
	predictionErrors.set(pop.size(),1);

	addInspectableMatrix("PredErrors",&predictionErrors,false);



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


} ;

void DN::updatePredictions (const Matrix& smHist, const Matrix& sensors, const Matrix& motors)
{
//Use the delta-rule to modify the predictions based on the actual new sensor values and the new motor values.

	//Go through each of the predictions, calculating the error.
	for(int i = 0; i < pop.size(); i++){
		//cout << "Update prediction " << i << "\n";
		predictionErrors.val(i,0) = pop[i]->updatePrediction(smHist, sensors, motors);
	}

} ;

 void DN::makePredictions (const Matrix& sensors, const Matrix& motors)
{
//   Matrix sm = sensors.above(motors);

   //1.The predictor in each loop should make a prediction.

	for(int i = 0; i < pop.size(); i++){
		//cout << "Making prediction " << i << "\n";
		pop[i]->makePrediction(sensors, motors);
	}

};


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

