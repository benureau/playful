/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "ctrloutline.h"
using namespace matrix;
using namespace std;

CtrlOutline::CtrlOutline()
  : AbstractController("CtrlOutline", "0.7"){
  t=0;

  addParameterDef("creativity",&creativity,0.5, "creativity term (0: disabled) ");

  addParameterDef("TLE",   &TLE, true, "whether to use the time loop error");
  addParameterDef("Logarithmic", &loga, true, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1,  "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.02, "learning rate of the model");

  addInspectableMatrix("A", &A, false, "model matrix");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("h", &h, false, "controller bias");
  addInspectableMatrix("b", &b, false, "model bias");

  addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");

};

CtrlOutline::~CtrlOutline(){
}


void CtrlOutline::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);

  A.toId(); // set a to identity matrix;
  //   A*=0.1;
  C.toId(); // set a to identity matrix;
  C*=.4;    // with 0.4 on the diagonals
  
  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);   
    xs_buffer[k].set(number_sensors,1);   
    y_buffer[k].set(number_motors,1);   

  }
}

matrix::Matrix CtrlOutline::getA(){
  return A;
}

void CtrlOutline::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix CtrlOutline::getC(){
  return C;
}

void CtrlOutline::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix CtrlOutline::geth(){
  return h;
}

void CtrlOutline::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void CtrlOutline::step(const sensor* x_, int number_sensors, 
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  learn();

  // update step counter
  t++;
};


/// performs one step without learning. Calulates motor commands from sensor inputs.
void CtrlOutline::stepNoLearning(const sensor* x_, int number_sensors, 
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);

  x.set(number_sensors,1,x_); // store sensor values
  x_buffer[t%buffersize] = x;
    
  // calculate controller values based on current input values (smoothed)
  //Matrix y = (C*(x) + h).map(g);
  const Matrix& y =   (C*(x + v_avg*creativity) + h).map(g);
  
  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor* 
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};
  
      

/// learn values h,C,A
void CtrlOutline::learn(){

  Matrix C_update(number_motors,number_sensors);
  Matrix h_update(number_motors,1);

  // the effective x/y is (actual-1) element of buffer
  const Matrix& x     = x_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& y     = y_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& x_fut = x_buffer[t% buffersize]; // future sensor (with respect to x,y)
  
  const Matrix& z    = (C * (x + v_avg * creativity) + h);

  const Matrix& g_prime = z.map(g_s);
  const Matrix& g_prime_inv = g_prime.map(one_over);

  const Matrix& xsi = x_fut  - (A* y + b);

  A += (xsi * (y^T) * epsA + (A *  -0.003) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);
  b += (xsi *  epsA        + (b *  -0.001) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);

  if (TLE) {
    //************ TLE (homeokinetic) learning **********
    // Matrix  eta = (A^-1) * xsi; 
    const Matrix&  eta = A.pseudoInverse() * xsi; 
    const Matrix& zeta  = (eta & g_prime_inv).mapP(1.0, clip);
    // C.multMT = (C*(C^T))
    const Matrix& mue   = (((C.multMT().pluslambdaI())^-1)*zeta);

    const Matrix& v =  ( (C^T) * mue ).mapP(1.0, clip);
    v_avg += ( v  - v_avg ) *.1; 

    double EE = 1.0; 
    if(loga){
      EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
    }

    C_update =  (( mue * (v^T) )
                 + (( mue & y & zeta) *(-2) * (x^T))) * (EE *epsC); 
    h_update =  ( mue & y & zeta) * (-2 *EE * epsC); 

  }else{ // not TLE    
    // ********** homeostatic learning: normal delta rule  ***********      
    const Matrix& eta = (A^T) * xsi; 
    C_update = (eta & g_prime)* (x^T) * epsC;
    h_update = (eta & g_prime) * epsC;
  }

  // apply updates to h,C
  h += h_update.mapP(.1, clip);
  C += C_update.mapP(.1, clip);
};

  
/** stores the controller values to a given file. */
bool CtrlOutline::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  h.store(f);
  A.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool CtrlOutline::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  A.restore(f);
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

