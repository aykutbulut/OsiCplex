#include "OsiCplexSolverInterface.hpp"
#include <CoinMpsIO.hpp>
#include <cplex.h>
//#include <algorithm>

// default constructor
OsiCplexSolverInterface::OsiCplexSolverInterface(): OsiCpxSolverInterface() {
}

// copy constructor
OsiCplexSolverInterface::OsiCplexSolverInterface(const OsiCplexSolverInterface & other):
  OsiCpxSolverInterface(other) {
}

// copy assignment operator
OsiCplexSolverInterface & OsiCplexSolverInterface::operator=(const OsiCplexSolverInterface & rhs) {
  // copy rhs to this
  OsiCpxSolverInterface::operator=(rhs);
  return *this;
}

// get conic constraints
void OsiCplexSolverInterface::getConicConstraint(int index,
						 OsiLorentzConeType & type,
                                                 int & numMembers,
                                                 int *& members) const {
  int ok = 0;
  // cplex status, 0 for success
  int status;
  // number of linear part nonzeros
  int linnz;
  int linsurplus = 0;
  // number of quadratic part nonzeros
  int qnz;
  numMembers = 0;
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getMutableEnvironmentPtr();
  // detect array lengths
  status = CPXgetqconstr(env, lp, &linnz, &qnz, NULL, NULL,
                         NULL, NULL, 0, &linsurplus,
                         NULL, NULL, NULL, 0, &numMembers, index);
  if (status==0) {
    // means number of variables in cone is 0.
    return;
  }
  else if (status == CPXERR_NEGATIVE_SURPLUS) {
    // linear or quadratic cone size was not enough
    if (numMembers==0) {
      // cone size was enough, means no member in cone
      return;
    }
    else {
      // cone size is greater than 0
      numMembers = -numMembers;
    }
  }
  // allocate memory for quadratic part
  int * qrow = new int[numMembers];
  int * qcol = new int[numMembers];
  double * qval = new double[numMembers];
  int qsurplus;
  // call get constraints again with updated cone size
  status = CPXgetqconstr(env, lp, &linnz, &qnz, NULL, NULL,
                         NULL, NULL, 0, &linsurplus,
                         qrow, qcol, qval, numMembers, &qsurplus, index);
  if (status!=0 && status!=CPXERR_NEGATIVE_SURPLUS) {
    std::cerr << "Cplex status error!" << std::endl;
    throw std::exception();
  }
  if (qsurplus<0) {
    std::cerr << "This should not happen. Cplex cheated!" << std::endl;
    throw std::exception();
  }
  members = qcol;
  delete[] qrow;
  if (qval[0]==-1.0 && qval[1]==1.0) {
    type = OSI_QUAD;
  }
  else {
    std::cerr << "This part is not implemented yet!" << std::endl;
    throw std::exception();
  }
  delete[] qval;
}

// add conic constraint in lorentz cone form
void OsiCplexSolverInterface::addConicConstraint(OsiLorentzConeType type,
						 int numMembers,
					       const int * members) {
  if (type==OSI_RQUAD) {
    std::cerr << "Rotated Cones are not implemented yet!" << std::endl;
    throw std::exception();
  }
  int status;
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  double * qval = new double[numMembers];
  qval[0] = -1.0;
  std::fill_n(qval+1, numMembers-1, 1.0);
  status = CPXaddqconstr (env, lp, 0, numMembers,
                          0.0, 'L', NULL, NULL,
                          members, members, qval, NULL);
  if (status != 0) {
    std::cerr << "Cplex function is not successful." << std::endl;
    throw std::exception();
  }
}

// add conic constraint in |Ax-b| <= dx-h form
void OsiCplexSolverInterface::addConicConstraint(CoinPackedMatrix const * A,
						 CoinPackedVector const * b,
						 CoinPackedVector const * d,
						 double h) {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}


void OsiCplexSolverInterface::removeConicConstraint(int index) {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

void OsiCplexSolverInterface::modifyConicConstraint(int index,
						    OsiLorentzConeType type,
						    int numMembers,
						    const int * members) {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}


int OsiCplexSolverInterface::getNumCones() const {

  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getMutableEnvironmentPtr();
  int n = CPXgetnumqconstrs(env, lp);
  return n;
}

int OsiCplexSolverInterface::getConeSize(int i) const {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

OsiConeType OsiCplexSolverInterface::getConeType(int i) const {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

OsiLorentzConeType OsiCplexSolverInterface::getLorentzConeType(int i) const {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

// fills array of cone sizes.
void OsiCplexSolverInterface::getConeSize(int * size) const {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

// fills array of cone types.
void OsiCplexSolverInterface::getConeType(OsiConeType * type) const {
  int num_cones = getNumCones();
  for (int i=0; i<num_cones; ++i) {
    type[i] = getConeType(i);
  }
}

void OsiCplexSolverInterface::getConeType(OsiLorentzConeType * type) const {
  int num_cones = getNumCones();
  for (int i=0; i<num_cones; ++i) {
    type[i] = getLorentzConeType(i);
  }
}

OsiConicSolverInterface * OsiCplexSolverInterface::clone(bool copyData) const {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

OsiCplexSolverInterface::~OsiCplexSolverInterface() {
}

int OsiCplexSolverInterface::readMps(const char * filename,
				     const char * extension) {
  OsiConicSolverInterface::readMps(filename, extension);
}

void OsiCplexSolverInterface::initialSolve() {
  //todo(aykut) I am not sure what switchToLp() does.
  switchToLP();
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  double objoffset;
  double primalobjlimit;
  double dualobjlimit;
  if (messageHandler()->logLevel() == 0)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 0);
  else if (messageHandler()->logLevel() == 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 1);
  else if (messageHandler()->logLevel() > 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 2);
  getDblParam(OsiObjOffset, objoffset);
  getDblParam(OsiPrimalObjectiveLimit, primalobjlimit);
  getDblParam(OsiDualObjectiveLimit, dualobjlimit);
  if (getObjSense() == +1) {
    if (primalobjlimit < COIN_DBL_MAX)
      CPXsetdblparam(env, CPX_PARAM_OBJLLIM, primalobjlimit + objoffset);
    if (dualobjlimit > -COIN_DBL_MAX)
      CPXsetdblparam(env, CPX_PARAM_OBJULIM, dualobjlimit + objoffset);
  }
  else {
    if (primalobjlimit > -COIN_DBL_MAX)
      CPXsetdblparam(env, CPX_PARAM_OBJULIM, primalobjlimit + objoffset);
    if (dualobjlimit < COIN_DBL_MAX)
      CPXsetdblparam(env, CPX_PARAM_OBJLLIM, dualobjlimit + objoffset);
  }
  int status = CPXhybbaropt(env, lp, CPX_ALG_NONE);
  if (status!=0) {
    std::cerr << "Cplex did not return 0 status." << std::endl;
    throw std::exception();
  }
}

//-----------------------------------------------------------------------------
// resolve is same as initialSolve, no warm start capability.
void OsiCplexSolverInterface::resolve() {
  //todo(aykut) I am not sure what switchToLp() does.
  switchToLP();
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  if (messageHandler()->logLevel() == 0)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 0);
  else if (messageHandler()->logLevel() == 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 1);
  else if (messageHandler()->logLevel() > 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 2);
  int status = CPXhybbaropt(env, lp, CPX_ALG_NONE);
  if (status!=0) {
    std::cerr << "Cplex did not return 0 status." << std::endl;
    throw std::exception();
  }
}
