#include "OsiCplexSolverInterface.hpp"
#include <CoinMpsIO.hpp>
#include <cplex.h>
#include <algorithm>

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
  // //const MSKenv_t env = OsiMskSolverInterface::getEnvironmentPtr();
  // const MSKtask_t task = OsiMskSolverInterface::getMutableLpPtr();
  // MSKrescodee res;
  // MSKconetypee conetype;
  // double conepar;
  // int nummem;
  // int * submem;
  // // get conic constraint
  // res = MSK_getcone(task, index, &conetype, &conepar, &nummem, submem);
  // if (res!=MSK_RES_OK) {
  //   std::cerr << "Mosek status " << res << std::endl;
  //   throw std::exception();
  // }
  // numMembers = nummem;
  // // who will free members?
  // members = new int[numMembers];
  // std::copy(submem, submem+numMembers, members);
  // if (conetype==MSK_CT_QUAD) {
  //   type = OSI_QUAD;
  // }
  // else if (conetype==MSK_CT_RQUAD) {
  //   type = OSI_RQUAD;
  // }
}

// add conic constraint in lorentz cone form
void OsiCplexSolverInterface::addConicConstraint(OsiLorentzConeType type,
						 int numMembers,
					       const int * members) {
  // MSKrescodee res;
  // MSKconetypee conetype;
  // const MSKtask_t task = OsiMskSolverInterface::getLpPtr();
  // double conepar = 0.0;
  // if (type==OSI_QUAD) {
  //   conetype = MSK_CT_QUAD;
  // }
  // else {
  //   conetype = MSK_CT_RQUAD;
  // }
  // res = MSK_appendcone(task, conetype, conepar, numMembers, members);
  // if (res!=MSK_RES_OK) {
  //   std::cerr << "Mosek status " << res << std::endl;
  //   throw std::exception();
  // }
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
  // MSKrescodee res;
  // MSKtask_t task = OsiMskSolverInterface::getLpPtr();
  // int num = 1;
  // int * subset;
  // subset = new int[1];
  // subset[0] = index;
  // res = MSK_removecones(task, num, subset);
  // delete[] subset;
}

void OsiCplexSolverInterface::modifyConicConstraint(int index,
						    OsiLorentzConeType type,
						    int numMembers,
						    const int * members) {
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}


int OsiCplexSolverInterface::getNumCones() const {
  // MSKrescodee res;
  // MSKtask_t task = OsiMskSolverInterface::getMutableLpPtr();
  // int num;
  // res = MSK_getnumcone(task, &num);
  // if (res!=MSK_RES_OK) {
  //   std::cerr << "Mosek status " << res << std::endl;
  //   throw std::exception();
  // }
  // return num;
}

int OsiCplexSolverInterface::getConeSize(int i) const {
  // const MSKtask_t task = OsiMskSolverInterface::getMutableLpPtr();
  // MSKrescodee res;
  // MSKconetypee conetype;
  // double conepar;
  // int nummem;
  // // get conic constraint information
  // res = MSK_getconeinfo(task, i, &conetype, &conepar, &nummem);
  // if (res!=MSK_RES_OK) {
  //   std::cerr << "Mosek status " << res << std::endl;
  //   throw std::exception();
  // }
  // return nummem;
}

OsiConeType OsiCplexSolverInterface::getConeType(int i) const {
  // int num_cones = getNumCones();
  // if (i>=num_cones) {
  //   std::cerr << __PRETTY_FUNCTION__ << "Cone " << i << " does not exist!"
  //             << std::endl;
  //   throw std::exception();
  // }
  // return OSI_LORENTZ;
}

OsiLorentzConeType OsiCplexSolverInterface::getLorentzConeType(int i) const {
  // const MSKtask_t task = OsiMskSolverInterface::getMutableLpPtr();
  // MSKrescodee res;
  // MSKconetypee conetype;
  // double conepar;
  // int nummem;
  // OsiLorentzConeType type;
  // // get conic constraint information
  // res = MSK_getconeinfo(task, i, &conetype, &conepar, &nummem);
  // if (res!=MSK_RES_OK) {
  //   std::cerr << "Mosek status " << res << std::endl;
  //   throw std::exception();
  // }
  // if (conetype==MSK_CT_QUAD) {
  //   type = OSI_QUAD;
  // }
  // else if (conetype==MSK_CT_RQUAD) {
  //   type = OSI_RQUAD;
  // }
  // else {
  //   std::cerr << __PRETTY_FUNCTION__ << " Unknown mosek cone type!"
  //             << std::endl;
  //   throw std::exception();
  // }
  // return type;
}

// fills array of cone sizes.
void OsiCplexSolverInterface::getConeSize(int * size) const {
  // const MSKtask_t task = OsiMskSolverInterface::getMutableLpPtr();
  // MSKrescodee res;
  // MSKconetypee conetype;
  // double conepar;
  // int nummem;
  // int num_cones = getNumCones();
  // for (int i=0; i<num_cones; ++i) {
  //   // get conic constraint information
  //   res = MSK_getconeinfo(task, i, &conetype, &conepar, &nummem);
  //   if (res!=MSK_RES_OK) {
  //     std::cerr << "Mosek status " << res << std::endl;
  //     throw std::exception();
  //   }
  //   size[i] = nummem;
  // }
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
  // we need to clone task and env, I think
  // OsiMskSolverInterface::clone will be enough.
  // OsiMosekSolverInterface * new_solver = new OsiMosekSolverInterface(*this);
  // return new_solver;
}

OsiCplexSolverInterface::~OsiCplexSolverInterface() {
  // free cplex pointer? or ~OsiCpxSolverInterface handles that.
}

int OsiCplexSolverInterface::readMps(const char * filename,
				     const char * extension) {
  // todo(aykut) this reads linear part and conic part
  OsiConicSolverInterface::readMps(filename, extension);
  // what will happen to loadProblem?
}
