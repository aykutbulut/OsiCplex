#include "OsiCplexSolverInterface.hpp"
#include <CoinMpsIO.hpp>
#include <cplex.h>
//#include <algorithm>

// todo(aykut)
// error check routine. Copied from OsiCpxSolverInterface.
// I am not sure whether this is a license violation. This
// may be removed/replaced in future.
// I would not experince any problem if this function was not static
// in OsiCpxSolverInterface.cpp. Since it is, I have to copy it here
// and create redundancy.
static inline void
checkCPXerror(int err, std::string cpxfuncname, std::string osimethod) {
  if(err != 0) {
    char s[100];
    sprintf( s, "%s returned error %d", cpxfuncname.c_str(), err );
#ifdef DEBUG
    std::cerr << "ERROR: " << s << " (" << osimethod << " in OsiCpxSolverInterface)" << std::endl;
#endif
    throw CoinError(s, osimethod.c_str(), "OsiCplexSolverInterface");
  }
}


// default constructor
OsiCplexSolverInterface::OsiCplexSolverInterface(): OsiCpxSolverInterface() {
  CPXENVptr env = getEnvironmentPtr();
  // set parameter number of threads to 1, CPXPARAM_Threads
  CPXINT num_threads = 1;
  CPXINT status;
  // Name prior to V12.6.0 is CPX_PARAM_THREADS, we assume V12.6.0 or later
  status = CPXsetintparam(env, CPXPARAM_Threads, num_threads);
  if (status) {
    std::cerr << "Cplex status error!" << std::endl;
    throw std::exception();
  }
}

// copy constructor
OsiCplexSolverInterface::OsiCplexSolverInterface(const OsiCplexSolverInterface & other):
  OsiCpxSolverInterface(other) {
  CPXENVptr env = getEnvironmentPtr();
  // set parameter number of threads to 1, CPXPARAM_Threads
  CPXINT num_threads = 1;
  CPXINT status;
  // Name prior to V12.6.0 is CPX_PARAM_THREADS, we assume V12.6.0 or later
  status = CPXsetintparam(env, CPXPARAM_Threads, num_threads);
  if (status) {
    std::cerr << "Cplex status error!" << std::endl;
    throw std::exception();
  }
}

// copy assignment operator
OsiCplexSolverInterface & OsiCplexSolverInterface::operator=(const OsiCplexSolverInterface & rhs) {
  // copy rhs to this
  OsiCpxSolverInterface::operator=(rhs);
  CPXENVptr env = getEnvironmentPtr();
  // set parameter number of threads to 1, CPXPARAM_Threads
  CPXINT num_threads = 1;
  CPXINT status;
  // Name prior to V12.6.0 is CPX_PARAM_THREADS, we assume V12.6.0 or later
  status = CPXsetintparam(env, CPXPARAM_Threads, num_threads);
  if (status) {
    std::cerr << "Cplex status error!" << std::endl;
    throw std::exception();
  }
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
  int status;
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  if (type==OSI_QUAD) {
    double * qval = new double[numMembers];
    qval[0] = -1.0;
    std::fill_n(qval+1, numMembers-1, 1.0);
    status = CPXaddqconstr (env, lp, 0, numMembers,
                            0.0, 'L', NULL, NULL,
                            members, members, qval, NULL);
    delete[] qval;
    checkCPXerror(status, std::string("CPXaddqconstr"),
                  std::string("addConicConstraint"));
    // leading variable is nonnegative
    double bound[] = {0.0};
    char lu[] = {'L'};
    status = CPXchgbds (env, lp, 1, members, lu, bound);
    checkCPXerror(status, std::string("CPXchgbds"),
                  std::string("addConicConstraint"));
  }
  else if (type==OSI_RQUAD) {
    double * qval = new double[numMembers-1];
    qval[0] = -2.0;
    std::fill_n(qval+1, numMembers-1, 1.0);
    int * members2 = new int[numMembers-1];
    members2[0] = members[0];
    std::copy(members+2, members+numMembers, members2+1);
    status = CPXaddqconstr (env, lp, 0, numMembers-1,
                            0.0, 'L', NULL, NULL,
                            members+1, members2, qval, NULL);
    delete[] qval;
    delete[] members2;
    checkCPXerror(status, std::string("CPXaddqconstr"),
                  std::string("addConicConstraint"));
    // leading variable is nonnegative
    double bound[] = {0.0};
    char lu[] = {'L'};
    // update lower bound to 0 for leading variables
    status = CPXchgbds (env, lp, 1, members, lu, bound);
    checkCPXerror(status, std::string("CPXchgbds"),
                  std::string("addConicConstraint"));
    status = CPXchgbds (env, lp, 1, members+1, lu, bound);
    checkCPXerror(status, std::string("CPXchgbds"),
                  std::string("addConicConstraint"));
  }
  else {
    std::cerr << "Unknown cone type!" << std::endl;
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
  // ignore integer constraints
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
  // set log level to 0 if reduce print
  bool reduce_print;
  OsiHintStrength strength;
  getHintParam(OsiDoReducePrint, reduce_print, strength);
  if (reduce_print) {
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 0);
    //CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
    CPXsetintparam(env, CPX_PARAM_BARDISPLAY, 0);
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
  // ignore integer constraints
  switchToLP();
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  if (messageHandler()->logLevel() == 0)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 0);
  else if (messageHandler()->logLevel() == 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 1);
  else if (messageHandler()->logLevel() > 1)
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 2);
  // set log level to 0 if reduce print
  bool reduce_print;
  OsiHintStrength strength;
  getHintParam(OsiDoReducePrint, reduce_print, strength);
  if (reduce_print) {
    CPXsetintparam(env, CPX_PARAM_SIMDISPLAY, 0);
    //CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
    CPXsetintparam(env, CPX_PARAM_BARDISPLAY, 0);
  }
  int status = CPXhybbaropt(env, lp, CPX_ALG_NONE);
  if (status!=0) {
    std::cerr << "Cplex did not return 0 status." << std::endl;
    throw std::exception();
  }
}
