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
  checkCPXerror(status, std::string("CPXsetintparam"),
                std::string("OsiCplexSolverInterface"));
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
  checkCPXerror(status, std::string("CPXsetintparam"),
                std::string("OsiCplexSolverInterface"));
  // add conic constraints
  for (int i=0; i<other.getNumCones(); ++i) {
    OsiLorentzConeType type;
    int size;
    int * members = 0;
    other.getConicConstraint(i, type, size, members);
    addConicConstraint(type, size, members);
    delete[] members;
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
  // cplex status, 0 for success
  int status;
  // number of linear part nonzeros
  int linnz = 0;
  int linsurplus = 0;
  // number of quadratic part nonzeros
  int qnz = 0;
  numMembers = 0;
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getMutableEnvironmentPtr();
  // detect array lengths
  status = CPXgetqconstr(env, lp, &linnz, &qnz, NULL, NULL,
                         NULL, NULL, 0, &linsurplus,
                         NULL, NULL, NULL, 0, &numMembers, index);
  if (status == CPXERR_NEGATIVE_SURPLUS) {
    numMembers = -numMembers;
  }
  else {
    std::cerr << "Cone is size 0, this is a problem!" << std::endl;
    throw std::exception();
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
  checkCPXerror(status, std::string("CPXgetqconstr"),
                std::string("getConicConstraint"));
  if (qval[0]==-1.0) {
    // quad constraint is as follows
    // size is numMembers
    // qrow: m[0] m[1] m[2] ... m[numMembers-1]
    // qcol: m[0] m[1] m[2] ... m[numMembers-1]
    // qval:  -1   1    1   ... 1.0
    //
    type = OSI_QUAD;
    members = qrow;
    delete[] qcol;
    delete[] qval;
  }
  else if (qval[0]==-2.0) {
    // quad constraint is as follows
    // size is numMembers-1
    // index:  0    1    2        numMembers-2
    // qrow: m[0] m[2] m[3] ... m[numMembers-1]
    // qcol: m[1] m[2] m[3] ... m[numMembers-1]
    // qval:  -2   1    1   ... 1.0
    //
    type = OSI_RQUAD;
    numMembers++;
    members = new int[numMembers];
    members[0] = qrow[0];
    std::copy(qcol, qcol+numMembers-1, members+1);
    delete[] qrow;
    delete[] qcol;
    delete[] qval;
  }
  else {
    delete[] qrow;
    delete[] qcol;
    delete[] qval;
    std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
    std::cerr << "This should not happen!" << std::endl;
    throw std::exception();
  }
}

// add conic constraint in lorentz cone form
void OsiCplexSolverInterface::addConicConstraint(OsiLorentzConeType type,
                                                 int numMembers,
                                                 int const * members) {
  int status;
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getEnvironmentPtr();
  if (type==OSI_QUAD) {
    // quad constraint is as follows
    // size is numMembers
    // qrow: m[0] m[1] m[2] ... m[numMembers-1]
    // qcol: m[0] m[1] m[2] ... m[numMembers-1]
    // qval:  -1   1    1   ... 1.0
    //
    int const * qrow = members;
    int const * qcol = members;
    double * qval = new double[numMembers];
    qval[0] = -1.0;
    std::fill_n(qval+1, numMembers-1, 1.0);
    status = CPXaddqconstr (env, lp, 0, numMembers,
                            0.0, 'L', NULL, NULL,
                            qrow, qcol, qval, NULL);
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
    // quad constraint is as follows
    // size is numMembers-1
    // index:  0    1    2        numMembers-2
    // qrow: m[0] m[2] m[3] ... m[numMembers-1]
    // qcol: m[1] m[2] m[3] ... m[numMembers-1]
    // qval:  -2   1    1   ... 1.0
    //
    int * qrow = new int[numMembers-1];
    int * qcol = new int[numMembers-1];
    double * qval = new double[numMembers-1];
    qrow[0] = members[0];
    std::copy(members+2, members+numMembers, qrow+1);
    qcol[0] = members[1];
    std::copy(members+2, members+numMembers, qcol+1);
    qval[0] = -2.0;
    std::fill_n(qval+1, numMembers-2, 1.0);
    status = CPXaddqconstr (env, lp, 0, numMembers-1,
                            0.0, 'L', NULL, NULL,
                            qrow, qcol, qval, NULL);
    delete[] qrow;
    delete[] qcol;
    delete[] qval;
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
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}


void OsiCplexSolverInterface::removeConicConstraint(int index) {
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

void OsiCplexSolverInterface::modifyConicConstraint(int index,
                                                    OsiLorentzConeType type,
                                                    int numMembers,
                                                    const int * members) {
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
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
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

OsiConeType OsiCplexSolverInterface::getConeType(int i) const {
  CPXLPptr lp = getMutableLpPtr();
  CPXENVptr env = getMutableEnvironmentPtr();
  int linnz = 0;
  int qnz = 0;
  int linsurplus;
  int qsurplus;
  int lcap;
  int qcap;
  CPXINT status;
  /* Call CPXgetqconstr() a first time with zero-length buffers to figure
   * how long the buffers must be.
   */
  status = CPXgetqconstr (env, lp, &linnz, &qnz, NULL, NULL,
                          NULL, NULL, 0, &linsurplus,
                          NULL, NULL, NULL, 0, &qsurplus, i);
  if (status!=CPXERR_NEGATIVE_SURPLUS) {
    checkCPXerror(status, std::string("CPXgetqconstr"),
                  std::string("getConeType"));
  }
  else {
    lcap = -linsurplus;
    qcap = -qsurplus;
    linnz = -linsurplus;
    qnz = -qsurplus;
  }
  int * qcol = new int[qcap];
  int * qrow = new int[qcap];
  double * qval = new double[qcap];
  double rhs;
  char sense;
  status = CPXgetqconstr (env, lp, &linnz, &qnz, &rhs,
                          &sense,
                          0, 0, 0, &linsurplus,
                          qrow, qcol, qval, qcap,
                          &qsurplus, i);
  checkCPXerror(status, std::string("CPXgetqconstr"),
                std::string("getConeType"));
  // check qval[0], if -1 QUAD, if -2 RQUAD
  if (qval[0]==-1.0) {
  }
  else if (qval[0]==-2.0) {
  }
  else {
    std::cerr << "This should not happen!" << std::endl;
    throw std::exception();
  }
  return OSI_LORENTZ;
}

OsiLorentzConeType OsiCplexSolverInterface::getLorentzConeType(int i) const {
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
  std::cerr << "Not implemented yet!" << std::cerr;
  throw std::exception();
}

// fills array of cone sizes.
void OsiCplexSolverInterface::getConeSize(int * size) const {
  std::cerr << "file: " << __FILE__ << " line: " << __LINE__ << std::endl;
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
  OsiConicSolverInterface * new_solver = new OsiCplexSolverInterface(*this);
  return new_solver;
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
