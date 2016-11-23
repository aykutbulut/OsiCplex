#ifndef OsiCplexSolverInterface_H
#define OsiCplexSolverInterface_H

#include <OsiConicSolverInterface.hpp>
#include <OsiCpxSolverInterface.hpp>
#include <CoinPackedMatrix.hpp>

class OsiCplexSolverInterface: virtual public OsiConicSolverInterface,
                               virtual public OsiCpxSolverInterface {

  OsiLorentzConeType getLorentzConeType(int i) const;
public:
  // default constructor
  OsiCplexSolverInterface();
  // copy constructor
  OsiCplexSolverInterface(const OsiCplexSolverInterface & other);
  // copy assignment operator
  OsiCplexSolverInterface & operator=(const OsiCplexSolverInterface & rhs);
  virtual ~OsiCplexSolverInterface();
  // get conic constraints
  virtual void getConicConstraint(int index, OsiLorentzConeType & type,
                                  int & numMembers,
                                  int *& members) const;
  // add conic constraints
  // add conic constraint in lorentz cone form
  virtual void addConicConstraint(OsiLorentzConeType type,
                                  int numMembers,
                                  int const * members);
  // add conic constraint in |Ax-b| <= dx-h form
  virtual void addConicConstraint(CoinPackedMatrix const * A, CoinPackedVector const * b,
                                  CoinPackedVector const * d, double h);
  virtual void removeConicConstraint(int index);
  virtual void modifyConicConstraint(int index, OsiLorentzConeType type,
                                     int numMembers,
                                     const int * members);
  virtual int getNumCones() const;
  virtual int getConeSize(int i) const;
  virtual OsiConeType getConeType(int i) const;
  virtual void getConeSize(int * size) const;
  virtual void getConeType(OsiConeType * type) const;
  virtual void getConeType(OsiLorentzConeType * type) const;
  virtual OsiConicSolverInterface * clone(bool copyData=true) const;
  virtual int readMps(const char * filename, const char * extension="mps");
  /// warm start methods
  virtual CoinWarmStart* getWarmStart() const {return NULL;}
  // virtual functions inherited from OsiSolverInterface
  // re-define them since definitions in OsiCpxSolverInterface
  // are for linear problems and does not work for conic problems
  virtual void initialSolve();
  virtual void resolve();
};

#endif
