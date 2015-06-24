#include <OsiCplexSolverInterface.hpp>

int main(int argc, char ** argv) {
  OsiConicSolverInterface * si = new OsiCplexSolverInterface();
  si->readMps(argv[1]);
  si->initialSolve();
  std::cout << "Optimal solution is " << si->getObjValue() << std::endl;
  delete si;
  // print conic constraints

  return 0;
}
