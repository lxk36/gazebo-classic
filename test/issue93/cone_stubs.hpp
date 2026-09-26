// Minimal scalar/body fixtures for exercising an extracted production function.
// These fixtures do not implement an ODE/Gazebo plant or a contact solver.
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using dReal = double;
using dRealPtr = const double*;
using dRealMutablePtr = double*;
struct dxBody { double lvel[3]{}, avel[3]{}; };
struct IndexError { int index; };
void dSetZero(double* p, int n) { std::fill(p,p+n,0.); }
double dFabs(double x) { return std::fabs(x); }
constexpr int d_ERR_LCP = 1;
void dMessage(int, const char*) { throw std::runtime_error("constraint index error"); }
namespace quickstep {
double dot6(const double* a,const double* b) { double r=0; for(int i=0;i<6;++i)r+=a[i]*b[i]; return r; }
void dxConeFrictionModel(dReal&,dReal&,dReal&,dReal&,int*,dRealPtr,int,int,int,int,int,dxBody*const*,int,const IndexError*,const int*,dRealPtr,dRealPtr,dRealMutablePtr,dRealMutablePtr);
}
