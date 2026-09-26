#include <gazebo/ode/ode.h>
#include <cmath>
#include <iostream>
#include <vector>
struct Run { dWorldID world; dJointGroupID contacts; };
static void near(void *raw,dGeomID a,dGeomID b) {
 auto *r=static_cast<Run*>(raw); dContact c[8]{};
 const int n=dCollide(a,b,8,&c[0].geom,sizeof(dContact));
 for(int i=0;i<n;++i){ c[i].surface.mode=dContactApprox1; c[i].surface.mu=0.7;
  auto j=dJointCreateContact(r->world,r->contacts,&c[i]);dJointAttach(j,dGeomGetBody(a),dGeomGetBody(b));}
}
int main(){dInitODE2(0);for(auto model:{pyramid_friction,cone_friction})for(double dt:{.004,.002,.001}){
 Run r{dWorldCreate(),dJointGroupCreate(0)};auto space=dHashSpaceCreate(0);dWorldSetGravity(r.world,0,0,-9.81);
 dWorldSetQuickStepFrictionModel(r.world,model);dWorldSetQuickStepNumIterations(r.world,80);
 dCreatePlane(space,0,0,1,0);auto body=dBodyCreate(r.world);dMass mass;dMassSetBoxTotal(&mass,1.,.3,.2,.2);dBodySetMass(body,&mass);
 auto box=dCreateBox(space,.3,.2,.2);dGeomSetBody(box,body);dBodySetPosition(body,0,0,.4);dBodySetLinearVel(body,.4,.2,0);
 for(int i=0;i<int(2./dt);++i){dSpaceCollide(space,&r,near);dWorldQuickStep(r.world,dt);dJointGroupEmpty(r.contacts);
  for(auto v:{dBodyGetPosition(body),dBodyGetLinearVel(body),dBodyGetAngularVel(body)})for(int k=0;k<3;++k)if(!std::isfinite(v[k]))return 2;}
 auto p=dBodyGetPosition(body);auto v=dBodyGetLinearVel(body);if(p[2]<.08||p[2]>.12||std::abs(v[2])>.1)return 3;
 std::cout<<"model="<<model<<" dt="<<dt<<" z="<<p[2]<<" vz="<<v[2]<<" finite=true\n";
 dJointGroupDestroy(r.contacts);dSpaceDestroy(space);dWorldDestroy(r.world);
 }dCloseODE();}
