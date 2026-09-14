int main(int argc,char**) {
 std::vector<double> J(36,0.); // exactly normal, tangent1, tangent2
 J[2]=1; J[12]=1; J[25]=1;
 int jb[]={0,-1,0,-1,0,-1}; int fi[]={-1,0,0};
 IndexError order[]={{0},{2},{1}}; // default PGS fills friction rows from tail
 double hi[]={100,.6,.6},l[]={100,0,0},le[]={150,0,0};
 dxBody b; b.lvel[0]=3; b.lvel[1]=4; dxBody* bodies[]={&b};
 double lo=-999,up=-999,loe=-999,upe=-999;
 if(argc>1){
   order[1].index=1;order[2].index=2;
   if(argc>2) b.lvel[0]=b.lvel[1]=0;
   quickstep::dxConeFrictionModel(lo,up,loe,upe,jb,J.data(),1,0,0,3,1,bodies,1,order,fi,nullptr,hi,l,le);
   std::cout<<"{\"lower\":"<<lo<<",\"upper\":"<<up<<",\"erp_lower\":"<<loe<<",\"erp_upper\":"<<upe<<"}\n";
 }else{
   quickstep::dxConeFrictionModel(lo,up,loe,upe,jb,J.data(),2,0,0,3,1,bodies,1,order,fi,nullptr,hi,l,le);
 }
}
