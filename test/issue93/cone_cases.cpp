void near(double a,double b){if(std::fabs(a-b)>1e-10)throw std::runtime_error("bounds mismatch");}
int main(){
 unsigned count=0;
 for(int rows:{3,4}) for(int body_count:{1,2}) {
  std::vector<int> permutation(rows); for(int k=0;k<rows;++k)permutation[k]=k;
  do {
   for(int signx:{-1,0,1}) for(int signy:{-1,0,1}) for(bool erp:{false,true}) {
    dxBody first,second; first.lvel[0]=3*signx;first.lvel[1]=4*signy;
    first.avel[2]=.5;
    second.lvel[0]=.7;second.lvel[1]=-.3;second.avel[2]=-.2;
    dxBody* bodies[]={&first,&second};
    std::vector<double> J(rows*12,0.);
    J[2]=1; J[12]=1; J[25]=1;
    J[17]=-.2; J[29]=.3;
    J[18]=-1;J[23]=.1;J[31]=-1;J[35]=-.4;
    if(rows==4) J[41]=1;
    std::vector<int> jb(rows*2),fi(rows,0);fi[0]=-1;
    std::vector<IndexError> order(rows);
    for(int k=0;k<rows;++k){jb[2*k]=0;jb[2*k+1]=body_count==2?1:-1;order[k].index=permutation[k];}
    std::vector<double> hi(rows,.6),lambda(rows,0.),lambda_erp(rows,0.);
    hi[2]=.3;lambda[0]=100;lambda_erp[0]=150;
    const double vf1=first.lvel[0]-.2*first.avel[2]+(body_count==2?-second.lvel[0]+.1*second.avel[2]:0.);
    const double vf2=first.lvel[1]+.3*first.avel[2]+(body_count==2?-second.lvel[1]-.4*second.avel[2]:0.);
    const double speed=std::hypot(vf1,vf2);
    for(int k=0;k<rows;++k) if(permutation[k]==1||permutation[k]==2){
      int row=permutation[k]; double lo=9,up=9,loe=9,upe=9;
      // A single-row solve chunk must not change storage-based pairing.
      quickstep::dxConeFrictionModel(lo,up,loe,upe,jb.data(),J.data(),row,0,k,1,body_count,bodies,k,order.data(),fi.data(),nullptr,hi.data(),lambda.data(),erp?lambda_erp.data():nullptr);
      double expected=std::fabs(row==1?vf1:vf2)/speed*hi[row]*100;
      near(up,expected);near(lo,-expected);near(upe,erp?1.5*expected:0.);near(loe,erp?-1.5*expected:0.);++count;
    }
   }
  }while(std::next_permutation(permutation.begin(),permutation.end()));
 }
 std::vector<double> J(7*12,0.);J[4*12+2]=1;J[5*12]=1;J[6*12+1]=1;
 int jb[14]={};for(int k=0;k<7;++k)jb[2*k+1]=-1;
 double hi[7]={0,0,0,0,1,.6,.6},l[7]={0,0,0,0,100,0,0},le[7]={0,0,0,0,150,0,0};
 dxBody body;body.lvel[2]=.8;dxBody* bodies[]={&body};
 for(int row:{5,6}){double lo=9,up=9,loe=9,upe=9;
 quickstep::dxConeFrictionModel(lo,up,loe,upe,jb,J.data(),row,4,0,1,1,bodies,0,nullptr,nullptr,nullptr,hi,l,le);
 near(lo,0);near(up,0);near(loe,0);near(upe,0);++count;}
 std::cout<<"{\"fixed_cone_cases\":"<<count<<",\"all_passed\":true,\"full_gazebo_runtime_tested\":false}\n";
}
