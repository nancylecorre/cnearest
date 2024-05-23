#include <iostream>
#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

matrix<double> stokeslet(vector<double> x, vector<double> y, double epsilon)
{
  matrix<double> stokeslet(3,3);
  double r2, pireps3,iso;
  vector<double> xd(3);
  const double pi=3.1415926535;
  xd=x-y;
  r2=xd(0)*xd(0)+xd(1)*xd(1)+xd(2)*xd(2);
  pireps3=1.0/(pow(r2+epsilon*epsilon,1.5)*8.0*pi);
  iso=r2+2.0*epsilon*epsilon;
  stokeslet(0,0)=(iso+xd(0)*xd(0))*pireps3;
  stokeslet(1,1)=(iso+xd(1)*xd(1))*pireps3;
  stokeslet(2,2)=(iso+xd(2)*xd(2))*pireps3;
  stokeslet(0,1)=xd(0)*xd(1)*pireps3;
  stokeslet(1,0)=stokeslet(0,1);
  stokeslet(0,2)=xd(0)*xd(2)*pireps3;
  stokeslet(2,0)=stokeslet(0,2);
  stokeslet(1,2)=xd(1)*xd(2)*pireps3;
  stokeslet(2,1)=stokeslet(1,2);
  return stokeslet;
}

vector<double> nn(matrix<double> x,matrix<double> y)
{
  int q,n;
  double r2min,r2;
  vector<double> nn(y.size1());
  n=x.size1();
  q=y.size1();
  for ( int qi = 0; qi < q; qi++)
    {
      r2min=1.0;
      for (int ni = 0; ni < n; ni++)
	{
	  r2=pow(x(ni,0)-y(ni,0),2)+pow(x(ni,1)-y(ni,1),2)+pow(x(ni,2)-y(ni,2),2);
	  if(r2<r2min){
	    r2min=r2;
	    nn(qi)=ni;
	  }
	}
    }
    return nn;
}

matrix<double> sph_projector(matrix<double> xp, double a)
{
  matrix<double> xs(xp.size1(),3);
  double pi=3.1415926535;
  int mpts=xp.size1();
  double r, th, phi;
  for (int j = 0; j < mpts; j++)
    {
      r=xp(j,0)*xp(j,0)+xp(j,1)*xp(j,1)+xp(j,2)*xp(j,2);
      th=acos(xp(j,2)/r);
      //
      if(xp(j,1)==0) phi=-acos((xp(j,0)+1.0e-9)/abs(xp(j,0)+1.0e-9));
      if(xp(j,1)>0)  phi= acos((xp(j,0)+1.0e-9)/pow(pow(xp(j,0)+1.0e-9,2)+pow(xp(j,1),2),0.5));  
      if(xp(j,1)<0)  phi= acos((xp(j,0)+1.0e-9)/pow(pow(xp(j,0)+1.0e-9,2)+pow(xp(j,1),2),0.5))+2.0*pi;
      //
      xs(j,0)=a*sin(th)*cos(phi);
      xs(j,1)=a*sin(th)*sin(phi);
      xs(j,2)=a*cos(th);
    }
  return xs;
}

matrix<double> unif_sphere_discr(double a, int msph)
{
  matrix<double> xp(6*msph*msph,3);
  double ds=2.0/msph;
  double c=1.0;
  int icount=0;
  double xi, et;
  for (int i = 0; i < msph; i++)
    {
      for (int j = 0; j < msph; j++)
	{
	  xi=-1+ds*(j-0.5);
	  et=-1+ds*(i-0.5);
	  xp(icount,0)=c;
	  xp(icount,1)=xi;
	  xp(icount,2)=et;
	  //
	  xp(icount+msph*msph,0)=-xi;
  	  xp(icount+msph*msph,1)=c;
  	  xp(icount+msph*msph,2)=et;
	  //
	  xp(icount+2*msph*msph,0)=xi;
  	  xp(icount+2*msph*msph,1)=et;
  	  xp(icount+2*msph*msph,2)=c;
	  //
	  xp(icount+3*msph*msph,0)=xi;
  	  xp(icount+3*msph*msph,1)=-et;
  	  xp(icount+3*msph*msph,2)=-c;
	  //
	  xp(icount+4*msph*msph,0)=xi;
  	  xp(icount+4*msph*msph,1)=-c;
  	  xp(icount+4*msph*msph,2)=et;
	  //
	  xp(icount+5*msph*msph,0)=-c;
  	  xp(icount+5*msph*msph,1)=-xi;
  	  xp(icount+5*msph*msph,2)=et;
	  //
	  icount++;
	}
    }
  xp=sph_projector(xp,a);
  return xp;
}



int main()
{
  vector<double> x(3);
  matrix<double> X(1,3);
  double epsilon;
  int msph=5;
  int qsph=10;
  matrix<double> xs,Xs;
  double a=1.0;
  x(0)=2.0;
  x(1)=1.5;
  x(2)=1.1;
  X(0,0)=x(0);
  X(0,1)=x(1);
  X(0,2)=x(2);
  epsilon=0.01;
  std::cout << stokeslet(x,x,epsilon);
  std::cout << "\n";
  std::cout <<  nn(X,X);
  std::cout << "\n";
  xs=unif_sphere_discr(a,msph);
  Xs=unif_sphere_discr(a,qsph);
  return 0;
}

  
