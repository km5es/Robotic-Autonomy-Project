#include "ros/ros.h"
#include "human_prob_motion/HumanProbMotion.h"

#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

// Define variables and matrices
int nk = 10;

// KF matrices
float TA [4] = {1, 0, 0, 1};	float TB [4] = {1, 0, 0, 1};
float TC [4] = {1, 0, 0, 1};	float TSwkm [4] = {1, 0, 0, 1};

// robot and target positions: from requests
float Sx [2] = {-5, 0};	float Su [2] = {1, 0};
float Tx [2] = {1, 1};	float Tu [2] = {0.25, 0};
float TmxkmGkm [2] = {0, 0};	float TSxkmGkm [4] = {5, 0 , 0, 5};

float Tdx;	float Tdy;
float dxk;	float dyk;	float dthk;	float dt = 0.1;

// derive_grad
float dl = 0.1;	float x1 [2];	float x2 [2];
float dxdg [2] = {0, 0};	float f1 [4];	float f2 [4];
float prob;	float di;	float pi;

float TmxkGkm [2];		float TSxkGkm [4];

float TAt [4];	float TSTAt [4];	float TATSTAt [4];

// other matrices in code
float STz [2];	float dst [2];	float d;	float dmat [4];
float th;	float trig [4];	float TSvk [4];

float TmxkGk [2];	float TSxkGk [4];	float TKk [4];
float I [4] = {1, 0, 0, 1};
// for TKk
float Ct [4];	float SCt [4];	float CSCt [4];
float CSCtS [4];	float invT [4];	float CtinvT [4];

// for TmxkGk
float Cm [2];	float TzCm [2];	float KkTzCm [2];

// for TSxkGk
float KkC [4];	float IKkC [4];

  // print to check matrix
  //for (int i = 0; i<nu; i++){
    //ROS_INFO("%lf",(float)Upi[i]);
  //}

// function to multiply two matrices
float multiply2x2by2x2(float a[], float b[], float c[]){
  c[0] = (a[0]*b[0]) + (a[1]*b[2]);  
  c[1] = (a[0]*b[1]) + (a[1]*b[3]);
  c[2] = (a[2]*b[0]) + (a[3]*b[2]);
  c[3] = (a[2]*b[1]) + (a[3]*b[3]);
}

float transpose2x2(float a[], float b[]){
  float a1 = a[1];
  float a2 = a[2];
  b[0] = a[0];
  b[1] = a2;
  b[2] = a1;
  b[3] = a[3];
}

float add2x2(float a[], float b[], float c[]){
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
  c[3] = a[3] + b[3];
}

float subtract2x2(float a[], float b[], float c[]){
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
  c[3] = a[3] - b[3];
}

float inv2x2(float a[], float b[]){
  float det = (a[0]*a[3])-(a[2]*a[1]);
  b[0] = (1/det)*a[3];
  b[1] = (1/det)*(-1)*a[1];
  b[2] = (1/det)*(-1)*a[2];
  b[3] = (1/det)*a[0];
}

float sensor(float x, float y, float di, float pi){
  float dc_min = 0.5;	float dc_max = 3.5;
  float w = 7;
  if (pi<-0.541 || pi>0.541){
    prob = 0;
    return prob;
  }
  else {
    prob = (1/(exp(-w*(x-dc_min))+1))-(1/(exp(-w*(x-dc_max))+1));
    return prob;
  }
}

bool prob_motion(human_prob_motion::HumanProbMotion::Request  &req, human_prob_motion::HumanProbMotion::Response &res)
{
  // request: robot and target positions
  // target position and movement
  Sx[0] = req.Sx0;
  Sx[1] = req.Sx1;
  Su[0] = req.Su0;
  Su[1] = req.Su1;
  Tx[0] = req.Tx0;
  Tx[1] = req.Tx1;
  Tu[0] = req.vli;
  Tu[1] = req.vri;

  TmxkmGkm[0] = req.Tm0;
  TmxkmGkm[1] = req.Tm1;
  TSxkmGkm[0] = req.TS0;
  TSxkmGkm[1] = req.TS1;
  TSxkmGkm[2] = req.TS2;
  TSxkmGkm[3] = req.TS3;
  
  ROS_INFO("Received: ");
  ROS_INFO("Position of robot: %lf, %lf", (float)req.Sx0, (float)req.Sx1);
  ROS_INFO("Movement of robot: %lf, %lf", (float)req.Su0, (float)req.Su1);
  ROS_INFO("Position of target: %lf, %lf", (float)req.Tx0, (float)req.Tx1);
  ROS_INFO("Movement of target: vri=%lf, vli=%lf, thk=%lf", (float)req.vri, (float)req.vli, (float)req.thk);
  ROS_INFO("Tm: %lf, %lf", (float)req.Tm0, (float)req.Tm1);
  ROS_INFO("TS: %lf, %lf, %lf, %lf", (float)req.TS0, (float)req.TS1, (float)req.TS2, (float)req.TS3);
  
  // TmxkGkm
  //TmxkGkm[0] = (TA[0]*TmxkmGkm[0])+(TA[1]*TmxkmGkm[1])+(TB[0]*Tu[0])+(TB[1]*Tu[1]);
  //TmxkGkm[1] = (TA[2]*TmxkmGkm[0])+(TA[3]*TmxkmGkm[1])+(TB[2]*Tu[0])+(TB[3]*Tu[1]);
  TmxkGkm[0] = TmxkmGkm[0]+dt*Tdx;
  TmxkGkm[1] = TmxkmGkm[1]+dt*Tdy;

  /////////////////////////////////////////////////////////////////////
  //TA=derive_grad(TmxkmGkm,dl=0.1,'wind',u);
  f1[0] = Tdx;
  f1[1] = Tdx;
  f1[2] = Tdy;
  f1[3] = Tdy;
  f2[0] = Tdx;
  f2[1] = Tdx;
  f2[2] = Tdy;
  f2[3] = Tdy;
  
  TA[0] = (f2[0]-f1[0])/dl;
  TA[1] = (f2[1]-f1[1])/dl;
  TA[2] = (f2[2]-f1[2])/dl;
  TA[3] = (f2[3]-f1[3])/dl;
  /////////////////////////////////////////////////////////////////////

  // TSxkGkm
  transpose2x2(TA, TAt);
  multiply2x2by2x2(TSxkmGkm, TAt, TSTAt);
  multiply2x2by2x2(TA, TSTAt, TATSTAt);
  add2x2(TATSTAt, TSwkm, TSxkGkm);

  // Update robot and target positions;
  Sx[0] = Sx[0] + Su[0];
  Sx[0] = Sx[0] + Su[1];
  Tdx = (0.5)*(req.vri + req.vli)*cos(req.thk); //req.Tu0;
  Tdy = (0.5)*(req.vri + req.vli)*sin(req.thk); //req.Tu1;
  Tx[0] = Tx[0] + dt*Tdx;
  Tx[1] = Tx[1] + dt*Tdy;
  res.Sxr0 = Sx[0];
  res.Sxr1 = Sx[1];
  res.Txr0 = Tx[0];
  res.Txr1 = Tx[1];
  res.thk = req.thk+dt*(1/0.35)*(req.vri-req.vli);
  STz[0] = Tx[0];
  STz[1] = Tx[1];
  
  dst[0] = STz[0] - Sx[0];
  dst[1] = STz[1] - Sx[1];
  d = sqrt((dst[0]*dst[0])+(dst[1]*dst[1]));

  th = atan(dst[1]/dst[0]);
  trig[0] = cos(th);
  trig[1] = -sin(th);
  trig[2] = sin(th);
  trig[3] = cos(th);
  dmat[0] = d;
  dmat[1] = 0;
  dmat[2] = 0;
  dmat[3] = 0.2*d;
  multiply2x2by2x2(trig, dmat, TSvk);
  
  // correct
  ////////////////////////////////////////////////////////////////////
  // TC = derive_grad(TmxkGkm, dl=0.1, 'sensor',[]);
  di = sqrt(((Tx[0]-Sx[0])*(Tx[0]-Sx[0]))+((Tx[1]-Sx[1])*(Tx[1]-Sx[1])));
  pi = atan((Tx[1]-Sx[1])/(Tx[0]-Sx[0]));
  prob = sensor(Tx[0], Tx[1], di, pi);
  f1[0] = prob*(TmxkmGkm[0]-(dl/2));
  f1[1] = prob*TmxkmGkm[0];
  f1[2] = prob*TmxkmGkm[1];
  f1[3] = prob*(TmxkmGkm[0]-(dl/2));
  f2[0] = prob*(TmxkmGkm[0]+(dl/2));
  f2[1] = prob*TmxkmGkm[0];
  f2[2] = prob*TmxkmGkm[1];
  f2[3] = prob*(TmxkmGkm[1]+(dl/2));
  
  TC[0] = (f2[0]-f1[0])/dl;
  TC[1] = (f2[1]-f1[1])/dl;
  TC[2] = (f2[2]-f1[2])/dl;
  TC[3] = (f2[3]-f1[3])/dl;
  ////////////////////////////////////////////////////////////////////
  // TKk
  transpose2x2(TC, Ct);
  multiply2x2by2x2(TSxkGkm, Ct, SCt);
  multiply2x2by2x2(TC, SCt, CSCt);
  add2x2(CSCt, TSvk, CSCtS);
  inv2x2(CSCtS, invT);
  multiply2x2by2x2(Ct, invT, CtinvT);
  multiply2x2by2x2(TSxkGkm, CtinvT, TKk);

  // TmxkGk
  Cm[0] = (TC[0]*TmxkGkm[0])+(TC[1]*TmxkGkm[1]);
  Cm[1] = (TC[2]*TmxkGkm[0])+(TC[3]*TmxkGkm[1]);
  TzCm[0] = STz[0] - Cm[0];
  TzCm[1] = STz[1] - Cm[1];

  KkTzCm[0] = (TKk[0]*TzCm[0])+(TKk[1]*TzCm[1]);
  KkTzCm[1] = (TKk[2]*TzCm[0])+(TKk[3]*TzCm[1]);

  TmxkGk[0] = TmxkGkm[0] + KkTzCm[0];
  TmxkGk[1] = TmxkGkm[1] + KkTzCm[1];

  // TSxkGk
  multiply2x2by2x2(TKk, TC, KkC);
  subtract2x2(I, KkC, IKkC);
  multiply2x2by2x2(IKkC, TSxkGkm, TSxkGk);

  res.Tmr0 = TmxkGk[0];
  res.Tmr1 = TmxkGk[1];
  res.TSr0 = TSxkGk[0];
  res.TSr1 = TSxkGk[1];
  res.TSr2 = TSxkGk[2];
  res.TSr3 = TSxkGk[3];

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "human_motion_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("human_prob_motion", prob_motion);  

  ROS_INFO("Ready to model human prob motion");
  ros::spin();

  return 0;
}

