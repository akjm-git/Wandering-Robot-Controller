//Anthony McGuigan
//com596 - intellegent robotics
//ulster university may 2021

#include <iostream>
#include <signal.h>
#include <math.h>
#include <libplayerc++/playerc++.h>
#include "Vector2D.hh"

using namespace std;
using namespace PlayerCc;

bool Running = true;
void programShutdown(int );

double rMax = 3.0;
double vMax = 0.4;
double sSigma = 30 * M_PI / 180;

double computeStimulusRight(const RangerProxy & laser)
{
  double retVal = 0;
  double wSum = 0.0;
  double w = 0.0;
  
  int beams = laser.GetRangeCount();
  for (int i = 0; i < beams; ++i)
    {
      double th = laser.GetMinAngle() + i * laser.GetAngularRes(); //get angle for scan
      if (th < 0) //beams on the left have positive heading, right have negative
		{
		  w = exp(- (th / th) * (sSigma / sSigma)); 
		  wSum += w;
		  
          //normalize
		  double normR = laser[i] / rMax;
		  // if > 1 set 1 if less set 0
		  normR = (normR > 1) ? 1 : normR;
		  retVal += (w * normR);
		}
    }
  retVal /= wSum;
  return retVal;
}

double computeStimulusLeft(const RangerProxy & laser)
{
  double retVal = 0;
  double wSum = 0.0;
  double w = 0.0;

  int beams = laser.GetRangeCount();
  for (int i = 0; i < beams; ++i)
    {
      double th = laser.GetMinAngle() + i * laser.GetAngularRes(); //get angle for scan
      if (th > 0) //beams on the left have positive heading, right have negative
		{
		  w = exp(- (th / th) * (sSigma / sSigma)); 
		  wSum += w;
		  
          //normalize
		  double normR = laser[i] / rMax;
		  //if > 1 set to 1 if less set 0
		  normR = (normR > 1) ? 1 : normR;
		  retVal += (w * normR);
		}
    }
  retVal /= wSum;
  return retVal;
}

double wanderV(double leftStimulus, double rightStimulus)  
{
  return vMax * ((leftStimulus + rightStimulus) / 2);
}

double wanderW(double leftStimulus, double rightStimulus) 
{
  return vMax * ((leftStimulus - rightStimulus) / 0.4);
}

bool doorDetection(const RangerProxy& laser, Vector2D& midpoint) {
    bool gapFound = false;
    //min threshold for detection between scans
    double minGap = 1;
    bool foundLeftSide = false, foundRightSide = false;
    int rightStartIndex, leftStartIndex;

    //door coords
    Vector2D rightCoordinates, leftCoordinates;

    //from the second laser scan to the last, until a door is found
    for (int i = 1; !gapFound && i < laser.GetRangeCount(); i++) {
        // Difference between current scan and the previous
        double diff = laser[i] - laser[i - 1];

        //difference is greater than the threshold
        if (diff > minGap) {
            foundRightSide = true;
            rightStartIndex = i - 1; //set right index

            double angleRight = laser.GetMinAngle() + rightStartIndex * laser.GetAngularRes();
            //get coords
            double x = laser[rightStartIndex] * cos(angleRight);
            double y = laser[rightStartIndex] * sin(angleRight);

            //set right coords
            rightCoordinates.X(x);
            rightCoordinates.Y(y);
        }

        if (diff < -minGap && foundRightSide) {
            //door found
            gapFound = true;
            leftStartIndex = i; //set left index

            double angleLeft = laser.GetMinAngle() + leftStartIndex * laser.GetAngularRes();
            //get coords
            double x = laser[leftStartIndex] * cos(angleLeft);
            double y = laser[leftStartIndex] * sin(angleLeft);

            //set left coords
            leftCoordinates.X(x);
            leftCoordinates.Y(y);
        }
    }
    //get door midpoint
    midpoint = (rightCoordinates + leftCoordinates) / 2;
    return gapFound;
}

double reachDoorV(const Vector2D & target, const Vector2D & robot)
{
  Vector2D d;
  double linearVelocity = 0.0;
  double dMax = 2;
  
  d = target - robot; //difference between positions, robot and targe
  double targetDist = d.Length();
  if (targetDist > dMax)
    linearVelocity = vMax;
  else
    linearVelocity = vMax * targetDist / dMax;
  
  return linearVelocity;
}

double reachDoorW(const Vector2D & target, const Vector2D & robot, double robotHeading)
{
  double k = 0.8; //controller value
  
  Vector2D d;
  
  d =  target - robot; //difference between positions, robot and target

  double th = d.Angle() - robotHeading;
  
  if (th > M_PI)
    th -= (2 * M_PI);
  if (th < -M_PI)
    th += (2 * M_PI);
  
  k *= th;
  
  return k;
}

void wander(const RangerProxy& lasers, Position2dProxy& base)
{
    //calc stim
    double stimulusLeft = computeStimulusLeft(lasers);
    double stimulusRight = computeStimulusRight(lasers);

    //calc vel and turning
    double v = wanderV(stimulusLeft, stimulusRight);
    double w = wanderW(stimulusLeft, stimulusRight);

    base.SetSpeed(v, 0, w);
}

int main(int argn, char *argv[])
{
  PlayerClient robotClient("localhost");

  RangerProxy laser(&robotClient, 1);
  Position2dProxy base(&robotClient, 0);

  //for ctrl-c shutdown
  signal(SIGINT, programShutdown);

  //pose, position and heading.
  Vector2D robot;
  double robotHeading;

  //get the pose of robot from server.
  do 
  {
    robotClient.Read();
  }
  while (!laser.IsFresh());
  
  //enable motor
  base.SetMotorEnable(true);
  while (Running)
    {
      robotClient.Read();
      Vector2D midPoint;
      //door dectection
      if(doorDetection(laser, midPoint)) {
        //calc velocity and turning velocities
        double v = reachDoorV(midPoint, robot);
        double w = reachDoorW(midPoint, robot, robotHeading);
        base.SetSpeed(v, w);
      } else {
        //wander randomly avoiding obsticles
        wander(laser, base);
      }
    }
  
  //disable motor
  base.SetMotorEnable(false);

  return 0;
}

void programShutdown(int s)
{
  Running = false;
}
