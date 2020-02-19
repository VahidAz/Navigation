#ifndef _MANAGE_H_
#define _MANAGE_H_


#include "CommonHeaders.hh"
#include "LA.hh"
#include "Fuzzy.hh"


#define DEFUALTNAVDIR 1 // 1 For Right Wall Following, -1 For Left Wall Following


using namespace gazebo;


class Manage:public Debug
{
  public:
    Manage();

    void start();

  private:
    LA    la;   // Object of Learning Automata
    Fuzzy fz;   // Object of Fuzzy

    QList<double> laserarray;   // Keeping Laser Data

    double        maxSpeed,     // Max Motor Speed for Robot, Left Speed and Right Speed
                  rightSped,    // Variable For Right Speed of Robot
                  leftSped,     // Variable For Left  Speed of Robot
                  maxDist,
                  trAng,
                  crnHAng,
                  minDist;

    int           time,         // Keeping Time of Simulator
                  curDirWall,   // Current Direction of Wall
                  tarwall,      // Keep Target Direction After LA
                  counter,      // Counter For Counting Iteration For Turning And Moving
                  startTime,    // Keep Start Time of LA
                  noVic,        // Count Number of Victims
                  dirTrn;

    bool          flagFuz,      // Fuzzy Flag
                  flagLA,       // Learning Automata Flag
                  flagEval,     // Evaluation Flag

                  flagRCamBlue,     // Right Camera for Blue
                  flagRCamWhite,    // Right Camrea For White
                  flagRCamYellow,   // Right Cemera For Yellow

                  flagLCamBlue,     // Left Camera For Blue
                  flagLCamWhite,    // Left Camera For White
                  flagLCamYellow,   // Left Camera For Yellow

                  flagTrnLeft,  // Turn Left Flag
                  flagTrnRight, // Turn Right Flag
                  flagMove,     // Move Flag

                  flagVictim,   // Flag Victiom To Recognize Fining Victim or Not
                  flagFindVic;  // Flag Find Victiom, To Active An Deactive Finding Victim Function

   // QPoint        lastPosVic;   // Keep Last Position That Robot Find Victim

    gazebo::transport::NodePtr       nodeLaser;  // For Reading Data of Laser
    gazebo::transport::NodePtr       nodeVel;    // For Sending Velacity Command to Robot
    gazebo::transport::NodePtr       nodeInfo;   // For Reading Data of Pose Info
    gazebo::transport::NodePtr       nodeLCam;   // For Reading Data of Left Camera
    gazebo::transport::NodePtr       nodeRCam;   // For Reading Data of Right Camera
    gazebo::transport::NodePtr       nodeCCam;   // For Reading Data of Center Camera

    gazebo::transport::SubscriberPtr subLas;     // For Subscribe To Laser  Topic
    gazebo::transport::SubscriberPtr subInfo;    // For Subscribe To Info   Topic
    gazebo::transport::SubscriberPtr subLCam;    // For Subscribe To Left Camera Topic
    gazebo::transport::SubscriberPtr subRCam;    // For Subscribe To Right Camera Topic
    gazebo::transport::SubscriberPtr subCCam;    // For Subscribe To Center Camera Topic

    gazebo::transport::PublisherPtr  pub;        // For Publish Velocity Command to Vel_Cmd Topic

    gazebo::msgs::Pose               poseMsg;    // For Convert Velocity Command to Gazebo Message
    gazebo::math::Pose               poseTemp;   // For Set Velocity of Wheels
    gazebo::math::Vector3            posRobot;   // For Keeping Position  of Robot
    gazebo::math::Vector3            startPos, lastPosForMove, lastPosVic, lastPosLA;   // For Keeping Position  of Robot
    gazebo::math::Quaternion         rotRobot;   // For Keeping Oriention of Robot

    void parseLas (ConstLaserScanStampedPtr&);  // Parse Laser Data
    void parseInfo(ConstPosesStampedPtr&);      // Parse Info Message
    void parseLCam(ConstImageStampedPtr&);      // Parse Image of Left Camera
    void parseRCam(ConstImageStampedPtr&);      // Parse Image of Right Camera
    void parseCCam(ConstImageStampedPtr&);      // Parse Image of Center Camera

    void stopRobot  ();     // Set Speed of Wheels To Zero
    void moving     (int);  // Moving Staraight

    void   updateFlag (int,int,double); // Update Flag of Situation

    double findTargetDegree(int,double);
};


#endif  //  _Manage_HH_


// victim finding have problem, for solve it we need to Image Processing ( hal shod )
// Faght Turn To
