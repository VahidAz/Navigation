#ifndef _FUZZY_HH_
#define _FUZZY_HH_


#include "CommonHeaders.hh"
#include "Debug.hh"


#define MINTHERESHOLD 0.5   // Threshold For Recognize Near Objects To Calculating Fuzzy Inputs
#define THERREPEATPOS 1000  // Threshold For Recognizing That Robot Can't Find Wall


class Fuzzy: public Debug
{
  public:
    Fuzzy();

    void          extractFuzIn(QList<double>,double,double);    // Set (Laser Data, X of Robot, Y of Robot) and Calculates Fuzzy Inputs
    QList<double> getFuzOutNav();                               // Return Calculated Speed of Wheels
    void          setNavDir(int);                               // Set Direction of Wall Following, 1 For Right, -1 For Left
    QString       getFuzzyKind();

  private:
    fl::Engine*         engNavR;    // Fuzzy Engine For Right Wall Following
    fl::Engine*         engNavL;    // Fuzzy Engine For Left  Wall Following
    fl::Engine*         engNavObs;  // Fuzzy Engine For Obstacle Avoidance

    // Fuzzy Inputs For Right Wall Following
    fl::InputVariable*  inRObNavR;  // Right Objects
    fl::InputVariable*  inFObNavR;  // Front Objects
    fl::InputVariable*  inLObNavR;  // Left  Objects

    // Fuzzy Inputs For Left Wall Following
    fl::InputVariable*  inRObNavL;  // Right Objects
    fl::InputVariable*  inFObNavL;  // Front Objects
    fl::InputVariable*  inLObNavL;  // Left  Objects

    // Fuzzy Inputs For Obstcale Avoidance
    fl::InputVariable*  inRObNavObs;    // Right Objects
    fl::InputVariable*  inFObNavObs;    // Front Objects
    fl::InputVariable*  inLObNavObs;    // Left  Objects

    // Fuzzy Outputs For Right Wall Following
    fl::OutputVariable* outRSpdNavR;    // Right Wheel
    fl::OutputVariable* outLSpdNavR;    // Left  Wheel

    // Fuzzy Outputs For Left Wall Following
    fl::OutputVariable* outRSpdNavL;    // Right Wheel
    fl::OutputVariable* outLSpdNavL;    // Left  Wheel

    // Fuzzy Outputs For Obstacle Avoidance
    fl::OutputVariable* outRSpdNavObs;  // Right Wheel
    fl::OutputVariable* outLSpdNavObs;  // Left  Wheel

    // Fuzzy Rules
    fl::RuleBlock*      rulBlkNavR;     // Right Wall Following
    fl::RuleBlock*      rulBlkNavL;     // Left  Wall Following
    fl::RuleBlock*      rulBlkNavObs;   // Obstacle Avoidance

    QList<double>       spdsNav,    // Calculated Speed With Fuzzy
                        lastSpd;    // Keep Speed of Previous Step

    QList<QPoint>       allPos;     // Keep All Positions That Robot Meets Those

    QPoint              lastPos,    // Last Position of Robot In Previous Step
                        lastPosRep; // Keep Position of Robot To Recognize Stop Robot

    QString             fuzKind;

    int                 countStop,  // Count Number of Steps That Robot Is in Stop
                        wallNavDir; // Keep Direction of Wall Following, +1 For Right Wall Following and -1 For Left Wall Following

    bool                flagConstPos,   // Flag For Recognizing Stop Robot
                        flagRepPos;     // Flag For Recognizing Repeadet Position

    void initFuzNavR  ();   // Initialize Right Wall Following Fuzzy
    void initFuzNavL  ();   // Initialize Left  Wall Following Fuzzy
    void initFuzNavObs();   // Initialize Obstacle Avoidance Fuzzy

    void setFuzInNav  (double,double,double,double,double); // Set Fuzzy Inputs In Algorithm
    bool posStop      (double,double);                      // Checking That Robot Stops In a Constant Position
    void setAllPos    (double,double);                      // Set All Meeted Position By Robot
    bool chkReptedPos (double,double);                      // Checkeing That New Position Is a New Or Meeted Position

    void setFuzzyKind(QString); //
};


#endif // _FUZZY_HH_


// for fuzzy navigation i can change the trianguat function to motavazi azla
// repeat hal shod
// barkhord dare (bayad change way of calculate fuzzy input) hal shod
