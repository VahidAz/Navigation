#include "Fuzzy.hh"


// Constructor
Fuzzy::Fuzzy():Debug()
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In Constructor of Fuzzy Class.";

    // Initialize Right Wall Following
    initFuzNavR();

    // Initialize Left Wall Following
    initFuzNavL();

    // Initialize Obstacle Avoidance
    initFuzNavObs();

    // Keep Speeds of Last Iteration
    lastSpd.clear();
    lastSpd.append(0.0);
    lastSpd.append(0.0);

    allPos.clear();

    lastPos.setX(0);
    lastPos.setY(0);

    lastPosRep.setX(0);
    lastPosRep.setY(0);

    // Counter For Counting Stop Iteration of Robot
    countStop  = 0;

    // Direction of Wall Following
    wallNavDir = 0;

    // Flag For Checking That Robot Is In a constant Position
    flagConstPos = true;

    // Flag For Checking That Robot Is In a Repeated Position
    flagRepPos   = false;
}


void Fuzzy::initFuzNavR()
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In Initialize Right Wall Following, Fuzzy Function.";

    engNavR = new fl::Engine;
    engNavR->setName("right_wall");
    engNavR->addHedge(new fl::Any);
    engNavR->addHedge(new fl::Extremely);
    engNavR->addHedge(new fl::Not);
    engNavR->addHedge(new fl::Seldom);
    engNavR->addHedge(new fl::Somewhat);
    engNavR->addHedge(new fl::Very);

    inLObNavR = new fl::InputVariable;
    inLObNavR->setName("leftobR");
    inLObNavR->setRange(0.000, 5.000);

    inLObNavR->addTerm(new fl::Triangle("near", 0.000,0.000,0.400)); //0.3
    inLObNavR->addTerm(new fl::Triangle("far", 0.200,5.000,5.000));

    engNavR->addInputVariable(inLObNavR);

    inFObNavR = new fl::InputVariable;
    inFObNavR->setName("frontobR");
    inFObNavR->setRange(0.000, 5.000);

    inFObNavR->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inFObNavR->addTerm(new fl::Trapezoid("med", 0.300,0.450,0.550,0.650));
    inFObNavR->addTerm(new fl::Triangle("far", 0.500,5.000,5.000));

    engNavR->addInputVariable(inFObNavR);

    inRObNavR = new fl::InputVariable;
    inRObNavR->setName("rightobR");
    inRObNavR->setRange(0.000, 5.000);

    inRObNavR->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inRObNavR->addTerm(new fl::Trapezoid("med", 0.300,0.450,0.550,0.650));
    inRObNavR->addTerm(new fl::Triangle("far", 0.550,5.000,5.000));

    engNavR->addInputVariable(inRObNavR);

    outLSpdNavR = new fl::OutputVariable;
    outLSpdNavR->setName("leftspdR");
    outLSpdNavR->setRange(-3.500, 3.500);
    outLSpdNavR->setDefaultValue(0.000);
    outLSpdNavR->setLockDefuzzifiedValue(true);
    outLSpdNavR->setDefuzzifier(new fl::Centroid(500));
    outLSpdNavR->output()->setAccumulation(new fl::Maximum);

    outLSpdNavR->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outLSpdNavR->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outLSpdNavR->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outLSpdNavR->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outLSpdNavR->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outLSpdNavR->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavR->addOutputVariable(outLSpdNavR);

    outRSpdNavR = new fl::OutputVariable;
    outRSpdNavR->setName("rightspdR");
    outRSpdNavR->setRange(-3.500, 3.500);
    outRSpdNavR->setDefaultValue(0.000);
    outRSpdNavR->setLockDefuzzifiedValue(true);
    outRSpdNavR->setDefuzzifier(new fl::Centroid(500));
    outRSpdNavR->output()->setAccumulation(new fl::Maximum);

    outRSpdNavR->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outRSpdNavR->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outRSpdNavR->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outRSpdNavR->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outRSpdNavR->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outRSpdNavR->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavR->addOutputVariable(outRSpdNavR);

    rulBlkNavR = new fl::RuleBlock;
    rulBlkNavR->setName("rule_right");
    rulBlkNavR->setTnorm(new fl::Minimum);
    rulBlkNavR->setSnorm(new fl::Maximum);
    rulBlkNavR->setActivation(new fl::Minimum);

    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is near and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is near and rightobR is med then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is near and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is med and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is med and rightobR is med then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is med and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is far and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is far and rightobR is med then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is near and frontobR is far and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is near and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is near and rightobR is med then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is near and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is med and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is med and rightobR is med then leftspdR is posavg and  rightspdR is posavg", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is med and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is far and rightobR is near then leftspdR is negslow and  rightspdR is posslow", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is far and rightobR is med then leftspdR is posavg and  rightspdR is posavg", engNavR));
    rulBlkNavR->addRule(fl::MamdaniRule::parse(
        "if leftobR is far and frontobR is far and rightobR is far then leftspdR is posslow and  rightspdR is negslow", engNavR));

    engNavR->addRuleBlock(rulBlkNavR);
}


void Fuzzy::initFuzNavL()
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In Initialize Left Wall Following, Fuzzy Function.";

    engNavL = new fl::Engine;
    engNavL->setName("left_wall");
    engNavL->addHedge(new fl::Any);
    engNavL->addHedge(new fl::Extremely);
    engNavL->addHedge(new fl::Not);
    engNavL->addHedge(new fl::Seldom);
    engNavL->addHedge(new fl::Somewhat);
    engNavL->addHedge(new fl::Very);

    inRObNavL = new fl::InputVariable;
    inRObNavL->setName("rightobR");
    inRObNavL->setRange(0.000, 5.000);

    inRObNavL->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inRObNavL->addTerm(new fl::Triangle("far", 0.200,5.000,5.000));

    engNavL->addInputVariable(inRObNavL);

    inFObNavL = new fl::InputVariable;
    inFObNavL->setName("frontobR");
    inFObNavL->setRange(0.000, 5.000);

    inFObNavL->addTerm(new fl::Triangle("near", 0.000,0.000,0.300));
    inFObNavL->addTerm(new fl::Trapezoid("med", 0.200,0.350,0.450,0.650));
    inFObNavL->addTerm(new fl::Triangle("far", 0.500,5.000,5.000));

    engNavL->addInputVariable(inFObNavL);

    inLObNavL = new fl::InputVariable;
    inLObNavL->setName("leftobR");
    inLObNavL->setRange(0.000, 5.000);

    inLObNavL->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inLObNavL->addTerm(new fl::Trapezoid("med", 0.300,0.450,0.550,0.650));
    inLObNavL->addTerm(new fl::Triangle("far", 0.550,5.000,5.000));

    engNavL->addInputVariable(inLObNavL);

    outLSpdNavL = new fl::OutputVariable;
    outLSpdNavL->setName("leftspdR");
    outLSpdNavL->setRange(-3.500, 3.500);
    outLSpdNavL->setDefaultValue(0.000);
    outLSpdNavL->setLockDefuzzifiedValue(true);
    outLSpdNavL->setDefuzzifier(new fl::Centroid(500));
    outLSpdNavL->output()->setAccumulation(new fl::Maximum);

    outLSpdNavL->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outLSpdNavL->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outLSpdNavL->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outLSpdNavL->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outLSpdNavL->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outLSpdNavL->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavL->addOutputVariable(outLSpdNavL);

    outRSpdNavL = new fl::OutputVariable;
    outRSpdNavL->setName("rightspdR");
    outRSpdNavL->setRange(-3.500, 3.500);
    outRSpdNavL->setDefaultValue(0.000);
    outRSpdNavL->setLockDefuzzifiedValue(true);
    outRSpdNavL->setDefuzzifier(new fl::Centroid(500));
    outRSpdNavL->output()->setAccumulation(new fl::Maximum);

    outRSpdNavL->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outRSpdNavL->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outRSpdNavL->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outRSpdNavL->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outRSpdNavL->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outRSpdNavL->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavL->addOutputVariable(outRSpdNavL);

    rulBlkNavL = new fl::RuleBlock;
    rulBlkNavL->setName("rule_left");
    rulBlkNavL->setTnorm(new fl::Minimum);
    rulBlkNavL->setSnorm(new fl::Maximum);
    rulBlkNavL->setActivation(new fl::Minimum);

    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is near and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is near and leftobR is med then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is near and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is med and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is med and leftobR is med then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is med and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is far and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is far and leftobR is med then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is near and frontobR is far and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is near and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is near and leftobR is med then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is near and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is med and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is med and leftobR is med then leftspdR is posavg and  rightspdR is posavg", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is med and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is far and leftobR is near then leftspdR is posslow and  rightspdR is negslow", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is far and leftobR is med then leftspdR is posavg and  rightspdR is posavg", engNavL));
    rulBlkNavL->addRule(fl::MamdaniRule::parse(
        "if rightobR is far and frontobR is far and leftobR is far then leftspdR is negslow and  rightspdR is posslow", engNavL));

    engNavL->addRuleBlock(rulBlkNavL);
}


void Fuzzy::initFuzNavObs()
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In Initialize Obstacle Avoidance Fuzzy Function.";

    engNavObs = new fl::Engine;
    engNavObs->setName("obstacle_avoidance");
    engNavObs->addHedge(new fl::Any);
    engNavObs->addHedge(new fl::Extremely);
    engNavObs->addHedge(new fl::Not);
    engNavObs->addHedge(new fl::Seldom);
    engNavObs->addHedge(new fl::Somewhat);
    engNavObs->addHedge(new fl::Very);

    inLObNavObs = new fl::InputVariable;
    inLObNavObs->setName("leftob");
    inLObNavObs->setRange(0.000, 5.000);

    inLObNavObs->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inLObNavObs->addTerm(new fl::Trapezoid("med", 0.300,1.000,2.000,2.500));
    inLObNavObs->addTerm(new fl::Triangle("far", 2.000,5.000,5.000));

    engNavObs->addInputVariable(inLObNavObs);

    inFObNavObs = new fl::InputVariable;
    inFObNavObs->setName("frontob");
    inFObNavObs->setRange(0.000, 5.000);

    inFObNavObs->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inFObNavObs->addTerm(new fl::Trapezoid("med", 0.300,1.000,2.000,2.500));
    inFObNavObs->addTerm(new fl::Triangle("far", 2.000,5.000,5.000));

    engNavObs->addInputVariable(inFObNavObs);

    inRObNavObs = new fl::InputVariable;
    inRObNavObs->setName("rightob");
    inRObNavObs->setRange(0.000, 5.000);

    inRObNavObs->addTerm(new fl::Triangle("near", 0.000,0.000,0.400));
    inRObNavObs->addTerm(new fl::Trapezoid("med", 0.300,1.000,2.000,2.500));
    inRObNavObs->addTerm(new fl::Triangle("far", 2.000,5.000,5.000));

    engNavObs->addInputVariable(inRObNavObs);

    outLSpdNavObs = new fl::OutputVariable;
    outLSpdNavObs->setName("leftspd");
    outLSpdNavObs->setRange(-3.500, 3.500);
    outLSpdNavObs->setDefaultValue(0.000);
    outLSpdNavObs->setLockDefuzzifiedValue(true);
    outLSpdNavObs->setDefuzzifier(new fl::Centroid(500));
    outLSpdNavObs->output()->setAccumulation(new fl::Maximum);

    outLSpdNavObs->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outLSpdNavObs->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outLSpdNavObs->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outLSpdNavObs->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outLSpdNavObs->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outLSpdNavObs->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavObs->addOutputVariable(outLSpdNavObs);

    outRSpdNavObs = new fl::OutputVariable;
    outRSpdNavObs->setName("rightspd");
    outRSpdNavObs->setRange(-3.500, 3.500);
    outRSpdNavObs->setDefaultValue(0.000);
    outRSpdNavObs->setLockDefuzzifiedValue(true);
    outRSpdNavObs->setDefuzzifier(new fl::Centroid(500));
    outRSpdNavObs->output()->setAccumulation(new fl::Maximum);

    outRSpdNavObs->addTerm(new fl::Triangle("neghigh", -3.500,-3.500,-1.500));
    outRSpdNavObs->addTerm(new fl::Trapezoid("negavg", -1.750,-1.500,-1.000,-0.750));
    outRSpdNavObs->addTerm(new fl::Triangle("negslow", -1.000,0.000,0.000));
    outRSpdNavObs->addTerm(new fl::Triangle("posslow", 0.000,0.000,1.000));
    outRSpdNavObs->addTerm(new fl::Trapezoid("posavg", 0.750,1.000,1.500,1.750));
    outRSpdNavObs->addTerm(new fl::Triangle("poshigh", 1.500,3.500,3.500));

    engNavObs->addOutputVariable(outRSpdNavObs);

    rulBlkNavObs = new fl::RuleBlock;
    rulBlkNavObs->setName("rule_obs");
    rulBlkNavObs->setTnorm(new fl::Minimum);
    rulBlkNavObs->setSnorm(new fl::Maximum);
    rulBlkNavObs->setActivation(new fl::Minimum);

    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is near and rightob is near then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is near and rightob is med then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is near and rightob is far then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is med and rightob is near then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is med and rightob is med then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is med and rightob is far then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is far and rightob is near then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is far and rightob is med then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is near and frontob is far and rightob is far then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is near and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is near and rightob is med then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is near and rightob is far then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is med and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is med and rightob is med then leftspd is posavg and rightspd is posavg", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is med and rightob is far then leftspd is poshigh and rightspd is poshigh", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is far and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is far and rightob is med then leftspd is poshigh and rightspd is poshigh", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is med and frontob is far and rightob is far then leftspd is poshigh and rightspd is poshigh", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is near and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is near and rightob is med then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is near and rightob is far then leftspd is posslow and rightspd is negslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is med and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is med and rightob is med then leftspd is posavg and rightspd is posavg", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is med and rightob is far then leftspd is poshigh and rightspd is poshigh", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is far and rightob is near then leftspd is negslow and rightspd is posslow", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is far and rightob is med then leftspd is poshigh and rightspd is poshigh", engNavObs));
    rulBlkNavObs->addRule(fl::MamdaniRule::parse(
        "if leftob is far and frontob is far and rightob is far then leftspd is poshigh and rightspd is poshigh", engNavObs));

    engNavObs->addRuleBlock(rulBlkNavObs);
}


void Fuzzy::setFuzInNav(double right, double front, double left, double tempx, double tempy)
{
    double rsp, lsp;

    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> In SetFuzInNav Function.";
        qDebug()<<"<<<FZ>>> Input Raw Parameters In SetFuzInNav Function, Right, Front, Left: "<<right<<front<<left;
    }

    spdsNav.clear();

    // This is For Type of Activation Fuzzy Function
    // I Can Remove This Part By Change Activation Fuzzy Function
    // I Change Activation Function But There is yet This Problem
    if ( right == 5.0 )
        right -= 0.2;

    if ( front == 5.0 )
        front -= 0.2;

    if ( left == 5.0 )
        left -= 0.2;

    if ( right == 0 )
        right += 0.1;

    if ( front == 0 )
        front += 0.1;

    if ( left == 0 )
        left += 0.1;

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Input Parameters In SetFuzInNav Function After Modification, Right, Front, Left:"<<right<<front<<left;

//    // Check That This Position Is a Repeated Position Or Not
//    if ( !chkReptedPos(tempx,tempy) )
//    {
//        if ( FZDebug )
//            qDebug()<<"<<<FZ>>> Robot Is In A New Path.";

//        flagRepPos = true;
//    }
//    else
//    {
//        if ( FZDebug )
//            qDebug()<<"<<<FZ>>> Robot Is In An Old Path.";

//        flagRepPos = false;
//    }

//    if ( FZDebug )
//        qDebug()<<"<<<FZ>>> Status of Flags, flagConstPos, flagRepPos: "<<flagConstPos<<flagRepPos;

//    if ( posStop(tempx,tempy) & flagConstPos & flagRepPos )
//    {
//        if ( FZDebug )
//            qDebug()<<"<<<FZ>>> Output Of posStop Function Is False.";

//        if ( wallNavDir == 1 ) // Right Wall Following
//        {
//            if ( FZDebug )
//                qDebug()<<"<<<FZ>>> Direction of Wall Following Is Right(1), wallNavDir: "<<wallNavDir;

//            // Fuzzy Navigation For Right Wall Following
//            inLObNavR->setInput(left);
//            inFObNavR->setInput(front);
//            inRObNavR->setInput(right);

//            engNavR->process();

//            lsp = outLSpdNavR->defuzzify();
//            rsp = outRSpdNavR->defuzzify();
//            // Right Wall Following

//            setFuzzyKind(QString("right"));
//        }
//        else if ( wallNavDir == -1 ) // Left Wall Following
//        {
//            if ( FZDebug )
//                qDebug()<<"<<<FZ>>> Direction of Wall Following Is Left(-1), wallNavDir: "<<wallNavDir;

//            // Fuzzy Navigation For Left Wall Following
//            inLObNavL->setInput(left);
//            inFObNavL->setInput(front);
//            inRObNavL->setInput(right);

//            engNavL->process();

//            lsp = outLSpdNavL->defuzzify();
//            rsp = outRSpdNavL->defuzzify();
//            // Left Wall Following

//            setFuzzyKind(QString("left"));
//        }
//    }
//    else
//    {
//        if ( FZDebug )
//            qDebug()<<"<<<FZ>>> Robot Has Constant Position Or Is Not Near To Wall.";

//        if ( ( right <= 0.6 || left <= 0.6 ) & flagRepPos )
//        {
//            if ( FZDebug )
//                qDebug()<<"<<<FZ>>> Robot Reachs To Wall.";

//            flagConstPos = true;

//            spdsNav.append(0);
//            spdsNav.append(0);

//            return;
//        }
//        else
//        {
//            if ( FZDebug )
//                qDebug()<<"<<<FZ>>> Robot Doesn't Reach To Wall.";

//            flagConstPos = false;
//        }

        //if ( FZDebug )
        inLObNavObs->setInput(left);
        inFObNavObs->setInput(front);
        inRObNavObs->setInput(right);

        engNavObs->process();

        lsp = outLSpdNavObs->defuzzify();
        rsp = outRSpdNavObs->defuzzify();

        setFuzzyKind(QString("obsavoid"));
   // }

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Calculated Speeds In SetFuzInNav, Right, Left: "<<rsp<<lsp;

    if ( rsp < 0 & rsp > -0.1 )
        rsp = -0.1;

    if ( lsp < 0 & lsp > -0.1 )
        lsp = -0.1;

    if ( rsp >= 0 & rsp < 0.1 )
        rsp = 0.1;

    if ( lsp >= 0 & lsp < 0.1 )
        lsp = 0.1;

    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> Calculated Speeds In SetFuzInNav After Modification, Right, Left: "<<rsp<<lsp;
        qDebug()<<"<<<FZ>>> Last Speeds Are, Right, Left: "<<lastSpd[0]<<lastSpd[1];
    }

//    // If Last Speeds And New Speed Turn in Opposite Direction I Set Last Speeds
//    if ( lastSpd[0] < 0 & lastSpd[1] > 0 & rsp > 0 & lsp < 0 )
//    {
//        if ( FZDebug )
//        {
//            qDebug()<<"<<<FZ>>> lastSpd[0] < 0 & lastSpd[1] > 0 & rsp > 0 & lsp < 0";

//            qDebug()<<"<<<FZ>>> Last Speeds Is Seted.";
//        }

//        rsp = lastSpd[0];
//        lsp = lastSpd[1];
//    }
//    else if ( lastSpd[0] > 0 & lastSpd[1] < 0 & rsp < 0 & lsp > 0 )
//    {
//        if ( FZDebug )
//        {
//            qDebug()<<"<<<FZ>>> lastSpd[0] > 0 & lastSpd[1] < 0 & rsp < 0 & lsp > 0";

//            qDebug()<<"<<<FZ>>> Last Speeds Is Seted.";
//        }

//        rsp = lastSpd[0];
//        lsp = lastSpd[1];
//    }

    // Final Speeds Is Seted
    spdsNav.append(rsp);
    spdsNav.append(lsp);

    lastSpd.clear();

    // Append Last Speeds
    lastSpd.append(rsp);
    lastSpd.append(lsp);

    // Set Current Position In All Position
    setAllPos(tempx,tempy);

    // Set Current Position To Last Position
    lastPosRep.setX(round(tempx));
    lastPosRep.setY(round(tempy));
}


// Getter Speeds of Fuzzy Navigation, [0] for Right Wheel and [1] for Left Wheel
QList<double> Fuzzy::getFuzOutNav()
{
    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> In getFuzOutNav Function.";
        qDebug()<<"<<<FZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZFZ";
    }

    return spdsNav;
}


// In This Function I Get Laser Data and Extract Fuzzy Inputs
void Fuzzy::extractFuzIn(QList<double> laserTemp, double x, double y)
{
    // I Set Max Range of Laser To 5m, And I Don't Use 10m, Because With 10m Robot Considers Much Area That is Not Required
    // I Consider Just From -1.9 Degree To 1.9 Degree, Because More Than This Angle Cause That Robot Considers Some Back Area
    // So I should Delete 60 Beams Laser From Start and End
    // So I Delete 0 - 59 (60) and 499 - 60 = 439 (60), Start is 60 and End is 439
    // For Recognizing Left and Right Obstacle, I Consider 1.4 Degree and For Front I Consider 1 Degree
    // For Recognizing Right Obstcales The Angle is From -1.9 To -0.5 (1.4 Degree)( 60 To 199)(140 Beams)
    // For Recognizing Front Obstcales The Angle is From -0.5 To  0.5 (1 Degree  )(200 To 299)(100 Beams)
    // For Recognizing Left  Obstacles The Angle is From  0.5 To  1.9 (1.4 Degree)(300 To 439)(140 Beams)

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In Extarct Fuzzy Inputs.";

    int             i   = 0;
    double          min = 5.0, sum = 0.0;
    QList<double>   tempFuzIn;

    tempFuzIn.clear();

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Size of Raw Laser Data Is: "<<laserTemp.size();

    // Set Laser Beam Greater Than 10 To 5
    for ( i = 0 ; i < laserTemp.size() ; i++ )
        if ( laserTemp[i] > 5 )
            laserTemp.replace(i,5.0);

    // Calculate Right Fuzzy Input
    // Find Minimum of Laser Beams For Right Fuzzy Input
    for ( i = 60 ; i <= 199 ; i++ )
        if ( laserTemp[i] < min )
            min = laserTemp[i];

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Minimum of Right Obstacle is: "<<min;

    // I Consider 1.0 Based On Exprimental Resualt
    if ( min <= MINTHERESHOLD )
        tempFuzIn.append(min);
    else
    {
        for ( i = 60 ; i <= 199 ; i++ )
            sum = sum + laserTemp[i];

        tempFuzIn.append(sum/140);
    }

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Value of Right Obstacle is: "<<tempFuzIn[0];

    // Calculate Front Fuzzy Input
    sum = 0.0;
    min = 5.0;

    // Find Minimum of Laser Beams For Front Fuzzy Input
    for ( i = 200 ; i <= 299 ; i++ )
        if ( laserTemp[i] < min )
            min = laserTemp[i];

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Minimum of Front Obstacle is: "<<min;

    if ( min <= MINTHERESHOLD )
        tempFuzIn.append(min);
    else
    {
        for ( i = 200 ; i <= 299 ; i++ )
            sum = sum + laserTemp[i];

        tempFuzIn.append(sum/199);
    }

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Value of Front Obstacle is: "<<tempFuzIn[1];

    // Calculate Left Fuzzy Input
    sum = 0.0;
    min = 5.0;

    // Find Minimum of Laser Beams For Left Fuzzy Input
    for ( i = 300 ; i <= 439 ; i++ )
        if ( laserTemp[i] < min )
            min = laserTemp[i];

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Minimum of Left Obstacle is: "<<min;

    if ( min <= MINTHERESHOLD )
        tempFuzIn.append(min);
    else
    {
        for ( i = 300 ; i <= 439 ; i++ )
            sum = sum + laserTemp[i];

        tempFuzIn.append(sum/140);
    }

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Value of Left Obstacle is: "<<tempFuzIn[2];

    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Calculated Fuzzy Input, Right, Front, Left: "<<tempFuzIn[0]<<tempFuzIn[1]<<tempFuzIn[2];

    // First Parameter is Right, Second is Front, Last One is Left
    setFuzInNav(tempFuzIn[0], tempFuzIn[1], tempFuzIn[2], x, y);
}


// Checking That Robot Is In Stop
bool Fuzzy::posStop(double tempx, double tempy)
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In posStop Function.";

    QPoint tmp_pos;
    tmp_pos.setX(round(tempx));
    tmp_pos.setY(round(tempy));

    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> Original Current Position Is, X, Y: "<<tempx<<tempy;
        qDebug()<<"<<<FZ>>> Rounded Original Current Position Is, X, Y: "<<round(tempx)<<round(tempy);
        qDebug()<<"<<<FZ>>> Last Position Is, X, Y: "<<lastPos.x()<<lastPos.y();
        qDebug()<<"<<<FZ>>> Value of CountStop Is, CountStop: "<<countStop;
    }

    if ( tmp_pos.x() == lastPos.x() & tmp_pos.y() == lastPos.y() )
    {
        countStop++;

        if ( FZDebug )
            qDebug()<<"<<<FZ>>> Current Pos Is Equal With Last Pos, Add to CountStop, CountStop: "<<countStop;
    }
    else
    {
        lastPos.setX(tmp_pos.x());
        lastPos.setY(tmp_pos.y());

        countStop = 0;

        if ( FZDebug )
            qDebug()<<"<<<FZ>>> Current Pos Is NOT Equal With Last Pos, CountStop Is Seted To Zero: "<<countStop;
    }

    if ( countStop == THERREPEATPOS )
    {
        countStop = 0;

        if ( FZDebug )
            qDebug()<<"<<<FZ>>> CountStop == THERREPEATPOS, Return False.";

        return false;
    }
    else
    {
        if ( FZDebug )
            qDebug()<<"<<<FZ>>> CountStop != THERREPEATPOS, Return True.";

        return true;
    }
}


void Fuzzy::setNavDir(int dir)
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> Wall Following Direction Is Seted To: "<<dir;

    wallNavDir = dir;
}


// Keep All Meeted Position In a List
void Fuzzy::setAllPos(double tempx, double tempy)
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In setAllPos Function.";

    QPoint tmp;
    tmp.setX(round(tempx));
    tmp.setY(round(tempy));

    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> Current Position Is, X, Y: "<<round(tempx)<<round(tempy);
        qDebug()<<"<<<FZ>>> All Meet Positions Are: ";

        for ( int i = 0 ; i < allPos.size() ; i++ )
            qDebug()<<"\t"<<allPos[i];
    }

    if ( !allPos.contains(tmp) )
    {
        if ( FZDebug )
            qDebug()<<"<<<FZ>>> This Position Is a New Position.";

        allPos.append(tmp);
    }
    else
    {
        if ( FZDebug )
            qDebug()<<"<<<FZ>>> This Position Is NOT a New Position.";
    }

    if ( FZDebug )
    {
        qDebug()<<"<<<FZ>>> allPos After Adding New Position.";

        for ( int i = 0 ; i < allPos.size() ; i++ )
            qDebug()<<"\t"<<allPos[i];
    }
}


// Checking That Robot Is In Metted Position Or Not
bool Fuzzy::chkReptedPos(double tempx, double tempy)
{
    if ( FZDebug )
        qDebug()<<"<<<FZ>>> In chkReptedPos Function.";

    int temp;

    QPoint tmp;
    tmp.setX(round(tempx));
    tmp.setY(round(tempy));

    if ( allPos.contains(tmp) )
    {
        if ( FZDebug )
        {
            qDebug()<<"<<<FZ>>> lastPosRep Is, X, Y: "<<lastPosRep;
            qDebug()<<"<<<FZ>>> Current Position Is, X, Y: "<<tmp;
        }

        temp = allPos.size();

        if ( temp >= 1 & tmp.x() == allPos[temp-1].x() & tmp.y() == allPos[temp-1].y() )
        {
            if ( FZDebug )
                qDebug()<<"<<<FZ>>> This Position Is Not Repeated Position.";

            return  false;
        }
        else
        {
            if (temp > 2 & tmp.x() == allPos[temp-2].x() & tmp.y() == allPos[temp-2].y() )
            {
                if ( FZDebug )
                    qDebug()<<"<<<FZ>>> This Position Is Not Repeated Position.";

                return  false;
            }
            else
            {
                if ( FZDebug )
                    qDebug()<<"<<<FZ>>> This Position Is Repeated Position.";

                return true;
            }
        }
    }
    else
    {
        if ( FZDebug )
            qDebug()<<"<<<FZ>>> This Position Is Not Repeated Position.";

        return  false;
    }
}


void Fuzzy::setFuzzyKind(QString kind)
{
    fuzKind = kind;
}


QString Fuzzy::getFuzzyKind()
{
    return fuzKind;
}
