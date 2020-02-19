#include "Manage.hh"


int main()
{
    Manage mg;
    mg.start();

    return 0;
}


//    fl::Engine* engine = new fl::Engine("simple-dimmer");//

//    fl::InputVariable* ambientLight = new fl::InputVariable;//
//    ambientLight->setName("AmbientLight");//
//    ambientLight->setRange(0.000, 1.000);//

//    ambientLight->addTerm(new fl::Triangle("LOW", 0.000,0.250,0.500));
//    ambientLight->addTerm(new fl::Triangle("MEDIUM", 0.250,0.500,0.750));
//    ambientLight->addTerm(new fl::Triangle("HIGH", 0.500,0.750,1.000));
//    engine->addInputVariable(ambientLight);//

//    fl::OutputVariable* bulbPower = new fl::OutputVariable;//
//    bulbPower->setName("BulbPower");//
//    bulbPower->setRange(0.000, 2.000);//
//    bulbPower->setDefaultValue(0);//

//    bulbPower->addTerm(new fl::Triangle("LOW", 0.000, 0.500, 1.000));
//    bulbPower->addTerm(new fl::Triangle("MEDIUM", 0.500, 1.000, 1.500));
//    bulbPower->addTerm(new fl::Triangle("HIGH", 1.000, 1.500, 2.000));
//    engine->addOutputVariable(bulbPower);//

//    fl::RuleBlock* ruleblock = new fl::RuleBlock;//
//    ruleblock->addRule(fl::MamdaniRule::parse("if AmbientLight is LOW then BulbPower is LOW", engine));
//    ruleblock->addRule(fl::MamdaniRule::parse("if AmbientLight is MEDIUM then BulbPower is MEDIUM", engine));
//    ruleblock->addRule(fl::MamdaniRule::parse("if AmbientLight is HIGH then BulbPower is HIGH", engine));
//    engine->addRuleBlock(ruleblock);//

//    engine->configure("Minimum", "Maximum", "AlgebraicProduct", "AlgebraicSum", "Centroid");//

//    for ( double i = 0.0 ; i <= 1.0 ; i+=0.1)
//    {
//    ambientLight->setInput(i);
//    engine->process();
//    double x = bulbPower->defuzzify();
//    qDebug() <<i<<"====="<<x;
//}
////    for (fl::scalar in = 0.0 ; in < 1.1 ; in += 0.1)
////    {
////            energy->setInput(in);
////            engine.process();
////            fl::flScalar out = health->output().defuzzify();
////            FL_LOG("Energy=" << in);
////            FL_LOG("Energy is " << energy->fuzzify(in));
////            FL_LOG("Health=" << out);
////            FL_LOG("Health is " << health->fuzzify(out));
////            FL_LOG("--");
////    }
