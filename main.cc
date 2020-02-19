#include "Manage.hh"


int main()
{
    Manage mg;
    mg.start();

    return 0;
}


// turn to maghsad va moving mitunam mecanize konam na dasti ( hal kardam!!!! )
// robot ba az do mete jabejaie shoru mikone be peyda kradan wall haye rangi
// moshkeli ke robot moghe bargasht az hamun masir mojaeh mishe
// age robot wallo az dast bede moghe lern shodam robot motevajeh nemishe ( evalo tamum mikonim)
// age hein learning ye masire jadid peyda kard chi kar konam (nemizarim peyda kone)
// ax to ye otagh nazane!!!!

// i Should write position control in fuzzy class for avoiding duplicate paths

// image proccessing
//mapping
//optimize fuzzy hal shod
// avoid repeat hal shod
//align to target




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
