#include "LA.hh"


LA::LA():Debug()
{
    if ( LADebug )
        qDebug()<<"<<<LA>>> In Constructor of LA.";

    allActs.clear();

    allActs.append("BLUE");
    allActs.append("WHITE");
    allActs.append("YELLOW");

    if ( LADebug )
    {
        qDebug()<<"<<<LA>>> All Acts Are: ";

        for ( int i = 0 ; i < allActs.size() ; i++ )
            qDebug()<<"\t"<<allActs[i];
    }

    orgProb.clear();

    for ( int i = 0 ; i < allActs.size() ; i++ )
        orgProb.append( (double)1 / allActs.size());

    if ( LADebug )
    {
        qDebug()<<"<<<LA>>> Original Probability Are: ";

        for ( int i = 0 ; i < orgProb.size() ; i++ )
            qDebug()<<"\t"<<orgProb[i];
    }

    punish = 0.001;

    reward = 0.999;

}


// Set Two Active Acts For Running LA
void LA::setActiveActs(QString act1, QString act2)
{
    if ( LADebug )
        qDebug()<<"LALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALALLALALALALALLALALALALALALALALALALALALALALALALAL";

    int randNum, i , selAct;

    if ( LADebug )
    {
        qDebug()<<"<<<LA>>> In SetActiveActs Function.";

        qDebug()<<"<<<LA>>> Two Seted Active Acts Are: "<<act1<<act2;

        qDebug()<<"<<<LA>>> Orginal Probability Before Change.";

        for ( i = 0 ; i < orgProb.size() ; i++ )
            qDebug()<<"\t"<<allActs[i]<<orgProb[i];
    }

    double sumProbActiveAct = orgProb[allActs.indexOf(act1)] + orgProb[allActs.indexOf(act2)];

    kn = sumProbActiveAct;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Sum Probability of Two Active Acts Is: "<<sumProbActiveAct;

    orgProb[allActs.indexOf(act1)] = orgProb[allActs.indexOf(act1)] / sumProbActiveAct;
    orgProb[allActs.indexOf(act2)] = orgProb[allActs.indexOf(act2)] / sumProbActiveAct;

    if ( LADebug )
    {
        qDebug()<<"<<<LA>>> Modified Probability of All Action Are: ";

        for ( i = 0 ; i < orgProb.size() ; i++ )
            qDebug()<<"\t"<<allActs[i]<<orgProb[i];
    }

    // Select a Random Act Based on Probility of those
    // Set Seed to Rand Function
    srand(time(NULL));

    // Generate a Random Number Between [1 100]
    randNum = rand() % 100 + 1;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Rand Number In [1 100]: "<<randNum;

    if ( randNum > 0 & randNum <= ( orgProb[allActs.indexOf(act1)] * 100 ) )
    {
        selAct = 1;
        resAct = act1;

        if ( LADebug )
            qDebug()<<"<<<LA>>> Selected Act Is: "<<act1;
    }
    else if ( randNum > ( orgProb[allActs.indexOf(act1)] * 100 ) & randNum <= ( orgProb[allActs.indexOf(act1)] + orgProb[allActs.indexOf(act2)] ) * 100 )
    {
        selAct = 2;
        resAct = act2;

        if ( LADebug )
            qDebug()<<"<<<LA>>> Selected Act Is: "<<act2;
    }

    resultAct = selAct;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Selected Act Is: "<<resultAct;

    acAct1 = act1;
    acAct2 = act2;
}


// Return Resulted Act
int LA::getLAResult()
{
    if ( LADebug )
        qDebug()<<"<<<LA>>> In getLAResulted Function.";

    return resultAct;
}


void LA::updateProb(int time, int vic, double dist)
{
    double score;
    int i, vict;

    if ( LADebug )
    {
        qDebug()<<"<<<LA>>> In UpdateProb Function.";
        qDebug()<<"<<<LA>>> Time, Victim, Distance: "<<time<<vic<<dist;
    }

//    if ( vic > 10 )
//    {
//        if ( LADebug )
//            qDebug()<<"<<<LA>>> Number of Victims Is greater Than 5, So Set Number of Victim To 5.";

//        vic = 10;
//    }

//    if ( dist > 10 )
//    {
//        if ( LADebug )
//            qDebug()<<"<<<LA>>> Distance is Greater Than 5, So Seted To 5.";

//        dist = 10;
//    }

    if ( resAct == "BLUE" )
        vict = 1;
    else if ( resAct == "YELLOW" )
        vict = 3;
    else if ( resAct == "WHITE" )
        vict = 0;

    //score = ( 1 - ( time / 500 ) ) + ( vic / 5 );// + ( dist / 10 );
    score = (double)vict / 3;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Original Score is: "<<score;

    //score = score / 2;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Score in 1 Scale: "<<score;

    // Convert Score To Punishment
    score = 1 - score;

    if ( LADebug )
        qDebug()<<"<<<LA>>> Score in Punishment Scale Is: "<<score;

    if ( LADebug )
        for ( i = 0 ; i < orgProb.size() ; i++ )
            qDebug()<<"<<<LA>>> Probability Before Change: "<<allActs[i]<<orgProb[i];

    // Update Probability of Selected Act
    orgProb.replace(allActs.indexOf(resAct),orgProb[allActs.indexOf(resAct)] +
            ( reward * (1-score) * (1-orgProb[allActs.indexOf(resAct)]))-
            ( punish * score * orgProb[allActs.indexOf(resAct)]));

    if ( LADebug )
        qDebug()<<"<<<LA>>> Probability of Resualt Act: "<<orgProb[allActs.indexOf(resAct)];

    if ( LADebug )
        qDebug()<<"<<<LA>>> Kn Value Is: "<<kn;

    // Multiple Probability of Selected Act In Sum of Probabilitis of Two Active Acts
    orgProb.replace(allActs.indexOf(resAct), orgProb[allActs.indexOf(resAct)] * kn);

    if ( LADebug )
        qDebug()<<"<<<LA>>> Probability of Resualt Act After Multiple in KN: "<<resAct<<orgProb[allActs.indexOf(resAct)];

    // Update Probability of Another Active Act
    if ( resAct == acAct1 )
    {
        orgProb.replace(allActs.indexOf(acAct2), orgProb[allActs.indexOf(acAct2)] -
                (reward*(1-score)*orgProb[allActs.indexOf(acAct2)]) +
                (punish*score*( (1/(2-1)) - orgProb[allActs.indexOf(acAct2)])));

        if ( LADebug )
            qDebug()<<"<<<LA>>> Probability of Selected Acts: "<<acAct2<<orgProb[allActs.indexOf(acAct2)];

        orgProb.replace(allActs.indexOf(acAct2), orgProb[allActs.indexOf(acAct2)] * kn);

        if ( LADebug )
            qDebug()<<"<<<LA>>> Probability of Selected Acts After Multiple In KN: "<<acAct2<<orgProb[allActs.indexOf(acAct2)];
    }
    else if ( resAct == acAct2 )
    {
        orgProb.replace(allActs.indexOf(acAct1), orgProb[allActs.indexOf(acAct1)] -
                (reward*(1-score)*orgProb[allActs.indexOf(acAct1)]) +
                (punish*score*( (1/(2-1)) - orgProb[allActs.indexOf(acAct1)])));

        if ( LADebug )
            qDebug()<<"<<<LA>>> Probability of Selected Acts: "<<acAct1<<orgProb[allActs.indexOf(acAct1)];

        orgProb.replace(allActs.indexOf(acAct1), orgProb[allActs.indexOf(acAct1)] * kn);

        if ( LADebug )
            qDebug()<<"<<<LA>>> Probability of Selected Acts After Multiple In KN: "<<acAct1<<orgProb[allActs.indexOf(acAct1)];
    }
}
