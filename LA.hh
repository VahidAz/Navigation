#ifndef _LA_HH_
#define _LA_HH_


#include "CommonHeaders.hh"
#include "Debug.hh"


class LA:public Debug
{
  public:
    LA();

    void setActiveActs(QString,QString);    // Set Two Active Acts in Each Selection For LA
    int  getLAResult  ();                   // Return Result of LA
    void updateProb   (int,int,double);     // Update Probability of Acts

  private:
    QList<QString> allActs; // All Acts In LA

    QList<double> orgProb;  // Keep Probability of All Acts

    QString acAct1, // First Active Act
            acAct2, // Second Active Act
            resAct; // Resulted act

    double punish,  // Punish Value In LA Algorithm
           reward;  // Reward Value In LA Algorithm

    int resultAct;  // Keep Resualt Act


    double kn;      // Keep Multiple of Probability of Two Active Acts
};


#endif // _LA_HH_
