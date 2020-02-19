#ifndef _DEBUG_HH_
#define _DEBUG_HH_


class Debug
{
    public:
        Debug()
        {
            LADebug = true;
            FZDebug = false;
            MGDebug = false;
        };

        bool LADebug;   // Learning Automata Class
        bool FZDebug;   // Fuzzy  Class
        bool MGDebug;   // Manage Class
};


#endif  //  _DEBUG_HH_

