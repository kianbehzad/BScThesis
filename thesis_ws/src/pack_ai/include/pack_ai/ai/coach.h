//
// Created by Kian Behzad on 3/13/21.
//

#ifndef PACK_AI_COACH_H
#define PACK_AI_COACH_H

#include "pack_ai/ai/extern_variables.h"

#include <QDebug>

class Coach
{
public:
    Coach();
    ~Coach();
    void execute();

private:
    // gets the desired id and returns the index corresponding to the id in wm.our
    int ID(int id);
};

#endif //PACK_AI_COACH_H
