//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach() = default;

Coach::~Coach() = default;

void Coach::execute()
{
    qDebug() << ID(3) << " - " << extern_wm->our[ID(3)].id;
}

int Coach::ID(int id)
{
    for(int i{}; i < static_cast<int>(extern_wm->our.size()); i++)
        if (extern_wm->our[i].id == id)
            return i;
    return -1;
}
