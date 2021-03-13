//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach() = default;

Coach::~Coach() = default;

void Coach::execute()
{
    static int cnt;
    if (cnt < 60*4)
        extern_skill_handler->gotopoint_avoid(1, rcsc::Vector2D{4, 0}, rcsc::Vector2D{}.invalidate(), true);

    else if(cnt < 60*8)
        extern_skill_handler->gotopoint_avoid(1, rcsc::Vector2D{-4, 0}, rcsc::Vector2D{}.invalidate(), true);

    else
        cnt = 0;

    cnt++;
}
