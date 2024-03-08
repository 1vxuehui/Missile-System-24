#ifndef UITASK_H
#define UITASK_H
#include "main.h"


extern void UI_task(void const *pvParameters);
extern void UIRefreshTask_Loop(void);
void UIRefreshTask_Setup(void);
#endif
