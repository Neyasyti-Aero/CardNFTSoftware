#ifndef __GIF_H
#define __GIF_H

#include "main.h"
#include "display.h"
#include "images.h"

extern uint8_t g_bGIFPlaying;
extern uint8_t g_bGIFStartRequest;
extern uint8_t g_bGIFStopRequest;

void PlayGif(void);
void ShowFrame(uint8_t frame);

#endif
