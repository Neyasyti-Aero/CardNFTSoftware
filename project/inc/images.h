#ifndef __IMAGES_H
#define __IMAGES_H

#include "main.h"
#include "display.h"
#include "gif.h"

// Use them as ShowImage() arguments
#define IMAGE2 0
#define IMAGE3 1
#define IMAGE4 2
#define IMAGE5 3
#define IMAGE6 4
#define IMAGE7 5
#define LAST_IMAGE IMAGE7

extern uint8_t g_bShowImageRequest;
extern uint8_t g_iCurrentImage;

void ShowImage(uint8_t image);
void NextImage(void);
void PreviousImage(void);

#endif
