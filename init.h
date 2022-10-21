#pragma once
#include <SDL2/SDL.h>
#include <iostream>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <SDL2/SDL_rect.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_keyboard.h>

#include <array>


const std::uint32_t pixel_count = 256;
// MAKE SURE THIS IS EQUAL TO YOUR SCREEN WIDTH TIMES YOUR SCREEN HEIGHT OR GRAPHICS WONT WORK




int update();

void program_end();

int init(std::uint16_t screen_width, std::uint16_t screen_height, std::uint8_t pixel_size);

void setPixel(std::uint16_t x, std::uint16_t y, std::uint8_t color);
void clearScreen(std::uint8_t color = 0x00);

void onEvent(SDL_Event event);



