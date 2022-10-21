#include "init.h"
#include <iostream>

std::uint16_t SCREEN_WIDTH = 10;
std::uint16_t SCREEN_HEIGHT = 10;
std::uint16_t converted_screen_width;
std::uint16_t converted_screen_height;




std::array<uint8_t, pixel_count> PIXELS;

void setPixel(std::uint16_t x, std::uint16_t y, std::uint8_t color) {
    int i = (y * converted_screen_width) + x;
    PIXELS[i] = color;
}

void clearScreen(std::uint8_t color) {
    std::uint16_t index = 0;
    for(auto i : PIXELS) {
        PIXELS[index] = color;
        i = color;
        index++;
    }
}

// Utility macros
#define CHECK_ERROR(test, message) \
    do { \
        if((test)) { \
            fprintf(stderr, "%s\n", (message)); \
            exit(1); \
        } \
    } while(0)





int init(std::uint16_t screen_width, std::uint16_t screen_height, std::uint8_t pixel_size) {
    

    std::array<uint8_t, 4> pallette = {0x00, 0x60, 0xC0, 0xFF};

    converted_screen_width = screen_width;

    SCREEN_WIDTH = screen_width*pixel_size;
    SCREEN_HEIGHT = screen_height*pixel_size;

    for (auto &i : PIXELS) i = 0x00;

    std::uint32_t pixelnum = 0;
    SDL_Rect pxrect;
    pxrect.x = 0;
    pxrect.y = 0;
    pxrect.w = pixel_size;
    pxrect.h = pixel_size;

    

    std::cout << "INITIALIZING\n";
    
    // Initialize SDL
    SDL_Init(SDL_INIT_VIDEO);

    // Create an SDL window
    SDL_Window *window = SDL_CreateWindow("ComputerEmu", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL);

    // Create a renderer (accelerated and in sync with the display refresh rate)
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);    



    // Initial renderer color
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);


    bool running = true;
    SDL_Event event;

    while(running) {
        // Process events
        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_QUIT) {
                running = false;
            } else {
                onEvent(event);
            }

        }
        SDL_RenderClear(renderer);



        update();
        pxrect.x = 0;
        pxrect.y = 0;
        pixelnum = 0;






        /* color of pixel
        pixel value = 0b00000000
        R = value & 0b11000000
        G = value & 0b00110000
        B = value & 0b00001100
        A = value & 0b

        values
        00 = 1
        01 = 2
        02 = 3
        03 = 4

        all values multiplied by 64

        */

        for (auto &i : PIXELS) {
            if((i & 0b00000011) != 0) {
                std::uint8_t r, g, b, a, p;
                p = i;
                a = ( p & 0b00000011 ); // take last 2 values
                p >>= 2;                // shift out last 2 values
                b = ( p & 0b00000011 );
                p >>= 2;
                g = ( p & 0b00000011 );
                p >>= 2;
                r = ( p & 0b00000011 );






                pxrect.x = (pixelnum % screen_width) * pixel_size;
                pxrect.y = (pixelnum / screen_width) * pixel_size;

                SDL_SetRenderDrawColor(renderer, pallette[r], pallette[g], pallette[b], pallette[a]);
                SDL_RenderFillRect(renderer, &pxrect);
            }
            pixelnum++;
        }





        pxrect.x += 32;



        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

        // Show what was drawn
        SDL_RenderPresent(renderer);
    }

    program_end();

    // Release resources
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "ENDING...\n";
    return 0;
}