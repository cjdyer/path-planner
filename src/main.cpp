#include <iostream>
#include <cstring>
#include <memory>
#include <cmath>
#include <thread>

#include <SDL3/SDL.h>

#include "types.h"
#include "config.h"
#include "path_planner.h"

std::unique_ptr<SDL_Window, void (*)(SDL_Window *)> setup_window(const Config &config)
{
    SDL_Window *window = SDL_CreateWindow("SDL Renderer", config.window_dimensions.x, config.window_dimensions.y, 0);
    if (!window)
    {
        std::cout << "SDL Window Creation Failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        throw std::runtime_error("SDL Window Creation Failed");
    }
    return std::unique_ptr<SDL_Window, void (*)(SDL_Window *)>(window, SDL_DestroyWindow);
}

std::unique_ptr<SDL_Renderer, void (*)(SDL_Renderer *)> setup_renderer(SDL_Window *window)
{
    SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer)
    {
        std::cout << "SDL Renderer Creation Failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        throw std::runtime_error("SDL Renderer Creation Failed");
    }
    return std::unique_ptr<SDL_Renderer, void (*)(SDL_Renderer *)>(renderer, SDL_DestroyRenderer);
}

int main()
{
    // Init Components
    config.set_file_path("config.yaml");

    if (!SDL_Init(SDL_INIT_EVERYTHING))
    {
        std::cout << "SDL Init Failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        throw std::runtime_error("SDL Init Failed");
    }

    std::unique_ptr<SDL_Window, void (*)(SDL_Window *)> window = setup_window(config);
    std::unique_ptr<SDL_Renderer, void (*)(SDL_Renderer *)> renderer = setup_renderer(window.get());

    std::unique_ptr<PathPlanner> planner = std::make_unique<AStarPathPlanner>();
    std::vector<position_t> path = planner->plan_path(config.map, {(uint16_t)config.start_point.x, (uint16_t)config.start_point.y}, {(uint16_t)config.end_point.x, (uint16_t)config.end_point.y});

    enum class Dragging
    {
        NONE,
        START,
        END,
        MAP_ADD,
        MAP_REMOVE
    };

    Dragging dragging = Dragging::NONE;

    bool running = true;
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            float x, y;
            SDL_GetMouseState(&x, &y);

            x = (uint16_t)std::min(x / config.scale, (config.window_dimensions.x / config.scale) - 1.0f);
            y = (uint16_t)std::min(y / config.scale, (config.window_dimensions.y / config.scale) - 1.0f);

            switch (event.type)
            {
            case SDL_EVENT_QUIT:
                running = false;
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    // Check if we clicked on the start or end point
                    if (x == config.start_point.x && y == config.start_point.y)
                    {
                        dragging = Dragging::START;
                        break;
                    }

                    if (x == config.end_point.x && y == config.end_point.y)
                    {
                        dragging = Dragging::END;
                        break;
                    }

                    dragging = Dragging::MAP_ADD;
                }

                if (event.button.button == SDL_BUTTON_RIGHT)
                {
                    dragging = Dragging::MAP_REMOVE;
                }

                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                switch (dragging)
                {
                case Dragging::START:
                    config.start_point = {x, y};
                    break;
                case Dragging::END:
                    config.end_point = {x, y};
                    break;
                case Dragging::MAP_ADD:
                    // Check that a wall isn't being added to the start or end positions
                    if (x == config.start_point.x && y == config.start_point.y)
                    {
                        break;
                    }

                    if (x == config.end_point.x && y == config.end_point.y)
                    {
                        break;
                    }

                    config.map[x][y] = true; // Draw on the map
                    break;
                case Dragging::MAP_REMOVE:
                    config.map[x][y] = false; // Remove from the map
                    break;
                default:
                    break;
                }

                path = planner->plan_path(config.map, {(uint16_t)config.start_point.x, (uint16_t)config.start_point.y}, {(uint16_t)config.end_point.x, (uint16_t)config.end_point.y});
                dragging = Dragging::NONE;
                break;
            case SDL_EVENT_MOUSE_MOTION:
                // Return early, no changes made
                if (dragging == Dragging::NONE)
                {
                    break;
                }

                switch (dragging)
                {
                case Dragging::START:
                    config.start_point = {x, y};
                    break;
                case Dragging::END:
                    config.end_point = {x, y};
                    break;
                case Dragging::MAP_ADD:
                    // Check that a wall isn't being added to the start or end positions
                    if (x == config.start_point.x && y == config.start_point.y)
                    {
                        break;
                    }

                    if (x == config.end_point.x && y == config.end_point.y)
                    {
                        break;
                    }

                    config.map[x][y] = true; // Draw on the map
                    break;
                case Dragging::MAP_REMOVE:
                    config.map[x][y] = false; // Remove from the map
                    break;
                default:
                    break;
                }

                path = planner->plan_path(config.map, {(uint16_t)config.start_point.x, (uint16_t)config.start_point.y}, {(uint16_t)config.end_point.x, (uint16_t)config.end_point.y});
                break;
            }
        }

        if (!running)
        {
            break;
        }

        // Clear the renderer
        SDL_SetRenderDrawColor(renderer.get(), 0, 0, 0, 255);
        SDL_RenderClear(renderer.get());

        // Draw path
        SDL_SetRenderDrawColor(renderer.get(), 255, 255, 255, 255);
        for (const auto &point : path)
        {
            SDL_FRect cell = {point.first * config.scale, point.second * config.scale, config.scale, config.scale};
            SDL_RenderFillRect(renderer.get(), &cell);
        }

        // Draw map
        SDL_SetRenderDrawColor(renderer.get(), 255, 0, 0, 255);

        uint16_t x = 0, y = 0;

        for (const auto &row : config.map)
        {
            for (const auto &point : row)
            {
                if (point)
                {
                    SDL_FRect cell = {x * config.scale, y * config.scale, config.scale, config.scale};
                    SDL_RenderFillRect(renderer.get(), &cell);
                }
                y++;
            }
            y = 0;
            x++;
        }

        // Draw the start and end points
        SDL_SetRenderDrawColor(renderer.get(), 0, 255, 0, 255);
        SDL_FRect start_rect = {config.start_point.x * config.scale, config.start_point.y * config.scale, config.scale, config.scale};
        SDL_RenderFillRect(renderer.get(), &start_rect);

        SDL_SetRenderDrawColor(renderer.get(), 0, 0, 255, 255);
        SDL_FRect end_rect = {config.end_point.x * config.scale, config.end_point.y * config.scale, config.scale, config.scale};
        SDL_RenderFillRect(renderer.get(), &end_rect);

        SDL_RenderPresent(renderer.get());
    }

    SDL_Quit();
    return 0;
}
