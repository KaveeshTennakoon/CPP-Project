#include "../include/Simulation.h"
#include "../include/Config.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <fstream>
#include <stdexcept>

Config loadConfig()
{
    // Use default values from Config.h
    Config cfg;

    // You can override defaults here if needed
    // cfg.num_particles = 1000;
    // cfg.field_size = 20.0;

    return cfg;
}

void renderASCII(const std::vector<std::unique_ptr<Particle>> &particles, double fieldSize, const Config &cfg)
{
    std::vector<std::vector<int>> gridCounts(cfg.grid_height, std::vector<int>(cfg.grid_width, 0));

    for (const auto &particle : particles)
    {
        double x = particle->getX();
        double y = particle->getY();

        int col = static_cast<int>((x + fieldSize / 2) * cfg.grid_width / fieldSize);
        int row = static_cast<int>((y + fieldSize / 2) * cfg.grid_height / fieldSize);

        col = std::clamp(col, 0, cfg.grid_width - 1);
        row = std::clamp(row, 0, cfg.grid_height - 1);

        gridCounts[row][col]++;
    }

    std::cout << "\033[2J\033[H";

    std::cout << '+' << std::string(cfg.grid_width, '-') << "+\n";

    for (int i = 0; i < cfg.grid_height; ++i)
    {
        std::cout << '|';
        for (int j = 0; j < cfg.grid_width; ++j)
        {
            int count = gridCounts[i][j];
            if (count == 0)
            {
                std::cout << ' ';
            }
            else
            {
                int level = std::min(count, cfg.max_density_level);
                auto it = cfg.density_map.find(level);
                std::cout << (it != cfg.density_map.end() ? it->second : ' ');
            }
        }
        std::cout << "|\n";
    }

    std::cout << '+' << std::string(cfg.grid_width, '-') << "+\n";

    std::cout << std::flush;
}

int main()
{
    try
    {
        Config config = loadConfig();
        std::cout << "Using default configuration" << std::endl;

        Simulation simulation(config);

        simulation.start();

        const double FRAME_TIME = 1.0 / config.target_fps;

        while (simulation.getParticleCount() > 0)
        {
            auto frameStart = std::chrono::high_resolution_clock::now();

            simulation.step();

            renderASCII(simulation.getParticles(), config.field_size, config);

            auto frameEnd = std::chrono::high_resolution_clock::now();
            auto frameDuration = std::chrono::duration<double>(frameEnd - frameStart).count();

            if (frameDuration < FRAME_TIME)
            {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(FRAME_TIME - frameDuration));
            }

            static int frameCount = 0;
            if (++frameCount % 30 == 0)
            {
                auto now = std::chrono::high_resolution_clock::now();
                static auto lastStatTime = now;
                auto elapsed = std::chrono::duration<double>(now - lastStatTime).count();
                double actualFps = (elapsed > 1e-6) ? (30.0 / elapsed) : 0.0;
                lastStatTime = now;

                std::cout << "\nParticles: " << simulation.getParticleCount()
                          << " | Energy: " << simulation.getTotalEnergy()
                          << " | FPS: " << actualFps << std::endl;
            }
        }

        simulation.stop();
        std::cout << "Simulation ended. All particles escaped.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}