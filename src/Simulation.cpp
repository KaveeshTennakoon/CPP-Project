#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>
Simulation::Simulation(const Config &config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads)
{
    initializeParticles(config);
}

Simulation::~Simulation()
{
    stop();
}

void Simulation::initializeParticles(const Config &config)
{
    std::random_device rd;
    std::mt19937 gen(config.random_seed ? config.random_seed : rd());
    std::uniform_real_distribution<> dis(-fieldSize / 2, fieldSize / 2);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0); // Velocity range

    size_t count = config.num_particles;
    for (size_t i = 0; i < count; ++i)
    {
        auto particle = std::make_unique<Particle>(
            dis(gen), dis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy);
        particle->setVelocity(vel_dis(gen), vel_dis(gen));
        particles.push_back(std::move(particle));
    }
    std::cout << "Initialized " << particles.size() << " particles." << std::endl;
}

void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field)
{
    containmentField = std::move(field);
}

void Simulation::start()
{
    running = true;
    for (size_t i = 0; i < numThreads; ++i)
    {
        workerThreads.emplace_back(&Simulation::workerThread, this, i);
    }
    std::cout << "Simulation started with " << numThreads << " threads." << std::endl;
}

void Simulation::stop()
{
    running = false;
    for (auto &thread : workerThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
    workerThreads.clear();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step()
{
    removeEscapedParticles();
    applyForces(timeStep);

    if (std::rand() % 3 != 0)
    {
        handleCollisions();
    }
}

void Simulation::addParticle(std::unique_ptr<Particle> particle)
{
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles()
{
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
                       [this](const std::unique_ptr<Particle> &p)
                       {
                           return !containmentField->isParticleContained(*p);
                       }),
        particles.end());
}

size_t Simulation::getParticleCount() const
{
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>> &Simulation::getParticles() const
{
    return particles;
}

double Simulation::getTotalEnergy() const
{
    double total = 0.0;
    for (const auto &particle : particles)
    {
        total += particle->getEnergy();
    }
    return total;
}

void Simulation::setNumThreads(size_t newNumThreads)
{
    numThreads = newNumThreads;
    threadManager->setNumThreads(newNumThreads);
}

size_t Simulation::getNumThreads() const
{
    return numThreads;
}

void Simulation::updatePositions(double dt)
{
    for (auto &particle : particles)
    {
        double x = particle->getX() + particle->getVX() * dt;
        double y = particle->getY() + particle->getVY() * dt;
        particle->setPosition(x, y);
    }
}

void Simulation::handleCollisions()
{
    for (size_t i = 0; i < particles.size(); ++i)
    {
        for (size_t j = i + 1; j < particles.size(); ++j)
        {
            if (particles[i]->isColliding(*particles[j]))
            {
                particles[i]->collide(*particles[j]);
            }
        }
    }
}

void Simulation::applyForces(double dt)
{
    for (auto &particle : particles)
    {
        double x = particle->getX();
        double y = particle->getY();
        double distance = std::sqrt(x * x + y * y);

        // Apply containment field force
        double force = containmentField->getContainmentForce(*particle);

        // Direction is toward the center (opposite of position)
        double ax = -x / (distance + 1e-10) * force;
        double ay = -y / (distance + 1e-10) * force;

        // Update velocity
        double vx = particle->getVX() + ax * dt;
        double vy = particle->getVY() + ay * dt;

        particle->setVelocity(vx, vy);
    }
}

void Simulation::workerThread(size_t threadId)
{
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        volatile int sum = 0;
        for (volatile int i = 0; i < 1000; i++)
        {
            sum += i;
        }
    }
}