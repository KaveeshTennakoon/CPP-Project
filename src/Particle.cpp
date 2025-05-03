#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius)
{
    // Initialize with the provided energy value
}

Particle::~Particle()
{
}

double Particle::getX() const
{
    return x;
}

double Particle::getY() const
{
    return y;
}

void Particle::setPosition(double newX, double newY)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX;
    y = newY;
}

double Particle::getVX() const
{
    return vx;
}

double Particle::getVY() const
{
    return vy;
}

void Particle::setVelocity(double newVX, double newVY)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const
{
    std::lock_guard<std::mutex> lock(particleMutex);
    return energy;
}

double Particle::getMaxEnergy() const
{
    return MAX_ENERGY;
}

void Particle::setEnergy(double newEnergy)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY);
}

void Particle::addEnergy(double delta)
{
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY);
}

void Particle::collide(Particle &other)
{
    // Simple elastic collision
    double dx = x - other.x;
    double dy = y - other.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-10)
        return; // Avoid division by zero

    // Exchange momentum
    double temp_vx = vx;
    double temp_vy = vy;

    vx = other.vx;
    vy = other.vy;

    other.vx = temp_vx;
    other.vy = temp_vy;

    // Transfer some energy
    double energyTransfer = energy * 0.1;
    energy -= energyTransfer;
    other.addEnergy(energyTransfer * 0.8); // Some energy is lost in the collision
}

bool Particle::isColliding(const Particle &other) const
{
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance < (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}
