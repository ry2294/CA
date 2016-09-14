#include "ExplicitEuler.h"
#include <iostream>

ExplicitEuler::ExplicitEuler()
: SceneStepper()
{}

ExplicitEuler::~ExplicitEuler()
{}

bool ExplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{   
    // Edge Cases.
    if (dt <= 0 || scene.getNumParticles() <= 0) { 
        return true;
    }

    int num_particles = scene.getNumParticles();
    VectorXs& x_s = scene.getX();
    VectorXs& v_s = scene.getV();
    const VectorXs& m_s = scene.getM();

    VectorXs f(2 * num_particles);
    for (int i = 0; i < 2 * num_particles; i++) {
        f[i] = 0;
    }
    scene.accumulateGradU(f);

    for (int index = 0; index < num_particles * 2; index++) {
        scalar x_x = x_s[index], v_x = v_s[index], m_x = m_s[index];

        // Calculating next position of the particle.
        x_s[index] += dt * v_s[index];

        // Calculating next velocity of the particle.
        if (!scene.isFixed(index / 2)) { 
            v_s[index] += f[index] / m_s[index] * dt;
        }
    }
    
    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
