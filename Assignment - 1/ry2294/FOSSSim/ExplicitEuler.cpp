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

        if (!scene.isFixed(index / 2)) {
            std::cout << "Before x_x: " << x_x 
                  << " v_x: " << v_x 
                  << " m_x: " << m_x << std::endl;
            std::cout << "Kinetic: " << scene.computeKineticEnergy() << std::endl;

            // Calculating next position of the particle.
            x_s[index] += dt * v_s[index];

            // Calculating next velocity of the particle.
            v_s[index] += f[index] / m_s[index] * dt;

            std::cout << "After x_x: " << x_x 
                      << " v_x: " << v_x 
                      << " m_x: " << m_x << std::endl;

            std::cout << "GradU: " << f << std::endl;          
            std::cout << "Kinetic: " << scene.computeKineticEnergy() << std::endl;
            std::cout << "Potential: " << scene.computePotentialEnergy() << std::endl;
            std::cout << "Total: " << scene.computeTotalEnergy() << std::endl;
        }
    }
    
    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
