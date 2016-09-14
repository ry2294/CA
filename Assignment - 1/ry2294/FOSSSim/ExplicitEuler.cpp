#include "ExplicitEuler.h"
#include <iostream>

void ExplicitEuler::setNextPosition(VectorXs& X_s, const VectorXs& V_s, int index, scalar dt) {
    X_s[index] += dt * V_s[index];
    X_s[index + 1] += dt * V_s[index + 1];
}

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

    int num_particles = scene.getNumParticles(), index = 0;
    VectorXs& x_s = scene.getX();
    VectorXs& v_s = scene.getV();
    const VectorXs& m_s = scene.getM();

    while (index < num_particles * 2) {
        scalar x_x = x_s[index], x_y = x_s[index + 1], 
               v_x = v_s[index], v_y = v_s[index + 1], 
               m_x = m_s[index], m_y = m_s[index + 1];

        std::cout << "Before x_x: " << x_x << " x_y: " << x_y
                  << " v_x: " << v_x << " v_y: " << v_y
                  << " m_x: " << m_x << " m_y: " << m_y << std::endl;
        std::cout << "Kinetic: " << scene.computeKineticEnergy() << std::endl;

        if (!scene.isFixed(index / 2)) {
            setNextPosition(x_s, v_s, index, dt);
        } else {
            // Ignore.
            std::cout << "fixed index: " << index / 2 << " ";
        }

        std::cout << "After x_x: " << x_x << " x_y: " << x_y
                  << " v_x: " << v_x << " v_y: " << v_y
                  << " m_x: " << m_x << " m_y: " << m_y << std::endl;
        std::cout << "Kinetic: " << scene.computeKineticEnergy() << std::endl;

        index += 2;
    }
    
    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
