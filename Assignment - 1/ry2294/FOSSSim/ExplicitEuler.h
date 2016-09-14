#ifndef __EXPLICIT_EULER__
#define __EXPLICIT_EULER__

#include "SceneStepper.h"
#include "MathDefs.h"

class ExplicitEuler : public SceneStepper
{
public:
    ExplicitEuler();
    virtual ~ExplicitEuler();
    
    virtual bool stepScene( TwoDScene& scene, scalar dt );
    
    virtual std::string getName() const;

private:
	void setNextPosition(VectorXs& X_s, const VectorXs& V_s, int index, scalar dt);
};

#endif
