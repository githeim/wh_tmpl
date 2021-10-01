#ifndef _PHYSIC_WORLD_H_
#define _PHYSIC_WORLD_H_ 

void CreateWorld();
void DestroyWorld();

void SpinWorld(double dbTimeStep, float &fX,float &fY,float &fAngle, 
  int iVelocityIter=6, int iPositionIter = 2) ;

void ForceToLeft(); 
void ForceToRight(); 
void ForceToUp();
void ForceToDown(); 

#endif /* ifndef _PHYSIC_WORLD_H_ */
