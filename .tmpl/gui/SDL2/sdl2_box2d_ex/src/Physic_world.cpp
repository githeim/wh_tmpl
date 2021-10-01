// for box2d
#include "box2d/box2d.h"
#include <stdio.h>

b2World* g_pWorld;


b2Body* g_pGroundBody;
b2Body* g_pBody;
void CreateWorld() {
  
  b2Vec2 gravity(0.0f,-10.0f);
  g_pWorld = new b2World(gravity);

  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f,-10.0f);

  g_pGroundBody = g_pWorld->CreateBody(&groundBodyDef);
  
  b2PolygonShape groundBox;
  groundBox.SetAsBox(20.0f, 10.0f);

  g_pGroundBody->CreateFixture(&groundBox,0.0f);

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(0.0f,20.0f);

  bodyDef.linearVelocity.Set(3.0f, 0.0f);
  g_pBody = g_pWorld->CreateBody(&bodyDef);

  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox(1.0f, 1.0f);

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = 1.0f;
  fixtureDef.friction = 0.3f;
  fixtureDef.restitution = 0.8f;

  g_pBody->CreateFixture(&fixtureDef);

}

void DestroyWorld() {
  g_pWorld->DestroyBody(g_pGroundBody);
  g_pWorld->DestroyBody(g_pBody);
  delete g_pWorld;
}

void SpinWorld(double dbTimeStep, float &fX,float &fY,float &fAngle, 
  int iVelocityIter=6, int iPositionIter = 2) {
  g_pWorld->Step(dbTimeStep, iVelocityIter, iPositionIter);

  b2Vec2 vec2Position = g_pBody->GetPosition();
  fAngle = g_pBody->GetAngle();
  fX = vec2Position.x;
  fY = vec2Position.y;
}

void ForceToLeft() {
  g_pBody->ApplyForceToCenter(b2Vec2(-100,0), true);
}
void ForceToRight() {
  g_pBody->ApplyForceToCenter(b2Vec2(100,0), true);
}
void ForceToUp() {
  g_pBody->ApplyForceToCenter(b2Vec2(0,300), true);
}
void ForceToDown() {
  g_pBody->ApplyForceToCenter(b2Vec2(0,-300), true);
}

