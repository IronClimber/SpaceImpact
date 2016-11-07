/*
 * space_impact.h
 *
 *  Created on: 27 ρεπο. 2016 π.
 *      Author: Home
 */


#ifndef SPACE_IMPACT_H_
#define SPACE_IMPACT_H_

#include "stdint.h"

//Align
#define VERTICAL 1
#define HORIZONTAL 0

#define ALIGN VERTICAL         //HORIZONTAL

#if ALIGN == VERTICAL
	#define X_BORDER 239
	#define Y_BORDER 319
#elif ALIGN == HORIZONTAL
	#define X_BORDER 319
	#define Y_BORDER 239
#else
	#define X_BORDER 239
	#define Y_BORDER 239
#endif

//Move parameters
#define STEP 1
#define SPEED 10

//Ship Start position
#define SPACE_SHIP_LEVEL Y_BORDER-50
#define START_POSITION (uint16_t)(X_BORDER/2)


#define BULLETS_QUANTITY 15
#define BULLETS_DELAY 20
#define ASTEROIDS_QUANTITY 10
#define ASTEROIDS_DELAY 85

#define NEXT_TRY_DELAY 200

typedef enum {
	LEFT =  -1,
	STOP = 0,
	RIGHT = 1
} MoveState;

typedef enum {
	DEACTIVATE = 0,
	ACTIVATE = 1,
} State;


typedef struct {
	State state;
	uint16_t color;
	int16_t x;
	int16_t y;
} SpaceShip;

typedef struct {
	uint16_t color;
	State state;
	SpaceShip* space_ship;
	int16_t x;
	int16_t y;
} Bullet;

typedef struct {
	State state;
	int16_t x;
	int16_t y;
	uint16_t color;
} Asteroid;



void drawSpaceShip(SpaceShip* ss);
void drawAsteroid(Asteroid* a);
void drawBullet(Bullet* b);

void moveSpaceShip(SpaceShip* ss, MoveState* mstate);
void moveBullet(Bullet* b);
void moveAsteroid(Asteroid* a);

void cleanSpaceShip(SpaceShip* ss);
void cleanBullet(Bullet* b);
void cleanAsteroid(Asteroid* a);

void initSpaceShip(SpaceShip* ss);
void initBullet(Bullet* b);
void initBulletHolder(Bullet* b, SpaceShip* ss);
void initAsteroid(Asteroid* a);
void initAsteroidArmy(Asteroid* a);

void calculateAsteroidArmy(Asteroid* a);
void calculateBulletHolder(Bullet* b);

void analizeSituation(SpaceShip* ss, Asteroid* a, Bullet* b);

uint16_t getRandom(uint16_t max_value);
uint8_t  isKilled(Asteroid* a, Bullet* b);
uint8_t  isAsteroidHitSpaceShip(SpaceShip* ss, Asteroid* a);


#endif /* SPACE_IMPACT_H_ */
