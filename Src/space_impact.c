/*
 * space_impact.c
 *
 *  Created on: 27 ρεπο. 2016 π.
 *      Author: Home
 */

#include "space_impact.h"
#include "display\lcd.h"
#include "stdint.h"

void drawSpaceShip(SpaceShip* ss){
	  LCD_DrawLine((ss->x),(ss->y),ss->x-4,ss->y+16,ss->color);
	  LCD_DrawLine(ss->x,ss->y+20,ss->x-4,ss->y+16,ss->color);
	  LCD_DrawLine(ss->x,ss->y,ss->x+4,ss->y+16,ss->color);
	  LCD_DrawLine(ss->x,ss->y+20,ss->x+4,ss->y+16,ss->color);
	  LCD_DrawLine(ss->x-3,ss->y+12,ss->x-8,ss->y+20,ss->color);
	  LCD_DrawLine(ss->x-2,ss->y+18,ss->x-8,ss->y+20,ss->color);
	  LCD_DrawLine(ss->x+3,ss->y+12,ss->x+8,ss->y+20,ss->color);
	  LCD_DrawLine(ss->x+2,ss->y+18,ss->x+8,ss->y+20,ss->color);
}

void drawAsteroid(Asteroid* a){
	  LCD_DrawCircle(a->x,a->y,4,a->color);
	  LCD_DrawLine(a->x-4,a->y-5,a->x-4,a->y-10,a->color);
	  LCD_DrawLine(a->x,a->y-7,a->x,a->y-12,a->color);
	  LCD_DrawLine(a->x+4,a->y-5,a->x+4,a->y-10,a->color);
}

void drawBullet(Bullet* b){
	  LCD_DrawLine(b->x,b->y,b->x,b->y-3,b->color);
}

void moveSpaceShip(SpaceShip* ss, MoveState* mstate) {
	if (*mstate != STOP) {
		if (!(ss->x < 0) && !(ss->x > X_BORDER)) {
			cleanSpaceShip(ss);
			ss->x += (*mstate)*STEP;
			drawSpaceShip(ss);
		}
		else {
			ss->x -= (*mstate)*STEP;
			*mstate = STOP;

		}
	}
}

void cleanSpaceShip(SpaceShip* ss) {
	//LCD_FillRect(ss->x-8,ss->y,17,21,BLACK);
	uint16_t c = ss->color;
	ss->color = BLACK;
	drawSpaceShip(ss);
	ss->color = c;
}

void cleanBullet(Bullet* b) {
	//LCD_DrawLine(b->x,b->y,b->x,b->y-3,BLACK);
	uint16_t c = b->color;
	b->color = BLACK;
	drawBullet(b);
	b->color = c;
}

void cleanAsteroid(Asteroid* a) {
	uint16_t c = a->color;
	a->color = BLACK;
	//LCD_FillRect(a->x-4, a->y-12, 9, 15, BLACK);
	drawAsteroid(a);
	a->color = c;
}

void moveBullet(Bullet* b) {
	if (b->state == DEACTIVATE) {
		b->x = b->space_ship->x;
		b->y = SPACE_SHIP_LEVEL;
		//LCD_Printf("%d ", b->state);
	}
	else if (b->state == ACTIVATE) {
		//LCD_Printf("%d ", b->state);
		cleanBullet(b);
		(b->y)--;
		drawBullet(b);
		if (b->y < 0) {
			b->state = DEACTIVATE;
			b->y = SPACE_SHIP_LEVEL;
			//b->x = b->space_ship->x;
		}
	}
}

void moveAsteroid(Asteroid* a) {
	if (a->state == DEACTIVATE) {
		cleanAsteroid(a);
		a->x = getRandom(X_BORDER);
		a->y = 0;
	}
	else if (a->state == ACTIVATE) {
		cleanAsteroid(a);
		(a->y)++;
		drawAsteroid(a);
		if (a->y > Y_BORDER+11) {
			a->state = DEACTIVATE;
			cleanAsteroid(a);
			a->y = 0;
			a->x = getRandom(X_BORDER);
		}
	}
}

uint16_t getRandom(uint16_t max_value) {
	uint16_t value = (HAL_GetTick()%1000)*max_value/1000;
	//LCD_Printf(" %ld  ", value);
	return value;
}

void initBullet(Bullet* b) {
	b->state = ACTIVATE;
}

void initBulletHolder(Bullet* b, SpaceShip* ss) {
	uint8_t i;
	for (i=0; i<BULLETS_QUANTITY; i++) {
	  (b+i)->color = GREEN;
	  (b+i)->state = DEACTIVATE;
	  (b+i)->space_ship = ss;
	  (b+i)->x = (b+i)->space_ship->x;
	  (b+i)->y = (b+i)->space_ship->y;
	}
}

void initAsteroid(Asteroid* a){
	a->state = ACTIVATE;
}

void initAsteroidArmy(Asteroid* a){
	uint8_t i;
	for (i=0; i<ASTEROIDS_QUANTITY; i++) {
	  (a+i)->color = GREEN;
	  (a+i)->state = DEACTIVATE;
	  (a+i)->x = getRandom(X_BORDER);
	  (a+i)->y = 0;
	}
}

void initSpaceShip(SpaceShip* ss) {
	ss->state = ACTIVATE;
	ss->color = GREEN;
	ss->x = START_POSITION;
	ss->y = SPACE_SHIP_LEVEL;
}

uint8_t  isKilled(Asteroid* a, Bullet* b){
	if (((b->x)<(a->x+5))&&((b->x)>(a->x-5))&&((b->y)<(a->y+4))&&((b->y)>(a->y-4))&&(a->state != DEACTIVATE)&&(b->state != DEACTIVATE)) {
		//LCD_Printf("!");
		return 1;
	} else return 0;
}

uint8_t isAsteroidHitSpaceShip(SpaceShip* ss, Asteroid* a){
	if (((ss->x)<(a->x+12))&&((ss->x)>(a->x-12))&&((ss->y)<(a->y+4))&&((ss->y)>(a->y-32))&&(a->state != DEACTIVATE)&&(ss->state != DEACTIVATE)) {
		return 1;
		LCD_Printf("!");
	} else return 0;
}

void calculateAsteroidArmy(Asteroid* a){
	static uint8_t next_asteroid_timer = 0;
	static uint8_t next_asteroid = 0;

	if (next_asteroid_timer == 0) {
		initAsteroid(a+next_asteroid);
		next_asteroid++;
		if (next_asteroid >= ASTEROIDS_QUANTITY)
			 next_asteroid = 0;
		   	 next_asteroid_timer = 50+getRandom(50);
	    }
		next_asteroid_timer--;

}

void calculateBulletHolder(Bullet* b){
	static uint8_t next_bullet_timer = 0;
	static uint8_t next_bullet = 0;

	if (next_bullet_timer == 0 && b->space_ship->state == ACTIVATE) {
	    initBullet(b+next_bullet);
		next_bullet++;
		if (next_bullet >= BULLETS_QUANTITY)
			next_bullet = 0;
	}

	if (next_bullet_timer < BULLETS_DELAY)
	  	next_bullet_timer++;
    else next_bullet_timer = 0;

}

void analizeSituation(SpaceShip* ss, Asteroid* a, Bullet* b){
	uint8_t i;
	uint8_t j;
	for (i=0; i<ASTEROIDS_QUANTITY; i++){
		if (isAsteroidHitSpaceShip(ss, a+i)) {
			ss->state = DEACTIVATE;
			(a+i)->state = DEACTIVATE;
			cleanAsteroid(a+i);
			cleanSpaceShip(ss);
		}
		for (j=0; j<BULLETS_QUANTITY; j++) {
			if (isKilled(a+i, b+j)) {
				(a+i)->state = DEACTIVATE;
				(b+j)->state = DEACTIVATE;
				cleanAsteroid(a+i);
				cleanBullet(b+j);
			}
		}
	}
}
