#ifndef BALL_H
#define BALL_H

//struct ball_s {
//	int x;							4
//	int y;							4
//	short radius;				2
//	short color;				2
//};

struct ball_s {
		short radius; //2
									//2
		int x;				//4
		short y;			//2
		short color;		//2
										
};

void draw_ball_3D(struct ball_s *p_ball);
void draw_all_ball_3D(struct ball_s *p_ball, int nb_ball);
#endif
