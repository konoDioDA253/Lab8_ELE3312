/* shapes.h */

/**
 * Fichier de définitions associées aux tetriminos.
 */

#ifndef SHAPES_H
#define SHAPES_H
 
/**
 * Nombre de types de tetrimino.
 */
#define N_TYPES 7

/**
 * Nombre de rotations.
 */
#define N_ROTATIONS 4

/**
 * Dimension de chaque matrice de rotation.
 */
#define TETRIMINO_SIZE 4

/**
 * Types de tetrimino.
 */
enum TETRIMINO_TYPE{O=0, I, S, Z, L, J, T};

/**
 * Structure de définition d'un tetrimino.
 */
typedef struct Tetrimino {
	short x;
	short y;
	enum TETRIMINO_TYPE type;
	char rotation;
} Tetrimino;

/**
 * Tableau des tetriminos possibles.
 */
extern char TETRIMINOS[N_TYPES][N_ROTATIONS][TETRIMINO_SIZE][TETRIMINO_SIZE];

#endif
