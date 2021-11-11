#include "tetris.h"

/**
 * Fichier d'implémentations des définitions de "tetris.h".
 * 
 * Veuillez vous référer au fichier d'en-tête pour consuler la documentation
 * des fonctions.
 */

// Valeurs
char Field [FIELD_H][FIELD_W] = {0};
Tetrimino tetrimino = {0};
Tetrimino previewTetrimino = {0};

const short TETRIMINO_COLORS[N_TYPES] = {
	BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK};
const short FIELD_COLOR = WHITE;
const short BACKGROUND_COLOR = DARKGREY;
const short TEXT_COLOR = BLACK;

int score = 0;
int nDeletedLines = 0;
const int linePoints[4] = {40, 100, 300, 1200};

// Logique du jeu.	
void initializeField()
{
	memset(Field, 0, sizeof(Field[0][0]) * FIELD_H * FIELD_W);
	previewTetrimino.type = rand() % N_TYPES;
	resetTetrimino(&tetrimino);
	score = 0;
	nDeletedLines = 0;
	displayScore();
	displayNDeletedLines();
	displayPreviewTetrimino();
}

bool resetTetrimino(Tetrimino* t)
{
	t->x = (FIELD_W / 2) - 2;
	t->y = 0;
	t->rotation = 0;
	t->type = previewTetrimino.type;
	previewTetrimino.type = rand() % N_TYPES;;
	erasePreviewTetrimino();
	displayPreviewTetrimino();
	displayTetrimino(t);

	if (detectBlockCollision(t))
		return true;
	return false;
}

bool detectWallCollision(Tetrimino* t)
{
	int max_i = 0, max_j = 0, min_j = 3;

	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++) {
			if(TETRIMINOS[t->type][t->rotation % N_ROTATIONS][i][j]) {
				if(max_i < i)
					max_i = i;
				if(max_j < j)
					max_j = j;
				if(min_j > j)
					min_j = j;
			}
		}
	}
	int right = t->x + max_j;
	int left = t->x + min_j;
	int bottom = t->y + max_i;

	if(left < 0 || right >= FIELD_W)
		return true;

	// Inutile de détecter des collisions vers le haut parce que le tetrimino
	// se déplace dans une seule direction.
	if (bottom >= FIELD_H + 1)
		return true;

	return false;
}

bool detectBlockCollision(Tetrimino* t)
{
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			short rotation = tetrimino.rotation % N_ROTATIONS;
			bool isTetriminoBlockSolid = (
				TETRIMINOS[t->type][rotation][i][j]);
			// On soustrait 1 pour convertir les coordonnées d'un tetrimino
			// vers celles du puits (Field).
			short row = t->y + i - 1;
			if (row < 0) // Éviter d'accéder à des éléments hors du tableau.
				row = 0;
			bool isFieldBlockSolid = Field[row][t->x + j];
			if (isTetriminoBlockSolid && isFieldBlockSolid){
				return true;
			}
		}
	}
	return false;
}

bool detectCollision(Tetrimino* t)
{
	return detectWallCollision(t) || detectBlockCollision(t);
}

void copyTetrimino(Tetrimino* src, Tetrimino* dst)
{
	dst->x = src->x;
	dst->y = src->y;
	dst->rotation = src->rotation;
	dst->type = src->type;
}

bool displaceTetrimino(int deltaX, int deltaY, Tetrimino* t)
{
	Tetrimino probe;
	copyTetrimino(t, &probe);

	probe.x += deltaX;
	probe.y += deltaY;

	if (!detectCollision(&probe)){
		eraseTetrimino(t);
		t->x += deltaX;
		t->y += deltaY;
		displayTetrimino(t);
		return false;
	}
	return true;
}

bool rotateTetrimino(Tetrimino* t)
{
	Tetrimino probe;
	copyTetrimino(t, &probe);

	probe.rotation++;

	if (!detectCollision(&probe)){
		eraseTetrimino(t);
		t->rotation++;
		displayTetrimino(t);
		return false;
	}
	return true;
}

bool updateField()
{
	// Faire tomber le tetrimino en mouvement.
	bool collision = displaceTetrimino(0, 1, &tetrimino);

	// S'il y a une collision vers le bas, il faut actualiser le puits.
	if (collision) {
		// Remplacer le tetrimino mobile par des blocs dans "Field".
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				short row = i + tetrimino.y - 1;
				short col = j + tetrimino.x;
				short rotation = tetrimino.rotation % N_ROTATIONS;
				bool isTetriminoBlockSolid = (
					TETRIMINOS[tetrimino.type][rotation][i][j]);
				if (isTetriminoBlockSolid && row < FIELD_H) {
					// On ajoute 1 au type parce que 0 correspond déjà à une
					// case vide dans le tableau Field.
					Field[row][col] = tetrimino.type + 1;
				}
			}
		}
		
		// Effacer des lignes si nécessaire.
		short nLinesToErase = 0;
		for(short row = FIELD_H - 1; row > 0; row--){
			bool isLineFull = true;
			for(short col = 0; col < FIELD_W; col++){
				if (!Field[row][col]){
					isLineFull = false;
					break;
				}
			}
			if (isLineFull) {
				eraseLine(row);
				nLinesToErase++;
				row++;
			}
		}
		if (nLinesToErase) {
			displayField();
			score += linePoints[nLinesToErase - 1];
			nDeletedLines += nLinesToErase;
			displayScore();
			displayNDeletedLines();
		}

		// Réinitialiser le tetrmino et vérifier s'il y a une erreur.
		bool lostGame = resetTetrimino(&tetrimino);
		if (lostGame) {
			return true;
		}
	}
	return false;
}

void eraseLine(short row)
{
	for (short i = row - 1; i > 0; i--){
		for (short col = 0; col < FIELD_W; col++){
			Field[i + 1][col] = Field[i][col];
		}
	}
}

void randomDisplaceOrRotate()
{
	if (rand() % 2) {
		return;
	}
	if (rand() % 13) {
		displaceTetrimino(rand() % 3 - 1, 0, &tetrimino);
	}
	else {
		rotateTetrimino(&tetrimino);
	}
}

// Affichage
void eraseTetrimino(Tetrimino* t){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			if(TETRIMINOS[t->type][t->rotation % N_ROTATIONS][i][j]){
				int x = ((t->x + j) * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_X;
				int y = ((t->y + i) * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_Y;
				if (t->y + i <= 0) // Éviter de dessiner hors du cadre.
					continue;
				LCD_FillRect(x, y, BLOCK_SIZE, BLOCK_SIZE, FIELD_COLOR);
			}
		}
	}
}

void displayTetrimino(Tetrimino* t)
{
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			if(TETRIMINOS[t->type][t->rotation % N_ROTATIONS][i][j]){
				int x = ((t->x + j) * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_X;
				int y = ((t->y + i) * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_Y;
				if (t->y + i <= 0) // Éviter de dessiner hors du cadre.
					continue;
				short color = TETRIMINO_COLORS[t->type];
				LCD_FillRect(x, y, BLOCK_SIZE, BLOCK_SIZE, color);
			}
		}
	}
}

void erasePreviewTetrimino()
{
	LCD_FillRect(PREVIEW_TOP_LEFT_X, PREVIEW_TOP_LEFT_Y,
		BLOCK_SIZE*4, BLOCK_SIZE*4, BACKGROUND_COLOR);
}

void displayPreviewTetrimino()
{
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			Tetrimino* t = &previewTetrimino;
			if(TETRIMINOS[t->type][t->rotation % N_ROTATIONS][i][j]){
				int x = ((t->x + j) * BLOCK_SIZE) + PREVIEW_TOP_LEFT_X;
				int y = ((t->y + i) * BLOCK_SIZE) + PREVIEW_TOP_LEFT_Y;
				short color = TETRIMINO_COLORS[t->type];
				LCD_FillRect(x, y, BLOCK_SIZE, BLOCK_SIZE, color);
			}
		}
	}
}

void displayField()
{
	LCD_FillRect(FIELD_TOP_LEFT_CORNER_X,
		FIELD_TOP_LEFT_CORNER_Y + BLOCK_SIZE,
		FIELD_W * BLOCK_SIZE,
		FIELD_H * BLOCK_SIZE,
		FIELD_COLOR);
	
	for (int row = 0; row < FIELD_H; row++){
		for (int col = 0; col < FIELD_W; col++){
			if (Field[row][col] == 0)
				continue;
			int x = (col * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_X;
			int y = ((row + 1) * BLOCK_SIZE) + FIELD_TOP_LEFT_CORNER_Y;
			// On soustrait 1 à la couleur pour tenir compte de la
			// représentation des éléments dans Field (voir la remarque dans
			// la fonction "updateField")
			short color = TETRIMINO_COLORS[Field[row][col] - 1];
			LCD_FillRect(x, y, BLOCK_SIZE, BLOCK_SIZE, color);
		}
	}
}

void displayScore()
{
	char str[12];
	sprintf(str, "%d", score);
	LCD_SetCursor(SCORE_TOP_LEFT_X, SCORE_TOP_LEFT_Y);
	LCD_SetTextColor(TEXT_COLOR, BACKGROUND_COLOR);
	LCD_Printf(str);
}

void displayNDeletedLines()
{
	char str[12];
	sprintf(str, "%d", nDeletedLines);
	LCD_SetCursor(N_DELETED_LINES_TOP_LEFT_X, N_DELETED_LINES_TOP_LEFT_Y);
	LCD_SetTextColor(TEXT_COLOR, BACKGROUND_COLOR);
	LCD_Printf(str);
}
