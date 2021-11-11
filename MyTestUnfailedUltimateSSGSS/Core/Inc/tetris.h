/* tetris.h */

/**
 * Fichier de définitions pour les fonctions, constantes et variables du jeu.
 * 
 * Soient une matrice de dimension FIELD_H par FIELD_W désignée comme le
 * "puits" (en : field) initialisée par la fonction "void initializeField"
 * et des éléments mobiles désignés comme des "tetriminos" définis dans le
 * fichier "shapes.h". Les tetriminos sont composés de sous-éléments nommés
 * "blocs".
 * 
 * Un tetrimino initialisé au sommet du puits par la fonction
 * "bool resetTetrimino" est emporté vers le bas du puits périodiquement par
 * la fonction "void updateField". Pendant la chute, les fonctions
 * "displaceTetrimino" et "rotateTetrimino" permettent de manipuler le
 * tetrimino en mouvement. "updateField" renvoie une valeur non nulle si
 * la partie est perdue.
 * 
 * Fonction et valeurs à utiliser à l'extérieur des fichiers :
 *   Initialisation d'une partie :
 *     - initializeField
 *     - displayField
 *   Fonctionnement d'une partie :
 *     - updateField
 *     - displaceTetrimino
 *     - rotateTetrimino
 *   Accès aux valeurs :
 *     - score
 *     - nDeletedLines
 */

#include <stdlib.h>
#include <string.h>

#include "MCUFRIEND_kbv.h"
#include "shapes.h"

#ifndef TETRIS_H
#define TETRIS_H

// ########################## VALEURS ##########################
/**
 * Taille d'un bloc en nombre de pixels.
 */
#define BLOCK_SIZE 10

/**
 * Dimensions du puits.
 */
#define FIELD_H 18 // Nombre de rangées.
#define FIELD_W 12 // Nombre de colonnes.

/**
 * Position à l'écran du coin supérieur gauche du puits.
 */
#define FIELD_TOP_LEFT_CORNER_X 60
#define FIELD_TOP_LEFT_CORNER_Y 60

/**
 * Position des autres éléments affichés
 */
#define SCORE_TOP_LEFT_X 0
#define SCORE_TOP_LEFT_Y 0

#define N_DELETED_LINES_TOP_LEFT_X 0
#define N_DELETED_LINES_TOP_LEFT_Y 50

#define PREVIEW_TOP_LEFT_X 0
#define PREVIEW_TOP_LEFT_Y 100

/**
 * Matrice du jeu. Chaque cellule du tableau contient un octet qui indique un
 * type de bloc.
 * 
 * Quand un élément du tableau est nul, cette case est vide et ne contient pas
 * de bloc. Sinon, elle contient un bloc qui peut entraîner une collision.
 */
extern char Field [FIELD_H][FIELD_W];

/**
 * Tetrimino mobile. Il n'y en a qu'un seul par partie. Une fois qu'il
 * atteint le fond du puits, "tetrimino" est converti en quatre blocs et
 * ajouté à la matrice "Field", puis "resetTetrimino" permet de le replacer
 * au sommet du puits.
 */
extern Tetrimino tetrimino;

/**
 * Tetrimino pour afficher le prochain envoyé dans le puits.
 */
extern Tetrimino previewTetrimino;

/**
 * Pointage de la partie.
 */
extern int score;
extern int nDeletedLines;

/**
 * Points attribués pour le nombre de lignes éliminées.
 */
extern const int linePoints[4];

/**
 * Couleurs des tetriminos et de l'arrière-plan.
 */
extern const short TETRIMINO_COLORS[N_TYPES];

extern const short FIELD_COLOR;

extern const short BACKGROUND_COLOR;

extern const short TEXT_COLOR;

// ########################## FONCTIONS ##########################
/**
 * Initialiser le puits "Field" en le remplissant d'éléments nuls. Utile pour
 * nettoyer le puits avant de recommencer une partie.
 */
void initializeField(void);

/**
 * Reconfigurer les valeurs de la variable "tetrimino" pour la replacer au
 * sommet du puits, permettant d'ajouter une autre pièce au jeu.
 * 
 * @param t Pointeur vers un tetrimino à réinitialiser.
 * @return bool Si une collision a lieu dès l'ajout du tetrimino, la partie est
 * 	perdue et "true" est renvoyé. Sinon, "false" est renvoyé.
 */
bool resetTetrimino(Tetrimino* t);

/**
 * Détecter des collisions avec les parois du puits.
 * 
 * @param t Pointeur vers un tetrimino à tester.
 * @return Si une collision est détectée, 1 est renvoyé, 0 sinon.
 */
bool detectWallCollision(Tetrimino* t);

/**
 * Détecter des collisions avec les blocs déposés dans le puits.
 * 
 * @param t Pointeur vers un tetrimino à tester.
 * @return Si une collision est détectée, 1 est renvoyé, 0 sinon.
 */
bool detectBlockCollision(Tetrimino* t);

/**
 * Détecter des collisions avec des blocs ou des parois.
 * 
 * @param t Pointeur vers un tetrimino à tester.
 * @return Si une collision est détectée, 1 est renvoyé, 0 sinon.
 */
bool detectCollision(Tetrimino* t);

/**
 * Copier les attributs de src dans ceux de dst.
 * 
 * @param src Structure d'origine à copier.
 * @param dst Structure où placer la copie.
 */
void copyTetrimino(Tetrimino* src, Tetrimino* dst);

/**
 * Déplacer un tetrimino SI le déplacement n'entraîne pas de collision. Si
 * le déplacement entraîne une collision, il n'est pas effectué.
 * 
 * @param deltaX Déplacement selon l'axe horizontal.
 * @param deltaY Déplacement selon l'axe vertical.
 * @param t Pointeur vers un tetrimino à déplacer.
 * @return Si une collision est détectée, 1 est renvoyé, 0 sinon.
 */
bool displaceTetrimino(int deltaX, int deltaY, Tetrimino* t);

/**
 * Effectuer la rotation d'un tetrimino.
 * 
 * @param t Pointeur vers un tetrimino à faire tourner.
 * @return Si une collision est détectée, 1 est renvoyé, 0 sinon.
 */
bool rotateTetrimino(Tetrimino* t);

/**
 * Mettre le puits à jour :
 * - Faire descendre le tetrimino.
 * - Effacer des lignes si nécessaire et ajuster le score.
 * 
 * @return 0 si la partie est perdue.
 */
bool updateField(void);

/**
 * Effectuer un mouvement ou une rotation de manière aléatoire du tetrimino.
 * Utile pour tester.
 */
void randomDisplaceOrRotate(void);

/**
 * Effacer des lignes du tableaux et faire descendre les blocs
 * en conséquence.
 * 
 * @param lines Tableau de lignes à effacer.
 */
void eraseLine(short row);

/**
 * Effacer un tetrimino affiché sur l'écran.
 * 
 *  @param t Pointeur vers un tetrimino à effacer.
 */
void eraseTetrimino(Tetrimino* t);

/**
 * Afficher un tetrimino dans le puits.
 * 
 *  @param t Pointeur vers un tetrimino à afficher.
 */
void displayTetrimino(Tetrimino* t);

/**
 * Effacer le tetrimino à venir.
 */
void erasePreviewTetrimino();

/**
 * Afficher le tetrimino à venir.
 */
void displayPreviewTetrimino();

/**
 * Afficher les parois du puits et les blocs.
 */
void displayField(void);

/**
 * Effacer l'ancien score et afficher le score présent.
 */
void displayScore(void);

/**
 * Effacer l'ancien nombre de lignes et afficher le nombre de lignes présent.
 */
void displayNDeletedLines(void);

#endif
