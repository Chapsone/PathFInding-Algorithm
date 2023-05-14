#!/usr/bin/env python3
import pygame, sys
import threading

# Matrix
matrix = [
    [1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 1, 1],
    [0, 1, 0, 1, 0, 1]]

# Color
FREE = (255, 255, 255) 
OCCUPIED = (0, 0, 0)
BARRER = (0, 255, 0) # green

# Dimension
DIM = 100

# setup pygame
HIDTH = 300
WIDTH = 600
pygame.init()
SCREEN = pygame.display.set_mode((WIDTH, HIDTH))
pygame.display.set_caption("Pathfinder Algo")

# fonction that convert the matrix into a rectangle bject
def creat_surface():
    rows = len(matrix)
    cols = len(matrix[0])
    surface = []
    for i in range(rows):
        surface.append([])
        for j in range(cols):
            surface[i].append(pygame.Surface((DIM,DIM)))
            if matrix[i][j] == 0:
                surface[i][j].fill(FREE)
            elif matrix[i][j] == 1:
                surface[i][j].fill(OCCUPIED)
    return surface

# the function that display the surface in the Screen
def draw(surface):
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        for j in range(cols):
            SCREEN.blit(surface[i][j],(j*DIM , i*DIM))

# that one is dilplaying the limit of any cell so that we can see all of theme
def make_grid():
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        pygame.draw.line(SCREEN, BARRER, (0, i*DIM), (DIM*cols, i*DIM))
        for j in range(cols):
            pygame.draw.line(SCREEN, BARRER, (j * DIM, 0), (j * DIM, DIM * rows))

def main():
    clock = pygame.time.Clock()
    while True:
        surface = creat_surface()
        draw(surface)
        make_grid()
        pygame.display.update()

        #close the All when click on the shutdown buton
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        clock.tick(60)

if __name__ == '__main__':
    main()


