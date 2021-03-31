import pygame
import numpy as np
from math import sin, cos

RED = (255, 0, 0)
BLUE = (0, 0, 255)
GRAY = (220, 220, 220)
BLACK = (0, 0, 0)
meter = 40  # [pixels/meter] for scaling

screen_height = 20 * meter
screen_width = 24 * meter
origin_x = screen_width/2 - 4*meter
origin_y = screen_height/2
screen = pygame.display.set_mode((screen_width, screen_height))


class MyRect:
    def __init__(self, pos_x, pos_y, fi, width, length, line=1, color=(0, 0, 0)):
        self.pos_x = pos_x + origin_x
        # Turn coordinates right-handed
        self.pos_y = -pos_y + origin_y
        self.fi = -fi
        self.half_width = width/2.0
        self.half_length = length/2.0
        self.corner1 = []
        self.corner2 = []
        self.corner3 = []
        self.corner4 = []
        self.calc_corners()
        self.line = line
        self.color = color

    # Rotate rectangle
    def calc_corners(self):
        R = np.array(((cos(self.fi), -sin(self.fi)),
                      (sin(self.fi),  cos(self.fi))))

        self.corner1 = np.array(( self.half_length,  self.half_width))
        self.corner2 = np.array(( self.half_length, -self.half_width))
        self.corner3 = np.array((-self.half_length, -self.half_width))
        self.corner4 = np.array((-self.half_length,  self.half_width))

        self.corner1 = R.dot(self.corner1) + np.array((self.pos_x, self.pos_y))
        self.corner2 = R.dot(self.corner2) + np.array((self.pos_x, self.pos_y))
        self.corner3 = R.dot(self.corner3) + np.array((self.pos_x, self.pos_y))
        self.corner4 = R.dot(self.corner4) + np.array((self.pos_x, self.pos_y))

    # Update position and recalculate rectangle
    def update(self, pos_x, pos_y, fi):
        self.pos_x = pos_x + origin_x
        self.pos_y = -pos_y + origin_y
        self.fi = -fi
        self.calc_corners()

    def display(self, screen, i=None):
        # Axis
        pygame.draw.line(screen, self.color, [self.pos_x, self.pos_y],
                         [self.pos_x + cos(self.fi) * self.half_length,
                          self.pos_y + sin(self.fi) * self.half_length], self.line)
        # Rectangle
        pygame.draw.line(screen, self.color, self.corner1, self.corner2, self.line)
        pygame.draw.line(screen, self.color, self.corner2, self.corner3, self.line)
        pygame.draw.line(screen, self.color, self.corner3, self.corner4, self.line)
        pygame.draw.line(screen, self.color, self.corner4, self.corner1, self.line)

        # Add index on screen
        if i is not None:
            font = pygame.font.Font('freesansbold.ttf', 20)
            text = font.render(str(i), True, BLACK)
            textRect = text.get_rect()
            textRect.center = (self.pos_x + 0.5*meter, self.pos_y + 2*self.half_width)
            screen.blit(text, textRect)


# Make car
car = MyRect(0, 0, fi=0, width=1.2*meter, length=1.75*meter, line=3)


def make_parking(list_in):
    p_list = []
    for p in list_in:
        p_list.append(MyRect(p[0]*meter, p[1]*meter, p[2], car.half_width*2+0.6*meter,
                             car.half_length*2+0.6*meter, 2, color=BLUE))
    return p_list


parking_list = []
def visualize(p_list):
    pygame.init()
    # set the pygame window name
    pygame.display.set_caption('Automaatjuhtimine')
    global parking_list
    parking_list = make_parking(p_list)


def close():
    pygame.quit()


def update(x, y, fi, park_spot, replay=False):
    screen.fill(GRAY)
    if replay:
        font = pygame.font.Font('freesansbold.ttf', 30)
        text = font.render("REPLAY", True, (0, 50, 200))
        textRect = text.get_rect()
        textRect.center = (100, 60)
        screen.blit(text, textRect)

    # Draw grid
    for i in range(-15, 15):
        if i == 0: linewidth = 2
        else: linewidth = 1
        pygame.draw.line(screen, (170, 170, 170), [origin_x + i * meter, 0], [origin_x + i * meter, screen_height], linewidth)
        pygame.draw.line(screen, (170, 170, 170), [0, origin_y + i * meter], [screen_width, origin_y + i * meter], linewidth)

    # Draw parking spots
    for i in range(len(parking_list)):
        if i == park_spot: parking_list[i].color = RED
        else: parking_list[i].color = BLUE
        parking_list[i].display(screen, i)

    car.update(x*meter, y*meter, fi)
    car.display(screen)

    pygame.display.update()


if __name__ == "__main__":
    visualize()
