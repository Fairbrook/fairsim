import pygame
import sys
import argparse
import math


WHITE = (255, 255, 255)
BLUE = (0, 100, 255)
BLACK = (0, 0, 0)


class DiferentialWheeledRobot:
    def __init__(self, width=50, long=60):
        self.width = width
        self.long = long
        self.x = 0
        self.y = 0
        self.theta = 0
        self.surface = pygame.Surface((self.width+20, self.long + 4))
        self.surface.fill(WHITE)

    def update_state(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta * 180 / math.pi
        self.theta -= 90
        if self.theta > 360:
            self.theta -= 360
        if self.theta > 180:
            self.theta -= 180
        print(self.theta)

    def draw(self, screen: pygame.Surface):
        screen_h = screen.get_height()
        rect = pygame.Rect(self.x - self.surface.get_width()/2, screen_h - self.y - self.surface.get_height()/2,
                           self.surface.get_width(), self.surface.get_height())
        pygame.draw.rect(self.surface, BLUE, ((
            self.surface.get_width()-self.width)/2, (self.surface.get_height()-self.long)/2,
            self.width, self.long))
        pygame.draw.rect(self.surface, (255, 0, 0), ((
            self.surface.get_width()-self.width/2)/2, self.surface.get_height()-self.long+4,
            self.width/2, self.long/4))
        pygame.draw.rect(self.surface, BLACK, (0, self.long-33, 10, 30))
        pygame.draw.rect(self.surface, BLACK,
                         (self.surface.get_width()-10, self.long-33, 10, 30))
        rotated = pygame.transform.rotate(self.surface, self.theta)
        screen.blit(rotated, rect)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-vl")
    parser.add_argument("-vr")
    parser.add_argument("-t", default=10)
    args = parser.parse_args()
    print(args)

    pygame.init()

    SCREEN_WIDTH = 1600
    SCREEN_HEIGHT = 900
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Moving Car")
    clock = pygame.time.Clock()
    car = DiferentialWheeledRobot()

    running = True

    dt = 0.01
    t = 0
    T = float(args.t)

    x = y = theta = 0
    x = SCREEN_WIDTH / 2
    y = SCREEN_HEIGHT / 2

#    x = car.width + 10
#    y = car.long + 10
    V = xp = yp = thetap = 0
    vl = float(args.vl)
    vr = float(args.vr)

    while running and t < T:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        thetap = (vr - vl)/car.width
        V = (vr + vl)/2
        xp = V*math.cos(theta)
        yp = V*math.sin(theta)

        x += xp * dt
        y += yp * dt
        theta += thetap * dt

        car.update_state(x, y, theta)

        screen.fill(WHITE)
        car.draw(screen)

        pygame.display.flip()
        clock.tick(60)
        t += dt

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
