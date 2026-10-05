#!/usr/bin/env python3
"""Animate the vendored FireSmoke PROTO sprite sheets."""

from controller import Robot


class FireMovement(Robot):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.fire_display = self.getDevice("fireDisplay")
        self.fire_image = self.fire_display.imageLoad("320x320_fire_sprint.png")
        self.fire_frames = [(x, y) for x in (0, -320, -640, -960)
                            for y in (0, -320, -640, -960)]

    def run(self):
        frame = 0
        while self.step(self.time_step) != -1:
            fire_x, fire_y = self.fire_frames[frame % len(self.fire_frames)]
            self.fire_display.imagePaste(self.fire_image, fire_x, fire_y, False)
            frame += 1


if __name__ == "__main__":
    FireMovement().run()
