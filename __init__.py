
from random import random

import bl00mbox
import leds
from st3m.application import Application, ApplicationContext
import st3m.run

class RK4(object):

    def __init__(self, h=0.02):
        self.h = h
        self.hh = h / 2

        self.ranges = tuple([mx - mi for mi, mx in self.limits])

    def diff(self, vec, var, delta):
        return [v  if i != var else v + delta
            for i, v in enumerate(vec)]

    def deltas(self, vec, kvec, mult):
        return zip(
            self.partials,
            (self.diff(vec, i, mult * k) for i, k in enumerate(kvec))
        )

    def step(self, vec, t):
        hdt = t + self.hh
        dt = t + self.h

        K1 = [f(t, vec) for f in self.partials]
        K2 = [f(hdt, delta) for f, delta in self.deltas(vec, K1, self.hh)]
        K3 = [f(hdt, delta) for f, delta in self.deltas(vec, K2, self.hh)]
        K4 = [f(dt, delta) for f, delta in self.deltas(vec, K3, self.h)]

        return tuple([
            v + self.h / 6 * (k1 + 2 * k2 +  2 * k3 + k4)
            for v, k1, k2, k3, k4 in zip(vec, K1, K2, K3, K4)
        ])

    def __iter__(self):
        vec = tuple([0.01 * random() for _ in self.partials])
        while True:
            vec = self.step(vec, 0)
            yield vec

    def trace(self):
        all_points = self.__iter__()
        last_point = all_points.__next__()
        this_point = all_points.__next__()
        while True:
            scaled = tuple([(p - lim[0]) / rng for p, lim, rng
                in zip(this_point, self.limits, self.ranges)])

            triggers = tuple([nxt < 0 == prv < 0
                for nxt, prv in zip(this_point, last_point)])

            yield scaled, triggers

            last_point = this_point
            this_point = all_points.__next__()

class Lorenz(RK4):

    def __init__(self):
        self.rho = 28.0
        self.sigma = 10.0
        self.beta = 8.0 / 3

        self.partials = (self.dxdt, self.dydt, self.dzdt)

        self.limits = ((-20.0, 25.0), (-40.0, 40.0), (0, 60))

        super(Lorenz, self).__init__()

    def dxdt(self, t, vec):
        x, y, z = vec
        return self.sigma * (y - x)

    def dydt(self, t, vec):
        x, y, z = vec
        return x * (self.rho - z) - y

    def dzdt(self, t, vec):
        x, y, z = vec
        return x * y - self.beta * z

class Chaoscillator(Application):
    def __init__(self, app_ctx):
        super().__init__(app_ctx)

        self.attractor = Lorenz()
        self.trace = self.attractor.trace()
        self.pos, _ = self.trace.__next__()

        self.trail = 128
        self.points = [self.point(self.pos[0], self.pos[2])]
        self.last_point = 0

        self.last_led = 0

        self.chan = bl00mbox.Channel('chaosxxiii')
        self.chan.volume = 8000
        self.mix = self.chan.new(bl00mbox.plugins.mixer)

        self.osc1 = self.chan.new(bl00mbox.plugins.osc_fm)
        self.osc2 = self.chan.new(bl00mbox.plugins.osc_fm)

        self.lp1 = self.chan.new(bl00mbox.plugins.lowpass)
        self.lp2 = self.chan.new(bl00mbox.plugins.lowpass)

        self.osc1.signals.output = self.lp1.signals.input
        self.osc2.signals.output = self.lp2.signals.input

        self.lp1.signals.output = self.mix.signals.input0
        self.lp2.signals.output = self.mix.signals.input1

        self.mix.signals.output = self.chan.mixer

    def point(self, x, y):
        return (-100 + 200 * x, -100 + 200 * (1-y))

    def rgb_led(self, point):
        return point

    def synth_update(self):

        self.osc1.signals.pitch.freq = 100 + 400 * self.pos[0]
        self.osc1.signals.lin_fm.value = 5000 * self.pos[1]
        self.lp1.signals.freq = 100 + 500 * self.pos[2]

        self.osc2.signals.pitch.freq = 100 + 400 * self.pos[1]
        self.osc2.signals.lin_fm.value = 5000 * self.pos[2]
        self.lp2.signals.freq = 100 + 500 * self.pos[0]

    def draw(self, ctx):
        x, y, z = self.pos
        self.pos, _ = self.trace.__next__()
        x, y, z = self.pos

        ctx.rgb(0, 0, 0).rectangle(-120, -120, 240, 240).fill()

        new_point = self.point(self.pos[0], self.pos[2])
        if len(self.points) < self.trail:
            self.points.append(new_point)
        else:
            self.points[self.last_point] = new_point
            self.last_point = (self.last_point + 1) % self.trail

        ctx.rgb(0, 255, 0)
        for point in self.points   :
            h, k = point
            ctx.rectangle(h, k, 1, 1).fill()

        re, gr, bl = self.rgb_led(self.pos)
        leds.set_rgb(self.last_led, re, gr, bl)
        self.last_led = (self.last_led + 1) % 40
        leds.update()

        self.synth_update()

    def think(self, ins: InputState, delta_ms: int):
        pass

    def on_exit(self):
        self.chan.free = True

if __name__ == '__main__':
    st3m.run.run_view(Chaoscillator(ApplicationContext()))
