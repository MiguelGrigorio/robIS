"""Bound a mission to one live CCTV identity, with cumulative movement gating."""
from copy import deepcopy
import math


class TargetMoved(RuntimeError):
    def __init__(self, instance):
        super().__init__('Objeto mudou de posição; recalculando aproximação')
        self.instance = instance


class TargetLost(RuntimeError):
    pass


class LiveTarget:
    def __init__(self, instance, replan_distance):
        self.instance = deepcopy(instance)
        self.replan_distance = replan_distance

    def check(self, current):
        if current is None or current['id'] != self.instance['id']:
            raise TargetLost('Posição atual do objeto indisponível ou expirada nas CCTVs')
        old, new = self.instance['pose'], current['pose']
        if math.hypot(new['x'] - old['x'], new['y'] - old['y']) >= self.replan_distance:
            raise TargetMoved(current)
        return current
