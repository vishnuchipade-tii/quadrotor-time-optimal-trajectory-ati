from dataclasses import dataclass
import math

@dataclass
class SimMetadata:
    dt: float = 0.02
    max_duration: float = 15.0
    start_time: float = 0.0

    @property
    def num_steps(self):
        return math.ceil(self.max_duration / self.dt)