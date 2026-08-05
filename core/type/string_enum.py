from enum import Enum


class StringEnum(Enum):

    @classmethod
    def from_string(cls, value: str):
        try:
            return cls(value)
        except ValueError:
            raise ValueError(
                f"Unknown {cls.__name__}: {value}. "
                f"Available: {[x.value for x in cls]}"
            )

    def __str__(self):
        return self.value