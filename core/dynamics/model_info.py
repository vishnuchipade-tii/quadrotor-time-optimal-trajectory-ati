from dataclasses import dataclass


@dataclass(frozen=True)
class ModelInfo:
    model_id: str
    display_name: str
    directory_name: str