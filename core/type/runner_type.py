from core.type.string_enum import StringEnum

class RunnerType(StringEnum):
    INTERNAL = "internal_case_runner"
    EXTERNAL_RPG = "external_rpg_case_runner"