from core.type.string_enum import StringEnum

class SolverType(StringEnum):

    ROOT_FINDING = "root_finding"

    DIRECT_COLLOCATION = "direct_collocation"

    INDIRECT_COLLOCATION = "indirect_collocation"

    EXTERNAL = "external"



