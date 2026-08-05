from core.type.string_enum import StringEnum

class DynamicsType(StringEnum):
    AUG_DOUBLE_INTEGRATOR = "augmented_double_integrator"
    AUG_TRIPLE_INTEGRATOR = "augmented_triple_integrator"
    SIX_DOF = "6dof"