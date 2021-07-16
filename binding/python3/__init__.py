from . import biorbd
from .biorbd import *
from ._version import __version__
from .surface_max_torque_actuator import *
from .rigid_body import *

if biorbd.currentLinearAlgebraBackend() == 1:
    from casadi import Function, MX, SX, horzcat

    def to_casadi_func(name, func, *all_param, expand=True):
        cx_param = []
        for p in all_param:
            if isinstance(p, (MX, SX)):
                cx_param.append(p)

        if isinstance(func, (MX, SX, Function)):
            func_evaluated = func
        else:
            func_evaluated = func(*all_param)
            if isinstance(func_evaluated, (list, tuple)):
                func_evaluated = horzcat(*[val if isinstance(val, MX) else val.to_mx() for val in func_evaluated])
            elif not isinstance(func_evaluated, MX):
                func_evaluated = func_evaluated.to_mx()
        func = Function(name, cx_param, [func_evaluated])
        return func.expand() if expand else func


def to_spatial_vector(f_ext: np.ndarray):
    """
    Converts a 6 x n_external_force np.array into biorbd spatial vector
    
    Parameters
    ----------
    f_ext: np.ndarray
        The array to convert

    Returns
    -------
    The conveted array
    """
    
    vector = biorbd.VecBiorbdSpatialVector()
    for idx in range(f_ext.shape[1]):
        vector.append(biorbd.SpatialVector(f_ext[:, idx]))
    return vector
