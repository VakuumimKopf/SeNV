from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange


def light_int_desc(desc):
    min_val = 0
    max_val = 255
    step = 1
    return ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description=desc,
                               integer_range=[IntegerRange(from_value=min_val, to_value=max_val,
                                                           step=step)])


def int_desc(desc):
    min_val = 0
    max_val = 1000
    step = 1
    return ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description=desc,
                               integer_range=[IntegerRange(from_value=min_val, to_value=max_val,
                                                           step=step)])


def float_desc(desc):
    min_val = 0.0
    max_val = 2.0
    step = 0.001
    return ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description=desc,
                               floating_point_range=[FloatingPointRange(
                                   from_value=min_val, to_value=max_val, step=step)])


def bool_desc(desc):
    return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description=desc)
