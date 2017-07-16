def scale(input, input_min, input_max, output_min, output_max):
    """
    Scales an input from one range to another
    
    :param input: The input on the input range
    :param input_min: The minimum of the input range
    :param input_max: The maximum of the input range
    :param output_min: The minimum of the output range
    :param output_max: The maximum of the output range
    
    :return: The input value scaled to the output range.
    """
    return ((input - input_min) / (input_max - input_min) * (output_max - output_min)) + output_min