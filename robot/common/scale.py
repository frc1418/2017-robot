def scale(input, input_min, input_max, output_min, output_max):
    return ((input - input_min) / (input_max - input_min) * (output_max - output_min)) + output_min