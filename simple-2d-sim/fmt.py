import math

def fmt_unit(val: float, base_unit: str) -> str:
    if math.isnan(val):
        return "(NaN) " + base_unit
    if math.isinf(val):
        if val > 0:
            return "(∞) " + base_unit
        else:
            return "(-∞) " + base_unit
    if val == 0:
        return "0" + base_unit
    exp = math.log(abs(val)) / math.log(10)
    exp_three = 3 * int(math.floor(exp / 3))

    mantissa = val * 10**-exp_three
    assert 1 <= abs(mantissa) < 1000

    sig = f"{mantissa:#.4g}"
    if mantissa >= 0:
        sig = " " + sig

    if exp_three < -9:
        return f"{sig} * 10^{exp_three} {base_unit}"
    elif exp_three == -9:
        return f"{sig}n{base_unit}"
    elif exp_three == -6:
        return f"{sig}μ{base_unit}"
    elif exp_three == -3:
        return f"{sig}m{base_unit}"
    elif exp_three == 0:
        return f"{sig}{base_unit}"
    elif exp_three == 3:
        return f"{sig}k{base_unit}"
    elif exp_three == 6:
        return f"{sig}M{base_unit}"
    elif exp_three == 9:
        return f"{sig}G{base_unit}"
    elif exp_three > 9:
        return f"{sig} * 10^{exp_three} {base_unit}"
    else:
        raise ValueError("wtf")
