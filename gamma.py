import math
import subprocess


def gamma(num_steps, gamma):
    gammas = [math.pow(x, gamma) for x in range(num_steps)]
    return [x / max(gammas) for x in gammas]


def rescale_all(min_value, max_value, gammas):
    return [max(min_value, min(max_value, round(x * (max_value - min_value + 1)))) for x in gammas]


def main():
    gamma_coefficient = 2.3
    steps = 8
    gamma_values = rescale_all(1, 255, gamma(steps, gamma_coefficient))

    with open("src/led/gamma.rs", "w") as output:
        output.write("//! An auto-generated {}-step brightness table: gamma = {}\n\n".format(steps, gamma_coefficient))
        output.write("pub const GAMMA_TABLE: &[u8; {}] = &[\n".format(steps))
        for value in gamma_values:
            output.write("\t %d,\n" % value)
        output.write("];\n\n")
        output.write("pub const GAMMA_SUM_WEIGHT: f32 = {};\n".format(float(sum(gamma_values))))

    subprocess.check_call(["cargo", "fmt"])


if __name__ == "__main__":
    main()
