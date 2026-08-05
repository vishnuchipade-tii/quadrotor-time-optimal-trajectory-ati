
def clean_string(x):
    """
    Make any string LaTeX-safe and human readable.
    """

    if x is None:
        return x

    if isinstance(x, float) or isinstance(x, int):
        return x

    x = str(x)

    # core fix
    x = x.replace("_", " ")
    x = x.replace("-", " ")

    # normalize whitespace
    x = " ".join(x.split())

    return x


def clean_and_capitalize_string(x):
    """
    Make any string human readable and capitalize each word.
    """

    if x is None:
        return x

    if isinstance(x, (float, int)):
        return x

    x = str(x)

    # core fix
    x = x.replace("_", " ")
    x = x.replace("-", " ")

    # normalize whitespace
    x = " ".join(x.split())

    # capitalize each word
    x = x.title()

    return x