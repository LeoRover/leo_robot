import sys
from builtins import input


def is_tool(name):
    """!
    Check whether an executable exists on PATH.
    @param name Name of the executable
    @return True if executable exists, False otherwise
    """
    from whichcraft import which

    return which(name) is not None


def query_yes_no(question, default="yes"):
    """!
    Ask a yes/no question via raw_input() and return their answer.
    @param question The question that is presented to the user.
    @param default The presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).
    @return True if the answer is "yes", False otherwise
    """
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            print("Please respond with 'yes' or 'no' " "(or 'y' or 'n').")


def write_flush(msg):
    """!Write a message to standard output and flush the buffer"""
    sys.stdout.write(msg)
    sys.stdout.flush()
