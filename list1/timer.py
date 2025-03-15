from time import time
from datetime import datetime
from const import DATE_FORMAT


def check_time(func):
    def aux(*args, **kwargs):
        s = time()

        res = func(*args, **kwargs)

        e = time()

        elapsed_time = (e - s) * 1000
        print(f"It took: {elapsed_time:.2f} ms")

        return res

    return aux


def str_to_time(s: str) -> time:
    return datetime.strptime(s, DATE_FORMAT).time()
