
from os import getcwd, listdir
from os.path import isfile, splitext, split

__all__ = [f for f in listdir(getcwd()) if isfile(f) and splitext(f)[1] == '.py' and split(f)[1] != '__init__.py']
