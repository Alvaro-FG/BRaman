# __init__.py for the braman package
import os

__version__ = '1.0.0'

config_folder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'config')

#from .controller import BRamanController

if __name__ == '__main__':
    print(config_folder_path)