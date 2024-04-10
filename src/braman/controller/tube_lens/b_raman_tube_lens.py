"""
BRamanTubeLens Class

Description:
    The BRamanTubeLens class manages configurations and properties of tube lenses used in the B-Raman platform.
    It allows setting tube lens properties such as name, maker, magnification, and focal length based on predefined configurations
    found in a CSV file. The class provides methods to retrieve and update these properties, as well as to fetch a comprehensive
    metadata dictionary describing the tube lens.

Author:
    Álvaro Fernández Galiana (alvaro.fernandezgaliana@gmail.com)

Date:
    2024-04-01

License:
    MIT License - Feel free to use and modify, but please keep this header.

Notes for Future Development:
    - Consider having all paths elsewhere.
"""

import pandas as pd
import os

config_folder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))), 'config')
tube_lens_list_csv_path = os.path.join(config_folder_path, 'B_Raman_Tube_Lens_List.csv')

class BRamanTubeLens:
    def __init__(self, name, maker=None, magnification=None, focal_length=None):
        """
        Initializes a BRamanTubeLens instance with specified or default properties.

        Args:
            name (str): The name of the tube lens.
            maker (str, optional): The manufacturer of the tube lens. Defaults to None.
            magnification (float, optional): The magnification factor of the tube lens. Defaults to None.
            focal_length (float, optional): The focal length of the tube lens in millimeters. Defaults to None.
        """
        self._name = name
        self._magnification = magnification
        self._maker = maker
        self._focal_length = focal_length  # Tube lens focal distance in mm
        self.set_tube_lens_from_name(self._name)
        self._metadata = self.set_metadata()


    def set_tube_lens_from_name(self, name, file_path=tube_lens_list_csv_path):
        """
        Sets the tube lens properties based on its name by looking up predefined configurations in a CSV file.

        Args:
            name (str): The name of the tube lens to look up.
            file_path (str): Path to the CSV file containing tube lens configurations.

        Raises:
            FileNotFoundError: If the specified CSV file cannot be found.
            Exception: If the tube lens name is not found in the database.
        """
        try:
            df = pd.read_csv(file_path)
        except FileNotFoundError:
            print(f"The file containing the laser information was not found in {file_path}. Please check the filename and try again.")
            raise

        if name in df['Name'].values:
            df = df[df['Name'] == name]
            self._magnification = float(df['Magnification'].values[0])
            self._maker = df['Maker'].values[0]
            self._focal_length = float(df['Focal_Length'].values[0])  # Tube lens focal distance in mm
        else:
            raise Exception(f'Tube lens name {name} is not in the database')


    def set_metadata(self):
        """
        Constructs and returns a metadata dictionary for the tube lens.

        Returns:
            dict: A dictionary containing metadata about the tube lens, including its name, maker, magnification, and focal length.
        """
        metadata = dict(Tube_Lens=self._name, Tube_Lens_Maker=self._maker, Tube_Lens_Magnification=self._magnification, Tube_Lens_Focal_Length=self._focal_length)
        self._metadata = metadata
        return metadata


    def get_metadata(self):
        """
        Retrieves the metadata dictionary for the tube lens.

        Returns:
            dict: A dictionary containing the tube lens's metadata, including its name, maker, magnification, and focal length.
        """
        return self._metadata

    def get_name(self):
        """
        Retrieves the name of the tube lens.

        Returns:
            str: The name of the tube lens.
        """
        return self._name

    def set_name(self, name):
        """
        Sets the name of the tube lens.

        Args:
            name (str): The new name for the tube lens.
        """
        self._name = name

    def get_magnification(self):
        """
        Retrieves the magnification factor of the tube lens.

        Returns:
            float: The magnification factor of the tube lens.
        """
        return self._magnification

    def set_magnification(self, magnification):
        """
        Sets the magnification factor of the tube lens.

        Args:
            magnification (float): The new magnification factor for the tube lens.
        """
        self._magnification = magnification

    def get_maker(self):
        """
        Retrieves the manufacturer of the tube lens.

        Returns:
            str: The manufacturer of the tube lens.
        """
        return self._maker

    def set_maker(self, maker):
        """
        Sets the manufacturer of the tube lens.

        Args:
            maker (str): The new manufacturer for the tube lens.
        """
        self._maker = maker

    def get_focal_length(self):
        """
        Retrieves the focal length of the tube lens.

        Returns:
            float: The focal length of the tube lens in millimeters.
        """
        return self._focal_length

    def set_focal_length(self, focal_length):
        """
        Sets the focal length of the tube lens.

        Args:
            focal_length (float): The new focal length for the tube lens, in millimeters.
        """
        self._focal_length = focal_length



if __name__ == '__main__':
    tube_lens = BRamanTubeLens('WFA4100')
    print(tube_lens.get_metadata())
