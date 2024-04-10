"""
BRamanObjective Class

Description:
    The BRamanObjective class manages configurations and properties of objectives used in the B-Raman spectroscopy setup.
    It allows setting objective properties such as name, maker, magnification, numerical aperture (NA), working distance (WD),
    immersion medium, and designed focal length of the tube lens based on predefined configurations found in a CSV file.
    The class provides methods to retrieve and update these properties, as well as to fetch a comprehensive metadata dictionary
    describing the objective.

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

# Paths to configuration
config_folder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))), 'config')
objective_list_csv_path = os.path.join(config_folder_path, 'B_Raman_Objectives_List.csv')

class BRamanObjective:
    def __init__(self, name, maker=None, magnification=None, NA=None, WD=None, immersion=None, f_tube_lens_design=None):
        """
        Initializes a BRamanObjective instance with specified or default properties.

        Args:
            name (str): The name of the objective.
            maker (str, optional): The manufacturer of the objective. Defaults to None.
            magnification (float, optional): The magnification factor of the objective. Defaults to None.
            NA (float, optional): The numerical aperture of the objective. Defaults to None.
            WD (float, optional): The working distance of the objective in millimeters. Defaults to None.
            immersion (str, optional): The type of immersion medium used with the objective (e.g., "Water", "Oil"). Defaults to None.
            f_tube_lens_design (float, optional): The designed focal length of the tube lens in millimeters used with the objective. Defaults to None.
        """
        self._name = name
        self._magnification = magnification
        self._maker = maker
        self._WD = WD  # Working distance in mm
        self._NA = NA
        self._f_tube_lens_design = f_tube_lens_design  # Design Tube lens focal distance in mm
        self._immersion = immersion
        self.set_objective_from_name(self._name)
        self._metadata = self.set_metadata()

    def set_objective_from_name(self, name, file_path=objective_list_csv_path):
        """
        Sets the objective properties based on its name by looking up predefined configurations in a CSV file.

        Args:
            name (str): The name of the objective to look up.
            file_path (str): Path to the CSV file containing objective configurations.

        Raises:
            FileNotFoundError: If the specified CSV file cannot be found.
            Exception: If the objective name is not found in the database.
        """
        # Implementation for setting objective based on the name
        # read the csv file
        try:
            df = pd.read_csv(file_path)
        except FileNotFoundError:
            print(f"The file containing the objective information was not found in {file_path}. Please check the filename and try again.")
            raise

        # check if the name exists in the 'name' column
        if name in df['Name'].values:
            df = df[df['Name'] == name]
            self._magnification =  float(df['Magnification'].values[0])
            self._NA = float(df['NA'].values[0])
            self._maker = df['Maker'].values[0]
            self._WD = float(df['WD'].values[0]) # Working distance in mm
            self._f_tube_lens_design = float(df['Tube_Lens_Design'].values[0])  # Design tube lens focal distance in mm
            self._immersion = df['Immersion'].values[0]
        else:
            raise Exception(f'Objective name {name} is not in the database')

    def set_metadata(self):
        """
        Constructs and returns a metadata dictionary for the objective.

        Returns:
            dict: A dictionary containing metadata about the objective, including its name, maker, magnification, NA, WD, immersion medium, and designed tube lens focal length.
        """
        # Implementation for setting metadata
        metadata = dict(Objective=self._name, Objective_Maker=self._maker, Objective_Magnification=self._magnification, Objective_NA=self._NA,
                             Objective_WD=self._WD, Objective_Immersion=self._immersion, Objective_Tube_Lens_f=self._f_tube_lens_design)
        self._metadata = metadata
        return metadata

    def get_metadata(self):
        """
        Retrieves the metadata dictionary for the objective.

        Returns:
            dict: A dictionary containing the objective's metadata.
        """
        return self._metadata


def get_name(self):
    """
    Retrieves the name of the objective.

    Returns:
        str: The name of the objective.
    """
    return self._name

def set_name(self, name):
    """
    Sets the name of the objective.

    Args:
        name (str): The new name for the objective.
    """
    self._name = name

def get_magnification(self):
    """
    Retrieves the magnification factor of the objective.

    Returns:
        float: The magnification factor of the objective.
    """
    return self._magnification

def set_magnification(self, magnification):
    """
    Sets the magnification factor of the objective.

    Args:
        magnification (float): The new magnification factor for the objective.
    """
    self._magnification = magnification

def get_maker(self):
    """
    Retrieves the manufacturer of the objective.

    Returns:
        str: The manufacturer of the objective.
    """
    return self._maker

def set_maker(self, maker):
    """
    Sets the manufacturer of the objective.

    Args:
        maker (str): The new manufacturer for the objective.
    """
    self._maker = maker

def get_NA(self):
    """
    Retrieves the numerical aperture (NA) of the objective.

    Returns:
        float: The numerical aperture of the objective.
    """
    return self._NA

def set_NA(self, NA):
    """
    Sets the numerical aperture (NA) of the objective.

    Args:
        NA (float): The new numerical aperture for the objective.
    """
    self._NA = NA

def get_WD(self):
    """
    Retrieves the working distance (WD) of the objective in millimeters.

    Returns:
        float: The working distance of the objective.
    """
    return self._WD

def set_WD(self, WD):
    """
    Sets the working distance (WD) of the objective.

    Args:
        WD (float): The new working distance for the objective, in millimeters.
    """
    self._WD = WD

def get_f_tube_lens_design(self):
    """
    Retrieves the designed focal length of the tube lens used with the objective in millimeters.

    Returns:
        float: The designed focal length of the tube lens.
    """
    return self._f_tube_lens_design

def set_f_tube_lens_design(self, f_tube_lens_design):
    """
    Sets the designed focal length of the tube lens used with the objective.

    Args:
        f_tube_lens_design (float): The new designed focal length for the tube lens, in millimeters.
    """
    self._f_tube_lens_design = f_tube_lens_design

def get_immersion(self):
    """
    Retrieves the type of immersion medium used with the objective.

    Returns:
        str: The type of immersion medium.
    """
    return self._immersion

def set_immersion(self, immersion):
    """
    Sets the type of immersion medium used with the objective.

    Args:
        immersion (str): The new immersion medium for the objective.
    """
    self._immersion = immersion


if __name__ == '__main__':
    objective = BRamanObjective('N PLAN 10x/0.25')
    print(objective.get_metadata())
    objective = BRamanObjective('N PLAN 10x/0.25_2')
    print(objective.get_metadata())