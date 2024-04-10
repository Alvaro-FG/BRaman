"""
MCM3000Controller Class

This class provides an interface to control the Thorlabs MCM3000 and MCM3001 3-axis motor controllers,
enabling precise positioning and movement of motorized stages. It supports initializing motor stages,
managing motion direction, verbosity of operational messages, and handling of minimum motion constraints.

Based on original code by amskikking (https://github.com/amsikking/thorlabs_MCM3000) with modifications
and enhancements for additional functionality and usability improvements.

Author: Alvaro Fernandez Galiana (alvaro.fernandezgaliana@gmail.com)

License: GNU General Public License (GPL)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see
<http://www.gnu.org/licenses/>.

Notes for development:
    - Improve the issue of the minimum motion and the finish move, sometimes it stops close to the target but not exactly there, and then waiting for the finish move it gets stuck
"""


import time
import serial

supported_stages = { # 'Type': (+-stage_limit_um, stage_conversion_um) from MCM300_Direct_Serial_Communication.pdf
    # The stage limit is set such that the allowed positions will be between +stage_limit_um and -stage_limit_um
    'ZFM2020':(1e3 * 12.7, 0.2116667),
    #'ZFM2030':(1e3 * 12.7, 0.2116667),
}


class MCM3000Controller:
    """
    Controller for the Thorlabs MCM3000 motor controller.

    This class provides interfaces to control motor stages, including initialization
    of the motor stages, moving them to specific encoder values or positions, setting
    encoder values to zero, and more. The controller supports operations such as reversing
    the motion direction, setting verbose output levels, and managing minimum motion constraints.

    Attributes:
        name (str): Name of the controller.
        supported_stages (dict): A dictionary mapping stage names to their specifications.
        verbose (bool): If True, prints basic operation info to the terminal.
        very_verbose (bool): If True, prints extended operation info to the terminal.
        stages (tuple): Names of the stage types for each channel.
        channels (tuple): Labels of each control channel.
        reverse (tuple): Indicates if the motion direction for each axis is reversed.
        reverse_factors (list): Factors to apply to reverse motion direction, derived from `reverse`.
        _internal_channels (tuple): Internal mapping of channels, used for indexing.
        _internal_channels_dict (dict): Maps external channel labels to internal channel indices.
        _stage_upper_limit_um (list): Upper motion limit for each stage, in micrometers.
        _stage_lower_limit_um (list): Lower motion limit for each stage, in micrometers.
        _stage_lowest_scan_point_um (list): Safe lower bound for scanning, to avoid damaging the sample.
        _stage_highest_scan_point_um (list): Safe upper bound for scanning.
        _stage_retract_point_um (list): Retract points before engaging in X-Y motion.
        _min_encoder_motion (int): Minimum number of encoder counts required for motion to be recognized.
        _stage_conversion_um (list): Conversion factors from encoder counts to micrometers for each stage.
        _current_encoder_value (list): Current encoder values for each channel.
        _pending_encoder_value (list): Target encoder values during motion, becomes None when motion finishes.

    Raises:
        IOError: If no connection can be established on the provided port.
        AssertionError: For various conditions such as incorrect types or values for `stages`, `reverse`, and `channels`.
    """

    def __init__(self,
                 port, # This is 'COMX' if using USB connection
                 name='MCM3000', # Name on controller
                 stages=3*(None,), # Defines the name of the type of stages (i.e. 'ZFM2020')
                 reverse=3*(False,), # States if the axis are reversed (i.e. down is positive)
                 channels = (1, 2, 3), # Label of each channel, WATCH OUT BECAUSE IN PRACTICE THEY ARE LABELLED 1,2,3
                 verbose=True, # If True, info in printed in terminal
                 very_verbose=False): # If True, more info in printed in terminal
        """
        Initializes the MCM3000Controller with the specified configuration.

        Args:
            port (str): Identifier for the communication port (e.g., 'COMX' for USB connections).
            name (str, optional): Name of the controller. Defaults to 'MCM3000'.
            stages (tuple, optional): Names of the stage types for each channel. Defaults to (None, None, None).
            reverse (tuple, optional): Indicates if the motion direction for each axis is reversed. Defaults to (False, False, False).
            channels (tuple, optional): Labels of each control channel, should be in the order (1, 2, 3). Defaults to (1, 2, 3).
            verbose (bool, optional): If True, prints basic operation info to the terminal. Defaults to True.
            very_verbose (bool, optional): If True, prints extended operation info to the terminal. Defaults to False.
        """
        self.name = name
        self.supported_stages = supported_stages
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print(f"{self.name}: opening...", end='')
        try:
            self.port = serial.Serial(port=port, baudrate=460800)
        except serial.serialutil.SerialException:
            raise IOError(
                f'{self.name}: no connection on port {port}')
        if self.verbose: print("done.")
        assert type(stages) == tuple and type(reverse) == tuple and type(channels) == tuple, (f'{self.name}: stages, reverse and channels must be a tuple, currently {type(stages)}, {type(reverse)}, {type(channels)}')
        assert len(stages) == 3 and len(reverse) == 3 and len(channels) == 3, (f'{self.name}: stages, reverse and channels must be a tuple of 3 elements, currently {len(stages)}, {len(reverse)}, {len(channels)}')
        for element in reverse: assert type(element) == bool, (f'{self.name}: reverse must be a tuple of booleans')
        for element in stages:
            if element is not None: assert element in self.supported_stages, (f'{self.name}: stage \'{element}\' not tested for {self.name} controller!!')
        self.stages = stages
        self.channels = channels
        self._internal_channels = (0, 1, 2) # This is for internal use
        self._internal_channels_dict = dict(zip(self.channels, self._internal_channels)) #This is for internal use
        self.reverse = reverse
        self.reverse_factors = len(reverse)*[1,]
        for ii in range(len(reverse)):
            if self.reverse[ii]:
                self.reverse_factors[ii] = -1
        # The lowest and highest range of the stage
        self._stage_upper_limit_um = 3*[None]
        self._stage_lower_limit_um = 3*[None]
        # The lowest and highest scan points (i.e. the lowest point before damaging the sample being scanned)
        self._stage_lowest_scan_point_um = 3*[None]
        self._stage_highest_scan_point_um = 3*[None]
        # The level at which to retract before engaging in X-Y motion
        self._stage_retract_point_um = 3*[None]
        # The minimum number of counts that it can move (for very small motions it struggles)
        self._min_encoder_motion = 5
        # Conversion factor: um/count
        self._stage_conversion_um = 3*[None]
        self._current_encoder_value = 3*[None]
        self._pending_encoder_value = 3*[None] # Is None when all motions have finished but, while in motion, it is the target encoder value (until it is reached and becomes None)

        for channel, stage in enumerate(stages):
            if stage is not None:
                assert stage in self.supported_stages, (f'{self.name}: stage \'{stage}\' not supported')
                # Set lower and upper limit (these are the max and min points that the stage can physically do)
                self._stage_upper_limit_um[channel] = self.supported_stages[stage][0]*self.reverse_factors[channel]
                self._stage_lower_limit_um[channel] = -self.supported_stages[stage][0]*self.reverse_factors[channel]
                # Set conversion factor
                self._stage_conversion_um[channel] = self.supported_stages[stage][1]
                # Set scan points and retract points (for initializing they are just the same as max points)
                self._stage_highest_scan_point_um[channel] = self.supported_stages[stage][0]*self.reverse_factors[channel]
                self._stage_lowest_scan_point_um[channel] = -self.supported_stages[stage][0]*self.reverse_factors[channel]
                self._stage_retract_point_um[channel] = self._stage_highest_scan_point_um[channel]-self._stage_conversion_um[channel]*self.reverse_factors[channel]
                self._current_encoder_value[channel] = self._get_encoder_value(self.channels[self._internal_channels.index(channel)], True)

        if self.verbose:
            print(f'\n{self.name}: stages:', self.stages)
            print(f'\n{self.name}: channels:', self.channels)
            print(f'\n{self.name}: reverse:', self.reverse)
            print(f'\n{self.name}: reverse factors', self.reverse_factors)
            print(f'\n{self.name}: stage_upper_limit_um:', self._stage_upper_limit_um)
            print(f'\n{self.name}: stage_lower_limit_um:', self._stage_lower_limit_um)
            print(f'\n{self.name}: stage_conversion_um:', self._stage_conversion_um)
            print(f'\n{self.name}: current_encoder_value:', self._current_encoder_value)

    def _send(self, cmd, channel, response_bytes=None):
        """
        Sends a command to a specific channel of the motor controller.

        This method sends a command to the motor controller associated with a specific
        channel and optionally reads a response. It checks if the stage for the given
        channel is initialized before sending the command.

        Args:
            cmd (bytes): The command to be sent to the motor controller.
            channel (int): The channel to which the command is sent.
            response_bytes (int, optional): The number of bytes to read from the response. If None, no response is read.

        Returns:
            bytes or None: The response from the controller if `response_bytes` is not None, otherwise None.

        Raises:
            AssertionError: If the stage for the specified channel is not initialized.
        """
        assert self.stages[self._internal_channels_dict[channel]] is not None, (f'{self.name}: channel {channel}: stage = None (cannot send command)')
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        return response

    def _get_encoder_value(self, channel, verbose = False):
        """
        Retrieves the current encoder value for a specified channel.

        This method sends a command to the motor controller to get the current encoder
        value of the stage associated with the specified channel. It validates if the
        channel is available and ensures that the response is correctly associated with
        the requested channel before returning the encoder value.

        Args:
            channel (int): The channel from which to retrieve the encoder value.
            verbose (bool, optional): If True, prints the retrieved encoder value to the terminal. Defaults to False.

        Returns:
            int: The current encoder value for the specified channel.

        Raises:
            AssertionError: If the specified channel is not available.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        channel_byte = self._internal_channels_dict[channel].to_bytes(1, byteorder='little')
        cmd = b'\x0a\x04' + channel_byte + b'\x00\x00\x00'
        response = self._send(cmd, channel, response_bytes=12)
        assert response[6:7] == channel_byte # channel = selected
        encoder_value = int.from_bytes(
            response[-4:], byteorder='little', signed=True)
        if verbose:
            print(f'\n{self.name}: ch{channel} -> stage encoder value = {encoder_value}')
        return encoder_value

    def _reverse_limit_signs(self, channel):
        """
        Reverses the sign of the motion limit values for a specified channel.

        This method modifies the upper and lower motion limit values, along with the
        highest and lowest scan point values, and the retract point value for the
        specified channel by reversing their signs. This is useful in scenarios where
        the direction of motion needs to be inverted.

        Args:
            channel (int): The channel for which to reverse the limit signs.

        Raises:
            AssertionError: If the specified channel is not available.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        # Set lower and upper limit (these are the max and min points that the stage can physically do)
        self._stage_upper_limit_um[self._internal_channels_dict[channel]] = -self._stage_upper_limit_um[self._internal_channels_dict[channel]]
        self._stage_lower_limit_um[self._internal_channels_dict[channel]] = -self._stage_lower_limit_um[self._internal_channels_dict[channel]]
        # Set scan points and retract points (for initializing they are just the same as max points)
        self._stage_highest_scan_point_um[self._internal_channels_dict[channel]] = -self._stage_highest_scan_point_um[self._internal_channels_dict[channel]]
        self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]] = -self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]]
        self._stage_retract_point_um[self._internal_channels_dict[channel]] = -self._stage_retract_point_um[self._internal_channels_dict[channel]]


    def _set_encoder_value_to_zero(self, channel):
        """
        Sets the current position as zero for the encoder count on a specified channel.

        This method resets the encoder value to zero for the given channel, effectively
        recalibrating the position measurement. It warns that after zeroing, the limits
        are no longer valid and should be reset unless the zeroing is done at the center
        of its range. The method waits until the encoder value is confirmed to be reset
        to zero before returning.

        Args:
            channel (int): The channel for which to set the encoder value to zero.

        Returns:
            None

        Raises:
            AssertionError: If the specified channel is not available.

        Note:
            After zeroing, it's essential to reset the motion limits, unless the zeroing
            operation was performed at the center of the stage's range.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if self.verbose:
            encoder_value = self._get_encoder_value(channel, verbose = self.verbose)
        channel_byte = self._internal_channels_dict[channel].to_bytes(2, byteorder='little')
        encoder_bytes = (0).to_bytes(4, 'little', signed=True) # set to zero
        cmd = b'\x09\x04\x06\x00\x00\x00' + channel_byte + encoder_bytes
        self._send(cmd, channel)
        if self.verbose:
            print(f'{self.name}: ch{channel} -> waiting for re-set to zero')
        while True:
            if self._get_encoder_value(channel, verbose=self.verbose) == 0: break
        self._current_encoder_value[self._internal_channels_dict[channel]] = 0
        if self.verbose:
            print(f'{self.name}: ch{channel} -> done with encoder re-set')
        return None

    def _move_to_encoder_value(self, channel, encoder_value, block=True):
        """
        Moves the stage encoder to the specified value for the given channel.

        Args:
            channel (str): The channel to move the stage encoder for.
            encoder_value (int): The value to move the stage encoder to.
            block (bool, optional): Whether to block until the move is finished. Defaults to True.

        Returns:
            None
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if self._pending_encoder_value[self._internal_channels_dict[channel]] is not None:
            self._finish_move(channel)
        encoder_bytes = encoder_value.to_bytes(4, 'little', signed=True)
        channel_bytes = self._internal_channels_dict[channel].to_bytes(2, byteorder='little')
        cmd = b'\x53\x04\x06\x00\x00\x00' + channel_bytes + encoder_bytes
        self._send(cmd, channel)
        self._pending_encoder_value[self._internal_channels_dict[channel]] = encoder_value
        if self.very_verbose:
            print(f'{self.name}: ch{channel} -> moving stage encoder to value = {encoder_value}')
        if block:
            self._finish_move(channel)
        return None

    def _finish_move(self, channel, polling_wait_s=0.1, verbose=False):
        """
        Finish the movement of the specified channel and update the current encoder value and position.

        Args:
            channel (str): The channel to finish the movement for.
            polling_wait_s (float, optional): The time to wait between polling the encoder value. Defaults to 0.1.
            verbose (bool, optional): Whether to print verbose output. Defaults to False.

        Returns:
            tuple: A tuple containing the current encoder value and the current position in um.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if self._pending_encoder_value[self._internal_channels_dict[channel]] is None:
            return
        current_encoder_value = self._get_encoder_value(channel, verbose = False)
        timeout = time.time() + 6 # 6s timeout, measured to be max time between top and bottom motions
        while True:
            if current_encoder_value == self._pending_encoder_value[self._internal_channels_dict[channel]]: #FIRST VERSION
            # if abs(current_encoder_value - self._pending_encoder_value[self._internal_channels_dict[channel]]) <= 1 : # Check that error is smaller than resolution (i.e. one encoder value)
                break
            if time.time() > timeout:
                print(f"\033[93m\nMCM3000 motion timed out\033[0m") # TODO double check requirements that is close enough
                position_error = current_encoder_value - self._pending_encoder_value[self._internal_channels_dict[channel]] # position error in enconder counts
                if abs(position_error) > 1:
                    print(f"\033[91m\nMCM3000 position error: {position_error} counts \033[0m")
                break
            if verbose: print('.', end='')
            time.sleep(polling_wait_s)
            current_encoder_value = self._get_encoder_value(channel, verbose = False)
        self._current_encoder_value[self._internal_channels_dict[channel]] = current_encoder_value
        current_position_um = self._um_from_encoder_value(
            channel, current_encoder_value)
        if verbose:
            print(f'\n{self.name}: ch{channel} -> finished moving to position_um = {current_position_um}')
        self._pending_encoder_value[self._internal_channels_dict[channel]] = None
        return current_encoder_value, current_position_um

    def _um_from_encoder_value(self, channel, encoder_value):
        """
        Converts the encoder value to micrometers (um) based on the channel and the stage conversion factor.

        Args:
            channel (str): The channel of the encoder value.
            encoder_value (float): The encoder value to be converted.

        Returns:
            float: The converted value in micrometers (um).
        """
        um = encoder_value * self._stage_conversion_um[self._internal_channels_dict[channel]]
        if self.reverse[self._internal_channels_dict[channel]]:
            um = -um
        return um + 0  # avoid -0.0

    def _encoder_value_from_um(self, channel, um):
        """
        Converts a distance in micrometers (um) to the corresponding encoder value for the specified channel.

        Args:
            channel (str): The channel for which to calculate the encoder value.
            um (float): The distance in micrometers to convert.

        Returns:
            int: The encoder value corresponding to the given distance in micrometers.
        """
        encoder_value = int(um / self._stage_conversion_um[self._internal_channels_dict[channel]])
        if self.reverse[self._internal_channels_dict[channel]]:
            encoder_value = -encoder_value
        return encoder_value + 0  # avoid -0.0

    def _check_min_motion(self, channel, target_encoder_value):
        """
        Check if the desired motion is smaller than the minimum motion and, if so, move out to be able to move back in.

        Args:
            channel (str): The channel to check the motion for.
            target_encoder_value (float): The target encoder value to check against.

        Returns:
            bool: True if it is necessary to move, False otherwise.
        """
        if self._pending_encoder_value[self._internal_channels_dict[channel]]:
            if target_encoder_value == self._pending_encoder_value[self._internal_channels_dict[channel]]:
                return False
            elif abs(target_encoder_value - self._pending_encoder_value[self._internal_channels_dict[channel]]) <= self._min_encoder_motion:
                self.move_um(channel, 10, relative=True, block=True, verbose=False)
        else:
            if target_encoder_value == self._current_encoder_value[self._internal_channels_dict[channel]]:
                return False
            elif abs(target_encoder_value - self._current_encoder_value[self._internal_channels_dict[channel]]) <= self._min_encoder_motion:
                self.move_um(channel, 10, relative=True, block=True, verbose=False)
        return True


    def get_position_um(self, channel, verbose=False):
        """
        Get the position in micrometers (um) for the specified channel.

        Args:
            channel (str): The channel for which to get the position.
            verbose (bool, optional): If True, print the stage position in micrometers. Defaults to False.

        Returns:
            float: The stage position in micrometers.

        Raises:
            AssertionError: If the specified channel is not available.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        encoder_value = self._get_encoder_value(channel, verbose=False)
        position_um = self._um_from_encoder_value(channel, encoder_value)
        if verbose:
            print(f'{self.name}: ch{channel} -> stage position_um = {position_um}')
        return position_um

    def set_stage_limit_um(self, channel, lower_limit=True, limit_um=None):
        """
        Set the limit of a stage motion for a specified channel.

        Args:
            channel (str): The channel for which to set the limit.
            lower_limit (bool, optional): Whether to set the lower limit or the upper limit. Defaults to True.
            limit_um (float, optional): The limit value in micrometers. If not provided, the current position is used.

        Returns:
            float: The target limit value in micrometers.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        # Set the limit of a stage motion for a specified channel
        if limit_um:
            target_limit_um = limit_um
        else:
            target_limit_um = self.get_position_um(channel, verbose = self.very_verbose)
        if self.reverse[self._internal_channels_dict[channel]]:
            assert self._stage_lower_limit_um[self._internal_channels_dict[channel]] >= target_limit_um >= self._stage_upper_limit_um[self._internal_channels_dict[channel]], (
                f'{self.name}: ch{channel} -> requested limit ({target_limit_um}) exceeds the stage limits ([{self._stage_lower_limit_um[self._internal_channels_dict[channel]]},{self._stage_upper_limit_um[self._internal_channels_dict[channel]]}])')
        else:
            assert self._stage_lower_limit_um[self._internal_channels_dict[channel]] <= target_limit_um <= self._stage_upper_limit_um[self._internal_channels_dict[channel]], (
                f'{self.name}: ch{channel} -> requested limit ({target_limit_um}) exceeds the stage limits ([{self._stage_lower_limit_um[self._internal_channels_dict[channel]]},{self._stage_upper_limit_um[self._internal_channels_dict[channel]]}])')
        if lower_limit:
            self._stage_lowest_scan_point_um = target_limit_um
            if self.verbose:
                print(f'{self.name}: ch{channel} -> stage lowest scan point set to: {target_limit_um} um' )
        else:
            self._stage_highest_scan_point_um = target_limit_um
            if self.verbose:
                print(f'{self.name}: ch{channel} -> stage highest scan point set to: {target_limit_um} um' )
            if self.reverse[self._internal_channels_dict[channel]]:
                if self._stage_retract_point_um < self._stage_highest_scan_point_um:
                    self.set_retract_point_um(self, channel, self._stage_highest_scan_point_um[self._internal_channels_dict[channel]])
            else:
                if self._stage_retract_point_um > self._stage_highest_scan_point_um:
                    self.set_retract_point_um(self, channel, self._stage_highest_scan_point_um[self._internal_channels_dict[channel]])
        return target_limit_um


    def get_retract_point_um(self, channel, verbose=False):
        """
        Get the stage retract point in micrometers for the specified channel.

        Parameters:
            channel (str): The channel for which to retrieve the stage retract point.
            verbose (bool): If True, print the stage retract point to the console.

        Returns:
            float: The stage retract point in micrometers for the specified channel.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if verbose:
            print(f'{self.name}: ch{channel} -> stage retract point = {self._stage_retract_point_um[self._internal_channels_dict[channel]]} um')
        return self._stage_retract_point_um[self._internal_channels_dict[channel]]

    def set_retract_point_um(self, channel, retract_um_pos=None, relative=False, verbose=False):
        """
        Set the retract position of a stage motion for a specified channel.

        Args:
            channel (str): The channel for which to set the retract position.
            retract_um_pos (float, optional): The retract position in micrometers. If not provided, the current position will be used.
            relative (bool, optional): Whether the retract position is relative to the current position. Default is False.
            verbose (bool, optional): Whether to print verbose output. Default is False.

        Returns:
            float: The target retract position in micrometers.
        """

        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        # Set the retract position of a stage motion for a specified channel
        if retract_um_pos:
            target_retract_um = retract_um_pos
            if relative:
                target_retract_um = target_retract_um + self.get_position_um(channel)
        else:
            target_retract_um = self.get_position_um(channel)
        if self.reverse[self._internal_channels_dict[channel]]:
            assert self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]] <= target_retract_um <= self._stage_highest_scan_point_um[self._internal_channels_dict[channel]], (
                f'{self.name}: ch{channel} -> requested retract point ({target_retract_um}) exceeds the stage limits ([{self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]]},{self._stage_highest_scan_point_um[self._internal_channels_dict[channel]]}])')
        else:
            assert self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]] >= target_retract_um >= self._stage_highest_scan_point_um[self._internal_channels_dict[channel]], (
                f'{self.name}: ch{channel} -> requested retract point ({target_retract_um}) exceeds the stage limits ([{self._stage._lowest_scan_point_um[self._internal_channels_dict[channel]]},{self._stage_highest_scan_point_um[self._internal_channels_dict[channel]]}])')
        self._stage_retract_point_um = target_retract_um
        if verbose:
            print(f'{self.name}: ch{channel} -> stage retract point set to: {target_retract_um} um' )
        return target_retract_um

    def legalize_move_um(self, channel, move_um, relative=True, verbose=True):
        """
        Checks if the desired motion is within the accepted boundaries and returns the legalized move in micrometers (um).

        Args:
            channel (str): The channel to perform the motion on.
            move_um (float): The desired motion in micrometers (um).
            relative (bool, optional): Whether the motion is relative or absolute. Defaults to True.
            verbose (bool, optional): Whether to print verbose output. Defaults to True.

        Returns:
            float: The legalized move in micrometers (um) if it is within the boundaries, None otherwise.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        target_encoder_value = self._encoder_value_from_um(channel, move_um)
        if relative:
            if self._pending_encoder_value[self._internal_channels_dict[channel]]:
                target_encoder_value += self._pending_encoder_value[self._internal_channels_dict[channel]]
            else:
                target_encoder_value += self._current_encoder_value[self._internal_channels_dict[channel]]
        # Check that want to move beyond the minimum number of counts and, if not, do the motion back and forth
        if not self._check_min_motion(channel, target_encoder_value): return None
        # Set the target motion in um
        target_move_um = self._um_from_encoder_value(channel, target_encoder_value) # This might produce a slight difference between move_un and target, due to the need to have an encoder value, but since afterwards it checks if the error is smaller than the resolution, all good
        # Check if there is a lowest scanning point, if not it refers to the limit
        if self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]]:
            lower_limit_um = self._stage_lowest_scan_point_um[self._internal_channels_dict[channel]]
        else:
            lower_limit_um = self._stage_lower_limit_um[self._internal_channels_dict[channel]]
        # Check if there is a highest scanning point, if not it refers to the limit
        if self._stage_highest_scan_point_um[self._internal_channels_dict[channel]]:
            upper_limit_um = self._stage_highest_scan_point_um[self._internal_channels_dict[channel]]
        else:
            upper_limit_um = self._stage_upper_limit_um[self._internal_channels_dict[channel]]
        # Check that value is within boundaries
        if self.reverse[self._internal_channels_dict[channel]]:
            assert lower_limit_um >= target_move_um >= upper_limit_um, (
                f'{self.name}: ch{channel} -> requested move_um ({target_move_um}) exceeds the limit_um ([{lower_limit_um},{upper_limit_um}])')
        else:
            assert lower_limit_um <= target_move_um <= upper_limit_um, (
                f'{self.name}: ch{channel} -> requested move_um ({target_move_um}) exceeds the limit_um ([{lower_limit_um},{upper_limit_um}])')
        legal_move_um = target_move_um
        if verbose:
            print(f'{self.name}: ch{channel} -> legalized move_um = {legal_move_um}'  +
                  f'({move_um} requested, relative={relative})')
        return legal_move_um

    def move_um(self, channel, move_um, relative=True, block=True, verbose=False):
        """
        Moves the specified channel by the given distance in micrometers.

        Args:
            channel (str): The channel to move.
            move_um (float): The distance to move in micrometers.
            relative (bool, optional): If True, the movement is relative to the current position. If False, the movement is absolute. Defaults to True.
            block (bool, optional): If True, the method blocks until the movement is completed. If False, the method returns immediately. Defaults to True.
            verbose (bool, optional): If True, additional information about the movement is printed. Defaults to False.

        Returns:
            float: The actual distance moved in micrometers.

        Raises:
            AssertionError: If the specified channel is not available.

        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        legal_move_um = self.legalize_move_um(channel, move_um, relative, verbose=verbose)
        if not legal_move_um: # If there is no need to move
            if self.verbose:
                print(f'No need to move, already in position {self.get_position_um(channel, verbose = False)} um ({move_um} um was requested))')
            return
        if self.verbose:
            print(f'{self.name}: ch{channel} -> moving to position_um = {legal_move_um}', end='\n')
        encoder_value = self._encoder_value_from_um(channel, legal_move_um)
        self._move_to_encoder_value(channel, encoder_value, block)
        if block:
            self._finish_move(channel, verbose=verbose)
        else:
            print('\n')
        if verbose:
            print(f"Channel {channel} in position {self.get_position_um(channel, False)}")
        return legal_move_um

    def move_zero(self, channel, block=True):
        """
        Moves the specified channel to the zero position.

        Args:
            channel (str): The channel to move to zero.
            block (bool, optional): Whether to block until the movement is complete. Defaults to True.

        Returns:
            bool: True if the movement was successful, False otherwise.
        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if self.verbose:
            print(f'{self.name}: ch{channel} -> Moving to Zero', end='')
        return self.move_um(channel, 0, relative=False, block=block)

    def retract(self, channel):
        """
        Retracts the specified channel of the MCM3000 controller.

        Args:
            channel (str): The channel to retract.

        Raises:
            AssertionError: If the specified channel is not available.

        """
        assert channel in self.channels, (f'{self.name}: channel \'{channel}\' not available')
        if self.verbose:
            print(f'{self.name}: ch{channel} -> Moving to RETRACT position', end='\n')
        self.move_um(channel, self._stage_retract_point_um[self._internal_channels_dict[channel]]*self.reverse_factors[self._internal_channels_dict[channel]], relative=False)


    def close(self):
        """
        Closes the connection to the MCM3000 controller.

        This method closes the connection to the MCM3000 controller by closing the port.
        """
        if self.verbose: print(f'{self.name}: closing...', end=' ')
        self.port.close()
        if self.verbose: print(f"{self.name} CLOSED.")
        return None

if __name__ == '__main__':
    chnl = 1 # Remember starts at 1
    stage_controller = MCM3000Controller(
        'COM3',stages=('ZFM2020', None, None), reverse=(True, False, False))

    # Read position
    print('\n# Get position:')
    stage_pos = stage_controller.get_position_um(chnl, verbose = True)

    print(stage_controller._stage_retract_point_um[chnl])
    print(stage_controller._stage_upper_limit_um[chnl])
    print(stage_controller._stage_lower_limit_um[chnl])
    print(stage_controller._stage_lowest_scan_point_um[chnl])
    print(stage_controller._stage_highest_scan_point_um[chnl])

    #stage_controller.move_um(chnl, stage_controller._stage_retract_point_um[chnl], relative=False, verbose=True)
    #stage_controller.retract(chnl)

    #stage_controller.move_um(chnl, stage_controller.get_position_um(chnl), relative=False, verbose=True)
    #stage_controller.move_um(chnl, stage_controller.get_position_um(chnl)+0.55*stage_controller._stage_conversion_um[chnl], relative=False, verbose=True)

    #stage_controller.move_um(chnl, 8000, relative=False, verbose=True)

    # Move relative position step is step_um
    # print('\n# Relative moves:')
    # step_um = 1000 # um of each step
    # move = stage_controller.move_um(chnl, step_um)
    # move = stage_controller.move_um(chnl, -step_um)

    # Move relative position, n_steps up and down, each step is step_um
    # print('\n# Some relative moves:')
    # step_num = 3 # number of steps
    # step_um = 10 # um of each step
    # for moves in range(3):
    #     move = stage_controller.move_um(chnl, step_um)
    # for moves in range(3):
    #     move = stage_controller.move_um(chnl, -step_um)

    # Position motion test
    # pos_test_um = 3900
    # pos_test_rel = False
    # print('\n# Legalized move:')
    # legal_move_um = stage_controller.legalize_move_um(chnl, pos_test_um, relative = pos_test_rel)
    # print(legal_move_um)
    # stage_controller.move_um(chnl, pos_test_um, relative = pos_test_rel)
    # print('\n# Get position:')
    # stage_controller.get_position_um(chnl)

    # Set limits test
    # upper_lim_test_um = stage_controller._stage_upper_limit_um[chnl]/10
    # lower_lim_test_um = stage_controller._stage_lower_limit_um[chnl]/10
    #
    # stage_controller.move_um(chnl, upper_lim_test_um, relative=False)
    # stage_controller.set_stage_limit_um(chnl, lower_limit=False)
    #
    # stage_controller.move_um(chnl, lower_lim_test_um, relative=False)
    # stage_controller.set_stage_limit_um(chnl, lower_limit=True)
    #
    # stage_controller.legalize_move_um(chnl, 2 * upper_lim_test_um, relative=False) #Should throw error
    # stage_controller.legalize_move_um(chnl, 2 * lower_lim_test_um, relative=False)  # Should throw error
    #
    # stage_controller.legalize_move_um(chnl, upper_lim_test_um / 2, relative=False)  # Should work
    # stage_controller.legalize_move_um(chnl, lower_lim_test_um / 2, relative=False)  # Should work

    # Set retract and test
    # retract_point_test_um = stage_controller._stage_upper_limit_um[chnl]/12
    # stage_controller.move_um(chnl, retract_point_test_um, relative=False)
    # stage_controller.set_retract_point_um(chnl, retract_um=None)
    #
    # stage_controller.move_zero(chnl)
    #
    # stage_controller.retract(chnl)



    # print('\n# Non-blocking move:')
    # stage_controller.move_um(chnl, 200, block=False)
    # stage_controller.move_um(chnl, 100, block=False)
    # print('(immediate follow up call forces finish on pending move)')
    # print('doing something else')
    # stage_controller._finish_move(chnl)
    #
    # print('\n# Home:')
    # stage_controller.move_zero(chnl)

    # re-set zero:
    if False:
        stage_controller.move_um(channel=chnl, move_um=8000, relative=False, verbose=True)
        stage_controller._set_encoder_value_to_zero(channel=chnl)
        stage_controller.move_um(channel=chnl, move_um=0, relative=False, verbose=True)

    # Test motion time from the limits
    if False:
        from datetime import datetime
        print(f'Stage Lower Limit: {stage_controller._stage_lower_limit_um[chnl]} um')
        print(f'Stage Upper Limit: {stage_controller._stage_upper_limit_um[chnl]} um')
        while True:
            response = input("\033[93mIs this ok and in the path clear? (y/q)\033[0m")
            if response.lower().strip() == 'y':
                for i in range(10):
                    stage_controller.move_um(channel=chnl, move_um=-stage_controller._stage_lower_limit_um[chnl], relative=False,
                                            verbose=True)
                    time.sleep(2)
                    move_start_time = datetime.now()
                    stage_controller.move_um(channel=chnl, move_um=-stage_controller._stage_upper_limit_um[chnl], relative=False,
                                            verbose=True)
                    move_elapsed_time = datetime.now() - move_start_time
                    move_elapsed_seconds = move_elapsed_time.total_seconds()
                    print(f'Motion finished in {move_elapsed_time.total_seconds()} s')
                break
            elif response.lower().strip() == 'q':
                break
            else:
                print('Incorrect input, please reply: (y/q).')

    stage_controller.close()
