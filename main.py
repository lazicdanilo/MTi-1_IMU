""" Module for communicating with  XSenseIMU MTi-1 """

import serial as s
from time import sleep
from colorama import Fore, init
import struct


class frame:
    # Message looks like this: Preamble (0xFA) | BUS ID (0xFF) | Message ID | Length (Max 254) | Data | Checksum (1 byte)
    preamble = 0xFA  # First byte in the message (Fixed)
    bus_id = 0xFF  # Second byte in the message (Fixed)
    message_id = 0  # Third byte in the message
    length = 0  # Fourth byte in the message (Depends on message 'data' length)
    data = []  # Data bytes in the message
    checksum = 0  # Last byte in the message


class imu:
    MAX_FRAME_SIZE = 254
    PREAMBLE = 0xFA
    BID = 0xFF

    # Configured to output acceleration data 0x4020 (402y in datasheet)
    # See https://mtidocs.movella.com/messages section SetOutputConfiguration for more information
    DESIRED_OUTPUT_CONFIGURATION = [0x40, 0x20, 0x00, 0x64]

    PRINT_RAW_RX_DATA = False
    PRINT_RAW_TX_DATA = False

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200) -> None:
        self.port = s.Serial(port, baudrate, timeout=1)

        # Init colorama
        init()

    def _calculate_frame_checksum(self, frame: frame) -> int:
        """Calculate the checksum of the frame
        The checksum is the 2's complement of the sum of all bytes in the message

        Args:
            frame (frame): The frame to calculate the checksum for

        Returns:
            int: The checksum
        """
        data = [frame.bus_id, frame.message_id, frame.length]
        data += frame.data
        return (256 - sum(data) % 256) % 256

    def _print_bytes_hex(self, data: list, txt: str) -> None:
        """Print the bytes in the list as hex. Used for debugging

        Args:
            data (list): The list of bytes to print
            txt (str): The text to print before the bytes
        """
        print()
        print(Fore.BLUE + txt + Fore.RESET)
        for i in range(len(data)):
            if i % 32 == 0:
                print()
            print(Fore.BLUE + f"{data[i]:02x}", end=" " + Fore.RESET)
        print()

    def _tx_rx(self, tx_frame: frame, expected_msg_id: int, tries: int = 3) -> frame:
        """Transmit a frame and wait for a response

        Args:
            tx_frame (frame): The frame to transmit
            expected_msg_id (int): The expected message ID of the response
            tries (int, optional): The number of tries before giving up. Defaults to 3.

        Returns:
            frame: The received frame
        """
        for _ in range(tries):
            tx_data = [
                tx_frame.preamble,
                tx_frame.bus_id,
                tx_frame.message_id,
                tx_frame.length,
            ]
            tx_data += tx_frame.data
            tx_data += [self._calculate_frame_checksum(tx_frame)]

            if self.PRINT_RAW_TX_DATA:
                self._print_bytes_hex(tx_data, "Transmitting data")

            self.port.write(tx_data)

            fr = self._read_frame()

            if fr == None:
                continue

            if fr.message_id == expected_msg_id:
                return fr.data
            else:
                print(
                    Fore.YELLOW
                    + f"Expected message ID: {hex(expected_msg_id)}, Received message ID: {hex(fr.message_id)}"
                    + Fore.RESET
                )

        return None

    def _put_sensor_in_conf_mode(self) -> None:
        """Put the sensor in configuration mode. Sensor needs to be in configuration mode to change settings"""
        fr = frame()
        fr.message_id = 0x30
        fr.length = 0
        if self._tx_rx(fr, 0x31, tries=10) == None:
            print(Fore.RED + "Failed to put sensor in config mode" + Fore.RESET)
            exit(1)

    def _reset_sensor(self) -> None:
        """Reset the sensor. Good to do before configuring the sensor"""
        fr = frame()
        fr.message_id = 0x40
        fr.length = 0
        if self._tx_rx(fr, 0x41, tries=10) == None:
            print(Fore.RED + "Failed to reset sensor" + Fore.RESET)
            exit(1)

    def _get_sensor_id(self) -> list:
        """Get the sensor ID. Sanity check to see if the sensor is connected

        Returns:
            list: The sensor ID
        """
        fr = frame()
        fr.message_id = 0x00
        fr.length = 0
        data = self._tx_rx(fr, 0x01)
        if data == None:
            print(Fore.RED + "Failed to get sensor ID" + Fore.RESET)
            exit(1)
        return data

    def _get_output_configuration(self) -> list:
        """Get the current output configuration

        Returns:
            list: The output configuration
        """
        fr = frame()
        fr.message_id = 0xC0
        fr.length = 0
        ret = self._tx_rx(fr, 0xC1, tries=10)
        if ret == None:
            print(Fore.RED + "Failed to get output configuration" + Fore.RESET)
            exit(1)
        return ret

    def _set_output_configuration(
        self, desired_config: list = DESIRED_OUTPUT_CONFIGURATION
    ) -> None:
        """Set the output configuration to the desired configuration

        Args:
            desired_config (list, optional): The desired output configuration.
            Defaults to DESIRED_OUTPUT_CONFIGURATION.
        """

        fr = frame()
        fr.message_id = 0xC0
        fr.length = 4
        fr.data = desired_config
        if self._tx_rx(fr, 0xC1, tries=10) == None:
            print(Fore.RED + "Failed to set output configuration" + Fore.RESET)
            exit(1)

    def _set_location_id(self, id: int) -> None:
        """Set the location ID of the sensor. Used for enumerating multiple sensors.

        Args:
            id (int): The location ID
        """
        fr = frame()
        fr.message_id = 0x84
        fr.length = 2
        fr.data = [id & 0xFF, (id >> 8) & 0xFF]
        if self._tx_rx(fr, 0x85, tries=10) == None:
            print(Fore.RED + "Failed to set location ID" + Fore.RESET)
            exit(1)

    def _get_location_id(self) -> int:
        """Get the location ID of the sensor. Used for enumerating multiple sensors.

        Returns:
            int: The location ID
        """
        fr = frame()
        fr.message_id = 0x84
        fr.length = 0
        ret = self._tx_rx(fr, 0x85, tries=10)
        if ret == None:
            print(Fore.RED + "Failed to get location ID" + Fore.RESET)
            exit(1)
        return ret

    def _put_sensor_in_measurement_mode(self) -> None:
        """Put the sensor in measurement mode. Sensor needs to be in measurement mode to output data"""
        fr = frame()
        fr.message_id = 0x10
        fr.length = 0
        if self._tx_rx(fr, 0x11, tries=10) == None:
            print(Fore.RED + "Failed to put sensor in measurement mode" + Fore.RESET)
            exit(1)

    def _read_frame(self) -> frame:
        """Read a frame from the sensor

        Returns:
            frame: The received frame
        """
        # Wait for preamble
        while True:
            self.data = self.port.read(1)
            if self.data == b"\xfa":
                break

        self.data += self.port.read(1)  # BUS ID
        self.data += self.port.read(1)  # Message ID
        self.data += self.port.read(1)  # Length
        self.data += self.port.read(self.data[3] + 1)  # +1 for checksum

        if self.PRINT_RAW_RX_DATA:
            self._print_bytes_hex(self.data, "Received data")

        fr = frame()
        fr.preamble = self.data[0]
        fr.bus_id = self.data[1]
        if fr.bus_id != 0xFF:
            print(Fore.RED + "No bus id found" + Fore.RESET)
            return None

        fr.message_id = self.data[2]
        fr.length = self.data[3]
        fr.data = self.data[4 : 4 + fr.length]
        fr.checksum = self.data[4 + fr.length]

        checksum = self._calculate_frame_checksum(fr)
        if checksum != fr.checksum:
            print(Fore.RED + "Checksum failed" + Fore.RESET)
            return None

        return fr

    def _get_acceleration_data(self, data: list) -> list:
        """Get the acceleration data from the frame

        Args:
            data (list): The data bytes from the frame

        Returns:
            list: The acceleration data in x, y, z (all floats)
        """
        # Remove first 3 bytes (I think they are output configuration)
        data = data[3:]
        byte_data = bytes(data)

        # Unpack the bytes as a big-endian float32
        x = struct.unpack(">f", byte_data[0:4])[0]
        y = struct.unpack(">f", byte_data[4:8])[0]
        z = struct.unpack(">f", byte_data[8:12])[0]

        return [x, y, z]

    def run(self) -> None:
        """Run the program"""

        self._reset_sensor()
        self._put_sensor_in_conf_mode()
        dev_id = self._get_sensor_id()
        print(
            Fore.GREEN
            + f"Device ID: {dev_id[0]:02x} {dev_id[1]:02x} {dev_id[2]:02x} {dev_id[3]:02x}"
            + Fore.RESET
        )
        sleep(1)

        conf = self._get_output_configuration()
        print(
            Fore.BLUE
            + f"Current output configuration: {conf[0]:02x} {conf[1]:02x} {conf[2]:02x} {conf[3]:02x}"
            + Fore.RESET
        )

        if conf != self.DESIRED_OUTPUT_CONFIGURATION:
            self._set_output_configuration(self.DESIRED_OUTPUT_CONFIGURATION)

        sleep(0.1)
        self._put_sensor_in_measurement_mode()

        while True:
            fr = self._read_frame()
            accel = self._get_acceleration_data(fr.data)
            print(
                Fore.GREEN
                + f"Acceleration: {accel[0]:.2f} {accel[1]:.2f} {accel[2]:.2f}"
                + Fore.RESET
            )


def main():
    i = imu("/dev/ttyUSB0", 115200)

    try:
        i.run()
    except KeyboardInterrupt:
        print(Fore.RED + "Exiting" + Fore.RESET)
        i.port.close()


if __name__ == "__main__":
    main()
