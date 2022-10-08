import binascii
import random
import sys
import getopt
import time
import serial
import os.path
import logging

log = logging.getLogger(__name__)
logging.basicConfig(format='%(message)s', level=logging.DEBUG)
MSG_DATA_LEN = 10


def santize_bit(in_value):
    if isinstance(in_value, int) or isinstance(in_value, bool):
        return int(in_value)
    else:
        return 0


class ErrorFlags:
    FL_ERR_GENERIC = 0
    FL_ERR_SEND_QUEUE_EMPTY = 0
    FL_ERR_SEND_QUEUE_FULL = 0
    FL_ERR_RECV_QUEUE_EMPY = 0
    FL_ERR_RECV_QUEUE_FULL = 0
    FL_ERR_FRAMING = 0

    def __dir__(self):
        return ['FL_ERR_GENERIC', 'FL_ERR_SEND_QUEUE_EMPTY', 'FL_ERR_SEND_QUEUE_FULL', 'FL_ERR_RECV_QUEUE_EMPY',
                'FL_ERR_RECV_QUEUE_FULL', 'FL_ERR_FRAMING']

    def decode_error_flags(self, in_value):
        for c, attr in enumerate(self.__dir__()):
            v = (in_value >> c) & 1
            setattr(self, attr, v)

    def printout(self):
        for c, v in enumerate(self.__dir__()):
            print(v, getattr(self, v))

    def __init__(self, v=None):
        if isinstance(v, int):
            self.decode_error_flags(v)


class Msg:

    def __init__(self, set_mode=None, read_tape=None, write_tape=None, eod=None, cassette_sense=None,
                 error_detected=None, ack=None, ext_pulse_len=None, pulses=None, available_msg=None):
        self.set_mode = santize_bit(set_mode)
        self.read_tape = santize_bit(read_tape)
        self.write_tape = santize_bit(write_tape)
        self.eod = santize_bit(eod)
        self.cassette_sense = santize_bit(cassette_sense)
        self.ack = santize_bit(ack)
        self.error_detected = santize_bit(error_detected)
        self.available_msg = santize_bit(available_msg)
        self.error_flags = None

        if ext_pulse_len is None:
            if pulses is None or not isinstance(pulses, list):
                self.ext_pulse_len = 0
                self.pulses = [0] * MSG_DATA_LEN
            else:
                if min(pulses) < 0:
                    raise Exception("Pulse length can't be negative")

                if max(pulses) > 2 ** 16 - 1:
                    raise Exception("Pulse length overflow")

                if max(pulses) <= 255:
                    self.ext_pulse_len = 0
                    if len(pulses) != MSG_DATA_LEN:
                        raise Exception("Wrong number of pulses per message")
                else:
                    self.ext_pulse_len = 1
                    if len(pulses) != MSG_DATA_LEN / 2:
                        raise Exception("Wrong number of pulses per message")

                self.pulses = pulses

        else:
            self.ext_pulse_len = santize_bit(ext_pulse_len)

            if pulses is None or not isinstance(pulses, list):
                if self.ext_pulse_len:
                    self.pulses = [0] * int(MSG_DATA_LEN / 2)
                else:
                    self.pulses = [0] * MSG_DATA_LEN
            else:
                if self.ext_pulse_len:
                    if len(pulses) != MSG_DATA_LEN / 2:
                        raise Exception("Wrong number of pulses per message")
                    self.pulses = pulses
                else:
                    if len(pulses) != MSG_DATA_LEN:
                        raise Exception("Wrong number of pulses per message")
                    self.pulses = pulses

    def to_bytearray(self):
        header = self.set_mode > 0
        header = header | ((self.read_tape > 0) << 1)
        header = header | ((self.write_tape > 0) << 2)
        header = header | ((self.eod > 0) << 3)
        header = header | ((self.cassette_sense > 0) << 4)
        header = header | ((self.error_detected > 0) << 5)
        header = header | ((self.ack > 0) << 6)
        header = header | ((self.ext_pulse_len > 0) << 7)

        ba = bytearray(header.to_bytes(1, 'little', signed=False))
        for c, v in enumerate(self.pulses):
            ba.extend(v.to_bytes(1 + self.ext_pulse_len, 'little', signed=False))

        return ba

    def to_string(self, direction="?"):
        bits = " ".join(map("{0:08b}".format, self.to_bytearray())).ljust((MSG_DATA_LEN + 1) * 9, " ")
        flags = ""
        if self.set_mode:
            flags = flags + "SET "
        else:
            flags = flags + "___ "

        if self.read_tape:
            flags = flags + "READ "
        else:
            flags = flags + "____ "

        if self.write_tape:
            flags = flags + "WRITE "
        else:
            flags = flags + "_____ "

        if self.eod:
            flags = flags + "EOD "
        else:
            flags = flags + "___ "

        if self.cassette_sense:
            flags = flags + "SENSE "
        else:
            flags = flags + "_____ "

        if self.error_detected:
            flags = flags + "ERROR "
        else:
            flags = flags + "_____ "

        if self.ack:
            flags = flags + "ACK "
        else:
            flags = flags + "___ "

        if self.ext_pulse_len:
            flags = flags + "EXT "
        else:
            flags = flags + "___ "

        if self.ack:
            return f"{direction} {bits} | {flags} | Slots: {self.available_msg}"
        else:
            return f"{direction} {bits} | {flags} | Pulse: {self.pulses}"

    def from_serial(self, ser):
        if not isinstance(ser, serial.Serial):
            return

        # Read header
        msg = bytearray(ser.read(1))
        self.set_mode = msg[0] & 0b00000001
        self.read_tape = (msg[0] & 0b00000010) >> 1
        self.write_tape = (msg[0] & 0b00000100) >> 2
        self.eod = (msg[0] & 0b00001000) >> 3
        self.cassette_sense = (msg[0] & 0b00010000) >> 4
        self.error_detected = (msg[0] & 0b00100000) >> 5
        self.ack = (msg[0] & 0b01000000) >> 6
        self.ext_pulse_len = (msg[0] & 0b10000000) >> 7

        if self.ack:
            msg.extend(bytearray(ser.read(2)))
            self.available_msg = int.from_bytes(msg[1:2], byteorder='little', signed=False)
            if self.error_detected:
                self.error_flags = ErrorFlags(int.from_bytes(msg[2:3], byteorder='little', signed=False))
            self.pulses = []

        else:
            msg.extend(bytearray(ser.read(MSG_DATA_LEN)))
            bp = 1
            i = 0
            while bp < len(msg):
                self.pulses[i] = int.from_bytes(msg[bp:bp + 1 + self.ext_pulse_len], byteorder='little',
                                                signed=False)
                bp += (1 + self.ext_pulse_len)
                i += 1

        log.debug(self.to_string("<"))

    def to_serial(self, ser):
        if not isinstance(ser, serial.Serial):
            return

        ser.write(self.to_bytearray())
        log.debug(self.to_string(">"))


def test_serial(serial_device="/dev/ttyUSB3"):
    try:
        ser = serial.Serial(serial_device, baudrate=250000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
    except (FileNotFoundError, BrokenPipeError, serial.serialutil.SerialException):
        log.error(f"Cannot open {serial_device}. Be sure it exists and it's connected to the FTDI serial adapter")
        sys.exit(1)

    log.info("Serial opened, sleeping a bit...")
    time.sleep(1)

    while ser.inWaiting() > 0:
        ser.read(1)

    while True:
        # Test message
        randbytes = list(random.getrandbits(8) for _ in range(MSG_DATA_LEN))

        Msg(set_mode=1, read_tape=1, write_tape=1, eod=0, ext_pulse_len=0, pulses=randbytes).to_serial(ser)

        msg = Msg()
        msg.from_serial(ser)

        t = 0
        for i, v in enumerate(msg.pulses):
            if msg.pulses[i] != randbytes[i]:
                print("Data mismatch, trying again")
                t += 1
                break

        if t == 0:
            break

    # Begin write
    Msg(set_mode=1, read_tape=0, write_tape=1, eod=0, pulses=0).to_serial(ser)

    counter = 0
    pulse_len = 300  # <85 tape seems unreliable. reliable threshold is probably higher. need to support less than 200
    minpulselen = pulse_len

    while True:
        requested = 0
        while ser.inWaiting():

            qsizemsg = Msg()
            qsizemsg.from_serial(ser)

            # Read the error flags
            if qsizemsg.error_flags:
                print("Shortest pulse ", minpulselen)
                qsizemsg.error_flags.printout()
                return

            requested += qsizemsg.available_msg

        # Send requested data
        buf = bytearray()
        for i in range(requested):
            pl = pulse_len - counter
            msg = Msg(set_mode=0, read_tape=0, write_tape=1, eod=0, ext_pulse_len=1,
                      pulses=[pl] * int(MSG_DATA_LEN / 2))
            minpulselen = min(minpulselen, pl)
            msg.to_serial(ser)
            # counter += 1


# def dump(serial_device, output_file):
# try:
#     ser = serial.Serial(serial_device, baudrate=250000, timeout=0)
# except (FileNotFoundError, BrokenPipeError, serial.serialutil.SerialException):
#     log.error(f"Cannot open {serial_device}. Be sure it exists and it's connected to the FTDI serial adapter")
#     sys.exit(1)
#
# try:
#     tapfile = open(output_file, "w+b")
# except (FileNotFoundError, PermissionError):
#     log.error(f"Cannot write to {output_file}.")
#     sys.exit(1)
#
# tap_header = b'\x43\x36\x34\x2D\x54\x41\x50\x45\x2D\x52\x41\x57\x01\x00\x00\x00\x00\x00\x00\x00'
# tapfile.write(tap_header)
#
# logging.info("PRESS PLAY ON TAPE")
# try:
#     while not ser.getCTS():
#         time.sleep(0.01)
#         continue
# except OSError:
#     logging.error("Unrecoverable error while reading from serial device. Aborting")
#     sys.exit(1)
#
# logging.info("Dumping started")
#
# data_len = 0
# pulses = 0
# shortest_pulse = None
# longest_pulse = None
# buffer = bytearray()
# checksum_errors = 0
# last_msg_ts = 0
# dumping = True
#
# msglen = 5
#
# while dumping:
#
#     if time.time() - last_msg_ts > 5:
#         last_msg_ts = time.time()
#         if pulses > 0:
#             logging.info(
#                 f"Dumped {pulses} pulses, {checksum_errors} serial errors, min/max len {shortest_pulse}/{longest_pulse}")
#         else:
#             logging.info(f"No pulses detected yet, {checksum_errors} serial errors")
#
#     try:
#         b = ser.read()
#     except OSError:
#         logging.error("Unrecoverable error while reading from serial device. Aborting")
#         sys.exit(1)
#
#     if len(b) == 0:
#         if not ser.getCTS():
#             dumping = False
#             continue
#         time.sleep(0.1)
#         continue
#
#     buffer.append(b[0])
#     if len(buffer) == msglen:
#         if verify_checksum(buffer):  # FIXME
#             # del buffer[0]  # FIXME this is the new header, not used for the moment
#             counter_cycles = buffer[1] + buffer[2] * 0xFF + buffer[3] * 0xFFFF + (buffer[4] & 0b00000011) * 0xFFFFFF
#
#             # AVR clock runs at 16MHz but the counter is clocked through a /8 prescaler
#             # so the sampling frequency of the device is 2MHz
#             # The the CIA runs at C64 clock speed, which for PAL version is 985248Hz
#             #
#             cia_cycles = counter_cycles / 2000000 * 985248
#
#             # TAP version 1 can encode pulse length using either one or three bytes
#             # If the pulse length is less than 255 when divided by 8, then one byte is used
#             # Otherwise 3 bytes are used but in this case the length is not divided by 8
#             tap_pulse_len = int(round(cia_cycles / 8))
#             if tap_pulse_len > 255:  # Will need 3 bytes, no /8 division, better accuracy
#                 tap_pulse_len = int(round(cia_cycles))
#                 if tap_pulse_len > 2 ** 24 - 1:  # 24 bits, 3 bytes, maximum allowed by TAP file
#                     tap_pulse_len = 2 ** 24 - 1
#
#             buffer.clear()
#             pulses += 1
#
#             if shortest_pulse:
#                 shortest_pulse = min(shortest_pulse, tap_pulse_len)
#             else:
#                 shortest_pulse = tap_pulse_len
#
#             if longest_pulse:
#                 longest_pulse = max(longest_pulse, tap_pulse_len)
#             else:
#                 longest_pulse = tap_pulse_len
#
#             try:
#                 if tap_pulse_len > 255:
#                     data_len += tapfile.write(b'\00')
#                     data_len += tapfile.write(tap_pulse_len.to_bytes(3, byteorder="little", signed=False))
#                 else:
#                     data_len += tapfile.write(tap_pulse_len.to_bytes(1, byteorder="little", signed=False))
#             except OSError:
#                 logging.error("Unrecoverable error while writing to file. Aborting")
#                 sys.exit(1)
#
#         else:
#             del buffer[0]
#             checksum_errors += 1
#
# tapfile.seek(16)
# tapfile.write(data_len.to_bytes(4, byteorder="little", signed=False))
# tapfile.close()
#
# logging.info(
#     f"Dumped {pulses} pulses, {checksum_errors} serial errors, min/max len {shortest_pulse}/{longest_pulse}")
# logging.info("Check led status on device")


def help(exit_error, message=None):
    if message is not None:
        log.warning(message)

    log.warning("truetape64cli [-d /dev/ttyUSB4] [-y] streetfighter_a.tap")
    log.warning(" -d: specify serial device (default is /dev/ttyUSB0)")
    log.warning(" -y: overwrite output file if exists")

    if exit_error:
        sys.exit(1)
    else:
        sys.exit(0)


def run(argv):
    serial_device = "/dev/ttyUSB0"
    output_file = None
    opts = None
    args = None
    overwrite_outfile = False

    try:
        opts, args = getopt.getopt(argv, "hd:y", ["device="])
    except getopt.GetoptError:
        help(True)

    for opt, arg in opts:
        if opt == '-h':
            help(0)
        elif opt in ("-d", "--device"):
            serial_device = arg
        elif opt in ("-o", "--outfile"):
            output_file = arg
        elif opt in "-y":
            overwrite_outfile = True

    if args and args[0] is not None:
        output_file = args[0]
        if os.path.isfile(output_file) and not overwrite_outfile:
            help(1, "Specified output file already exists")
    else:
        help(True)

    # dump(serial_device, output_file)
    test_serial(serial_device=serial_device)


def main():
    try:
        run(sys.argv[1:])
    except KeyboardInterrupt:
        print("Aborted.")
        sys.exit(0)


if __name__ == "__main__":
    main()
