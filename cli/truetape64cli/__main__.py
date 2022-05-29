import binascii
import sys
import getopt
import time
import serial
import os.path
import logging

log = logging.getLogger(__name__)
logging.basicConfig(format='%(message)s', level=logging.INFO)


def compute_checksum(buf):
    computed = 8
    for bi in range(len(buf) - 1):
        computed = (computed + buf[bi] & 0x3f) & 0x3f

    computed = (computed + (buf[len(buf) - 1] & 0b00000011)) & 0x3f

    return computed


def verify_checksum(buf):
    received = buf[len(buf) - 1] >> 2
    computed = compute_checksum(buf)
    if computed != received:
        log.error(f"Checksum error: computed {computed}, received {received}")
        for bi in range(len(buf)):
            log.error(f"0x{buf[bi]:02x}")

        return False

    return True


def encode_msg(set_mode, read_tape, write_tape, eod, data):
    header = set_mode > 0
    header = header | ((read_tape > 0) << 1)
    header = header | ((write_tape > 0) << 2)
    header = header | ((eod > 0) << 3)
    msg = bytearray(header.to_bytes(1, 'little', signed=False))

    msg.extend(data.to_bytes(4, 'little', signed=False))
    msg[4] = msg[4] | (compute_checksum(msg) << 2)
    return msg


def decode_msg(in_msg):
    msg = bytearray(in_msg)
    set_mode = (msg[0] & 0b00000001) > 0
    read_tape = (msg[0] & 0b00000010) > 0
    write_tape = (msg[0] & 0b00000100) > 0
    eod = (msg[0] & 0b00001000) > 0
    sense = (msg[0] & 0b00010000) > 0
    error = (msg[0] & 0b00100000) > 0
    msg[4] = msg[4] & 0b00000011
    data = int.from_bytes(msg[1:], byteorder='little', signed=False)

    print(" ".join(map("{0:08b}".format, msg)),
          f" | Set:{set_mode} Read:{read_tape} Write:{write_tape} EOD:{eod} Sense: {sense} - Error: {error} - Data: {data}")

    return set_mode, read_tape, write_tape, eod, sense, error, data


def test_serial(serial_device="/dev/ttyUSB3"):
    try:
        ser = serial.Serial(serial_device, baudrate=250000)
    except (FileNotFoundError, BrokenPipeError, serial.serialutil.SerialException):
        log.error(f"Cannot open {serial_device}. Be sure it exists and it's connected to the FTDI serial adapter")
        sys.exit(1)

    # Test message
    max_pulse_len = 67108863
    msg = encode_msg(set_mode=1, read_tape=1, write_tape=1, eod=0, data=max_pulse_len)
    ser.write(msg)

    msg = bytearray(ser.read(5))
    if not verify_checksum(msg):
        return

    _, _, _, _, _, _, data = decode_msg(msg)
    if data != max_pulse_len:
        print("data mismatch")
        return

    # Prepare to write
    msg = encode_msg(set_mode=1, read_tape=0, write_tape=1, eod=0, data=0)
    decode_msg(msg)
    ser.write(msg)

    msg = bytearray(ser.read(5))
    if not verify_checksum(msg):
        return

    # Write data
    _, _, _, _, _, _, data = decode_msg(msg)
    for i in range(data):
        msg = encode_msg(set_mode=0, read_tape=0, write_tape=0, eod=(i == data - 1), data=(i + 1) * 128)
        ser.write(msg)
        decode_msg(msg)


def dump(serial_device, output_file):
    try:
        ser = serial.Serial(serial_device, baudrate=250000, timeout=0)
    except (FileNotFoundError, BrokenPipeError, serial.serialutil.SerialException):
        log.error(f"Cannot open {serial_device}. Be sure it exists and it's connected to the FTDI serial adapter")
        sys.exit(1)

    try:
        tapfile = open(output_file, "w+b")
    except (FileNotFoundError, PermissionError):
        log.error(f"Cannot write to {output_file}.")
        sys.exit(1)

    tap_header = b'\x43\x36\x34\x2D\x54\x41\x50\x45\x2D\x52\x41\x57\x01\x00\x00\x00\x00\x00\x00\x00'
    tapfile.write(tap_header)

    logging.info("PRESS PLAY ON TAPE")
    try:
        while not ser.getCTS():
            time.sleep(0.01)
            continue
    except OSError:
        logging.error("Unrecoverable error while reading from serial device. Aborting")
        sys.exit(1)

    logging.info("Dumping started")

    data_len = 0
    pulses = 0
    shortest_pulse = None
    longest_pulse = None
    buffer = bytearray()
    checksum_errors = 0
    last_msg_ts = 0
    dumping = True

    msglen = 5

    while dumping:

        if time.time() - last_msg_ts > 5:
            last_msg_ts = time.time()
            if pulses > 0:
                logging.info(
                    f"Dumped {pulses} pulses, {checksum_errors} serial errors, min/max len {shortest_pulse}/{longest_pulse}")
            else:
                logging.info(f"No pulses detected yet, {checksum_errors} serial errors")

        try:
            b = ser.read()
        except OSError:
            logging.error("Unrecoverable error while reading from serial device. Aborting")
            sys.exit(1)

        if len(b) == 0:
            if not ser.getCTS():
                dumping = False
                continue
            time.sleep(0.1)
            continue

        buffer.append(b[0])
        if len(buffer) == msglen:
            if verify_checksum(buffer):  # FIXME
                # del buffer[0]  # FIXME this is the new header, not used for the moment
                counter_cycles = buffer[1] + buffer[2] * 0xFF + buffer[3] * 0xFFFF + (buffer[4] & 0b00000011) * 0xFFFFFF

                # AVR clock runs at 16MHz but the counter is clocked through a /8 prescaler
                # so the sampling frequency of the device is 2MHz
                # The the CIA runs at C64 clock speed, which for PAL version is 985248Hz
                #
                cia_cycles = counter_cycles / 2000000 * 985248

                # TAP version 1 can encode pulse length using either one or three bytes
                # If the pulse length is less than 255 when divided by 8, then one byte is used
                # Otherwise 3 bytes are used but in this case the length is not divided by 8
                tap_pulse_len = int(round(cia_cycles / 8))
                if tap_pulse_len > 255:  # Will need 3 bytes, no /8 division, better accuracy
                    tap_pulse_len = int(round(cia_cycles))
                    if tap_pulse_len > 2 ** 24 - 1:  # 24 bits, 3 bytes, maximum allowed by TAP file
                        tap_pulse_len = 2 ** 24 - 1

                buffer.clear()
                pulses += 1

                if shortest_pulse:
                    shortest_pulse = min(shortest_pulse, tap_pulse_len)
                else:
                    shortest_pulse = tap_pulse_len

                if longest_pulse:
                    longest_pulse = max(longest_pulse, tap_pulse_len)
                else:
                    longest_pulse = tap_pulse_len

                try:
                    if tap_pulse_len > 255:
                        data_len += tapfile.write(b'\00')
                        data_len += tapfile.write(tap_pulse_len.to_bytes(3, byteorder="little", signed=False))
                    else:
                        data_len += tapfile.write(tap_pulse_len.to_bytes(1, byteorder="little", signed=False))
                except OSError:
                    logging.error("Unrecoverable error while writing to file. Aborting")
                    sys.exit(1)

            else:
                del buffer[0]
                checksum_errors += 1

    tapfile.seek(16)
    tapfile.write(data_len.to_bytes(4, byteorder="little", signed=False))
    tapfile.close()

    logging.info(
        f"Dumped {pulses} pulses, {checksum_errors} serial errors, min/max len {shortest_pulse}/{longest_pulse}")
    logging.info("Check led status on device")


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
    test_serial()


def main():
    try:
        run(sys.argv[1:])
    except KeyboardInterrupt:
        print("Aborted.")
        sys.exit(0)


if __name__ == "__main__":
    main()
