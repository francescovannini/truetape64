import sys
import getopt
import time
import serial
import os.path
import logging

log = logging.getLogger(__name__)
logging.basicConfig(format='%(message)s', level=logging.INFO)


def verify_checksum(buf):
    checksum = 64
    for bi in range(len(buf) - 1):
        checksum = checksum + buf[bi]

    checksum = checksum % 256

    if buf[3] != checksum:
        return False

    return True


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
    buffer = bytearray()
    checksum_errors = 0
    last_msg_ts = 0
    dumping = True

    while dumping:

        if time.time() - last_msg_ts > 5:
            last_msg_ts = time.time()
            if pulses > 0:
                logging.info(f"{pulses} pulses, {checksum_errors} serial errors, shortest pulse was {shortest_pulse}")
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
        if len(buffer) == 4:
            if verify_checksum(buffer):
                counter_cycles = buffer[0] + buffer[1] * 0xFF + buffer[2] * 0xFFFF  # 2MHz clock of TC1 via /8 prescaler
                counter_cycles_64 = counter_cycles / 2000000 * 985248  # .98MHz C64 clock conversion
                buffer.clear()
                pulses += 1

                tap_pulse = int(round(counter_cycles_64 / 8 * .985248))

                if shortest_pulse:
                    shortest_pulse = min(shortest_pulse, tap_pulse)
                else:
                    shortest_pulse = tap_pulse

                try:
                    if tap_pulse <= 255:
                        data_len += tapfile.write(tap_pulse.to_bytes(1, byteorder="little", signed=False))
                    else:
                        data_len += tapfile.write(b'\00')
                        data_len += tapfile.write(tap_pulse.to_bytes(3, byteorder="little", signed=False))
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
        f"Dumping halted. Dumped {pulses} pulses, {checksum_errors} serial errors, shortest pulse was {shortest_pulse}")
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

    dump(serial_device, output_file)


def main():
    try:
        run(sys.argv[1:])
    except KeyboardInterrupt:
        print("Aborted.")
        sys.exit(0)


if __name__ == "__main__":
    main()
