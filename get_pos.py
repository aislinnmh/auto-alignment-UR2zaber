import socket
import struct

def get_pos(tracking_mode="pupil"):
    # tracking mode is pupil or pitag

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('localhost', 2345))
    data = s.recv(4096)

    if tracking_mode == "pupil":
        offset = 0

        (frame,) = struct.unpack_from('<Q', data, offset)
        offset += struct.calcsize('<Q')

        (timestamp,) = struct.unpack_from('<Q', data, offset)
        offset += struct.calcsize('<Q')

        (flag,) = struct.unpack_from('<?', data, offset)
        offset += struct.calcsize('<?')

        if flag:
            eye_position_3d = struct.unpack_from('<ddd', data, offset)
            offset += struct.calcsize('<ddd')
        else:
            eye_position_3d = None

    elif tracking_mode == "pitag":
        offset = 0

        (enable,) = struct.unpack_from('<?', data, offset)
        offset += struct.calcsize('<?')

        (frame,) = struct.unpack_from('<Q', data, offset)
        offset += struct.calcsize('<Q')

        (timestamp,) = struct.unpack_from('<Q', data, offset)
        offset += struct.calcsize('<Q')

        (sample_rate,) = struct.unpack_from('<d', data, offset)
        offset += struct.calcsize('<d')

        (flag,) = struct.unpack_from('<?', data, offset)
        offset += struct.calcsize('<?')

        if flag:
            eye_position_3d = struct.unpack_from('<ddd', data, offset)
            offset += struct.calcsize('<ddd')
        else:
            eye_position_3d = None

    return eye_position_3d