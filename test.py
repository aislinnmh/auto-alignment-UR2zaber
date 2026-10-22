import socket
import struct

def read_exact(sock, count):
    data = b""
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("Robot closed connection")
        data += chunk
    return data

def receive(sock):
    size, kind = struct.unpack(">HB", read_exact(sock, 3))
    return kind, read_exact(sock, size - 3)

def send(sock, kind, payload=b""):
    sock.sendall(struct.pack(">HB", len(payload) + 3, kind) + payload)

try:
    with socket.create_connection(("192.168.1.39", 30004), timeout=5) as sock:
        sock.settimeout(5)

        send(sock, 86, struct.pack(">H", 2))
        print("Protocol:", receive(sock), flush=True)

        # Request actual_q at 125 Hz.
        send(sock, 79, struct.pack(">d", 125.0) + b"actual_q")
        print("Output setup:", receive(sock), flush=True)

        send(sock, 83)
        for _ in range(10):
            kind, body = receive(sock)
            if kind == 85:
                print("Joint angles:", struct.unpack(">6d", body[1:]))
                break
            print("Response:", kind, body, flush=True)
        else:
            print("No joint-data packet received")

except Exception as e:
    print(f"{type(e).__name__}: {e}", flush=True)