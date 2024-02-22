import json
import socket
import struct
import threading
import time
from collections import namedtuple

import numpy as np
from pythonosc import osc_server


# Setup configuration
ref_config = {  # Reference config, used when haritoslime.json is missing
    "autodiscovery": True,
    "slime_ip": "127.0.0.1",
    "slime_port": 6969,
    "osc_port": 12345,
    "tps": 150,
    "tracker_count": 5,
}
NEXT_MSEC = 0  # used to keep track of the last time we sent a packet
PACKET_COUNTER = 0  # global packet counter. MUST be incremented every time a packet is sent or slime gets mad

try:
    f = open("haritoslime.json")
except:
    with open("haritoslime.json", "w") as f:
        json.dump(ref_config, f, indent=4)
    print(
        "haritoslime.json not found. A new config file has been created, "
        "please edit it before attempting to run HaritoSlime again."
    )
    quit()

try:
    CONFIG = json.load(f)
except:
    print(
        """There was an issue loading haritoslime.json. Please check that there aren't any stray commas or misspellings.
        If this issue persists, delete haritoslime.json and run HaritoSlime again to regenerate the config."""
    )
    quit()

try:
    AUTODISCOVER = CONFIG["autodiscovery"]  # SlimeVR autodiscovery
    SLIME_IP = CONFIG["slime_ip"]  # SlimeVR Server
    SLIME_PORT = CONFIG["slime_port"]  # SlimeVR Server
    TPS = CONFIG["tps"]  # SlimeVR packet frequency. Keep below 300
    OSC_PROT = CONFIG["osc_port"]
    TRACKER_COUNT = CONFIG["tracker_count"]  # Number of trackers
except:
    with open("haritoslime.json", "w") as f:
        json.dump(ref_config, f, indent=4)
    print("One or more options were missing from haritoslime.json. The file has been recreated.")
    print("Please check the file then try running HariSlime again.")
    quit()

del f

HaritoraPacket = namedtuple(
    "HaritoraPacket", "qw, qx, qy, qz, ax, ay, az"
)  # container that holds the data of a given tracker_count

for i in range(0, TRACKER_COUNT + 1):
    globals()[f"sensor_{str(i)}_data"] = HaritoraPacket(0, 0, 0, 0, 0, 0, 0)  # Create tracker data containers
del i


def build_handshake():
    fw_string = "HaritoSlime"
    buffer = b"\x00\x00\x00\x03"  # packet 3 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack(">I", 0)  # board ID
    buffer += struct.pack(">I", 0)  # IMU type
    buffer += struct.pack(">I", 0)  # MCU type
    buffer += struct.pack(">III", 0, 0, 0)  # IMU info
    buffer += struct.pack(">I", 0)  # Build
    buffer += struct.pack("B", len(fw_string))  # length of fw string
    buffer += struct.pack(str(len(fw_string)) + "s", fw_string.encode("UTF-8"))  # fw string
    buffer += struct.pack("6s", "111111".encode("UTF-8"))  # MAC address
    buffer += struct.pack("B", 255)
    return buffer


def add_imu(tracker_id):
    global PACKET_COUNTER
    buffer = b"\x00\x00\x00\x0f"  # packet 15 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack("B", tracker_id)  # tracker id (shown as IMU Tracker #x in SlimeVR)
    buffer += struct.pack("B", 0)  # sensor status
    buffer += struct.pack("B", 0)  # sensor type
    sock.sendto(buffer, (SLIME_IP, SLIME_PORT))
    print("Add IMU: " + str(tracker_id))
    PACKET_COUNTER += 1


def build_rotation_packet(qw: float, qx: float, qy: float, qz: float, tracker_id: int):
    # qw,qx,qy,qz: parts of a quaternion / tracker_count: Tracker ID
    buffer = b"\x00\x00\x00\x11"  # packet 17 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack("B", tracker_id)  # tracker id (shown as IMU Tracker #x in SlimeVR)
    buffer += struct.pack("B", 1)  # data type (use is unknown)
    buffer += struct.pack(">ffff", qx, qz, qy, qw)  # quaternion as x,z,y,w
    buffer += struct.pack("B", 0)  # calibration info (seems to not be used by SlimeVR currently)
    return buffer


def build_accel_packet(ax: float, ay: float, az: float, tracker_id: int):
    buffer = b"\x00\x00\x00\x04"  # packet 4 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack(">fff", ax, ay, az)  # acceleration as x y z
    buffer += struct.pack("B", tracker_id)  # tracker id (shown as IMU Tracker #x in SlimeVR)
    return buffer


def sendAllIMUs(tracker_count: int):
    global PACKET_COUNTER, NEXT_MSEC
    if NEXT_MSEC - time.time_ns() / 1000000 > 0:
        return
    for tracker_id in range(1, tracker_count + 1):
        sensor = globals()[f"sensor_{str(tracker_id)}_data"]
        rot = build_rotation_packet(sensor.qw, sensor.qx, sensor.qy, sensor.qz, tracker_id)
        accel = build_accel_packet(sensor.ax, sensor.ay, sensor.az, tracker_id)
        sock.sendto(rot, (SLIME_IP, SLIME_PORT))
        PACKET_COUNTER += 1
        sock.sendto(accel, (SLIME_IP, SLIME_PORT))
        PACKET_COUNTER += 1
    NEXT_MSEC = time.time_ns() / 1000000 + 1000 / TPS


def tracker_handler(address: str, x: float, y: float, z: float):
    if address.split("/")[3] == "head":
        Tracker_ID = 0
    else:
        Tracker_ID = int(address.split("/")[3])
    data_type = address.split("/")[4]
    if data_type.find("position"):
        # TODO: need to convert to acceleration before sending
        # print(f"position: {tracker_count} {x} {y} {z}")
        # if tracker_count == "head":
        # TODO: Acceleration is not ready yet
        globals()[f"sensor_{str(Tracker_ID)}_data"] = HaritoraPacket(0, 0, 0, 0, 0, 0, 0)
    else:
        quaternion = euler_to_quaternion(x, y, z)
        sensor = globals()[f"sensor_{str(Tracker_ID)}_data"]
        globals()[f"sensor_{str(Tracker_ID)}_data"] = HaritoraPacket(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
            sensor.ax,
            sensor.ay,
            sensor.az,
        )
    if Tracker_ID == TRACKER_COUNT:
        sendAllIMUs(TRACKER_COUNT)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qw, qx, qy, qz


if __name__ == "__main__":
    dispatcher = osc_server.Dispatcher()
    dispatcher.map("/tracking/trackers/*", tracker_handler)
    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 12345), dispatcher)
    print(f"Starting OSC on {server.server_address}")

    time.sleep(0.1)
    osc_server_thread = threading.Thread(target=server.serve_forever)
    osc_server_thread.start()

    # TODO: read more from moslime.py, L336, not finished, yet.
    if AUTODISCOVER:
        SLIME_IP = "255.255.255.255"
        print(
            "Autodiscovery enabled.\n"
            'If this gets stuck at "Searching...", \n'
            "try disabling it and manually set the SlimeVR IP."
        )
    else:
        SLIME_IP = CONFIG["slime_ip"]
        SLIME_PORT = CONFIG["slime_port"]
        print(
            "Using SlimeVR IP from config. "
            'If this gets stuck at "Searching...", '
            "make sure you have the right IP set in haritoslime.json."
        )

    # Connected To SlimeVR Server
    found = False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Setup network socket
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # allow broadcasting on this socket
    sock.bind(("0.0.0.0", 9696))  # listen on port 9696 to avoid conflicts with slime
    sock.settimeout(1)
    handshake = build_handshake()
    print("Searching for SlimeVR")

    while not found:
        try:
            print("Searching...")
            sock.sendto(handshake, (SLIME_IP, SLIME_PORT))  # broadcast handshake on all interfaces
            data, src = sock.recvfrom(1024)

            if "Hey OVR =D" in str(data.decode()):  # SlimeVR responds with a packet containing "Hey OVR =D"
                found = True
                SLIME_IP = src[0]
                SLIME_PORT = src[1]
        except:
            time.sleep(0)

    print("Found SlimeVR at " + str(SLIME_IP) + ":" + str(SLIME_PORT))
    PACKET_COUNTER += 1
    time.sleep(0.1)

    # Add additional IMUs. SlimeVR only supports one "real" tracker per IP, so the workaround is to make all the
    # trackers appear as extensions of the first tracker.
    for i in range(1, TRACKER_COUNT + 1):
        for _ in range(3):
            # slimevr has been missing "add IMU" packets, so we just send them 3 times to make sure they get through
            add_imu(i)

    time.sleep(0.1)
