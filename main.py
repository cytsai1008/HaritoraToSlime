import json
import math
import socket
import struct
import time
from collections import namedtuple

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

try:
    f = open("haritoslime.json")
except:
    with open("haritoslime.json", "w") as f:
        json.dump(ref_config, f, indent=4)
    print(
        "haritoslime.json not found. A new config file has been created, please edit it before attempting to run "
        "HaritoSlime again."
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
    print(
        "One or more options were missing from haritoslime.json. The file has been recreated."
    )
    print("Please check the file then try running HariSlime again.")
    quit()


# TODO: rewrite to a config to support multiple trackers
HaritoraPacket = namedtuple(
    "HaritoraPacket", "sensor_id, qw, qx, qy, qz, ax, ay, az"
)  # container that holds the data of a given tracker_id
for i in range(0, TRACKER_COUNT):
    globals()[f"sensor_{str(i)}_data"] = HaritoraPacket(
        0, 0, 0, 0, 0, 0, 0, 0
    )  # Create tracker data containers


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
    buffer += struct.pack(
        str(len(fw_string)) + "s", fw_string.encode("UTF-8")
    )  # fw string
    buffer += struct.pack("6s", "111111".encode("UTF-8"))  # MAC address
    buffer += struct.pack("B", 255)
    return buffer


def add_imu(tracker_id):
    global PACKET_COUNTER
    buffer = b"\x00\x00\x00\x0f"  # packet 15 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack(
        "B", tracker_id
    )  # tracker id (shown as IMU Tracker #x in SlimeVR)
    buffer += struct.pack("B", 0)  # sensor status
    buffer += struct.pack("B", 0)  # sensor type
    sock.sendto(buffer, (SLIME_IP, SLIME_PORT))
    print("Add IMU: " + str(tracker_id))
    PACKET_COUNTER += 1


def build_rotation_packet(qw: float, qx: float, qy: float, qz: float, tracker_id: int):
    # qw,qx,qy,qz: parts of a quaternion / tracker_id: Tracker ID
    buffer = b"\x00\x00\x00\x11"  # packet 17 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack(
        "B", tracker_id
    )  # tracker id (shown as IMU Tracker #x in SlimeVR)
    buffer += struct.pack("B", 1)  # data type (use is unknown)
    buffer += struct.pack(">ffff", qx, qz, qy, qw)  # quaternion as x,z,y,w
    buffer += struct.pack(
        "B", 0
    )  # calibration info (seems to not be used by SlimeVR currently)
    return buffer


def build_accel_packet(ax: float, ay: float, az: float, tracker_id: int):
    buffer = b"\x00\x00\x00\x04"  # packet 4 header
    buffer += struct.pack(">Q", PACKET_COUNTER)  # packet counter
    buffer += struct.pack(">fff", ax, ay, az)  # acceleration as x y z
    buffer += struct.pack(
        "B", tracker_id
    )  # tracker id (shown as IMU Tracker #x in SlimeVR)
    return buffer


"""
def sendAllIMUs(
    Tracker_ID,
):  # Tracker_ID: Table of Tracker ID. Just used to get the number of trackers
    global TPS, PACKET_COUNTER
    while True:
        for z in range(TPS):
            for i in range(len(Tracker_ID)):
                sensor = globals()[f"sensor_{str(i)}_data"]
                rot = build_rotation_packet(
                    sensor.qw, sensor.qx, sensor.qy, sensor.qz, i
                )
                sock.sendto(rot, (SLIME_IP, SLIME_PORT))
                PACKET_COUNTER += 1
                # Accel is still technically not ready yet (it doesn't rotate with the axes) but it's enough for
                # features like tap detection
                accel = build_accel_packet(sensor.ax, sensor.ay, sensor.az, i)
                sock.sendto(accel, (SLIME_IP, SLIME_PORT))
                PACKET_COUNTER += 1
            time.sleep(1 / TPS)
"""


def tracker_handler(address: str, x: float, y: float, z: float):
    if address.split("/")[3] == "head":
        Tracker_ID = 0
    else:
        Tracker_ID = int(address.split("/")[3])
    data_type = address.split("/")[4]
    if data_type.find("position"):
        # TODO: need to convert to acceleration before sending
        # print(f"position: {Tracker_ID} {x} {y} {z}")
        # if Tracker_ID == "head":
        pass
    else:
        quaternion = euler_to_quaternion(x, y, z)
        rot = build_rotation_packet(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3], Tracker_ID
        )


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return qw, qx, qy, qz


if __name__ == "__main__":
    PACKET_COUNTER = 0  # global packet counter. MUST be incremented every time a packet is sent or slime gets mad
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Setup network socket
    dispatcher = osc_server.Dispatcher()
    dispatcher.map("/tracking/trackers/*", tracker_handler)

    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 12345), dispatcher)
    print(f"Starting OSC on {server.server_address}")
    time.sleep(0.1)
    server.serve_forever()

    # TODO: read more from moslime.py, L336, not finished, yet.
    if AUTODISCOVER:
        SLIME_IP = "255.255.255.255"
        print(
            """Autodiscovery enabled. 
            If this gets stuck at "Searching...", 
            try disabling it and manually set the SlimeVR IP."""
        )
    else:
        SLIME_IP = CONFIG["slime_ip"]
        SLIME_PORT = CONFIG["slime_port"]
        print(
            "Using SlimeVR IP from config. "
            'If this gets stuck at "Searching...", '
            "make sure you have the right IP set in "
            "moslime.json"
        )

    # Connected To SlimeVR Server
    found = False
    sock.setsockopt(
        socket.SOL_SOCKET, socket.SO_BROADCAST, 1
    )  # allow broadcasting on this socket
    handshake = build_handshake()
    sock.bind(("0.0.0.0", 9696))  # listen on port 9696 to avoid conflicts with slime
    sock.settimeout(1)
    print("Searching for SlimeVR")

    while not found:
        try:
            print("Searching...")
            sock.sendto(
                handshake, (SLIME_IP, SLIME_PORT)
            )  # broadcast handshake on all interfaces
            data, src = sock.recvfrom(1024)

            if "Hey OVR =D" in str(
                data.decode("utf-8")
            ):  # SlimeVR responds with a packet containing "Hey OVR =D"
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
    for i in range(TRACKER_COUNT):
        for _ in range(3):
            # slimevr has been missing "add IMU" packets so we just send them 3 times to make sure they get through
            add_imu(i)

    time.sleep(0.5)
