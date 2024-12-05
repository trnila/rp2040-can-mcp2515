import subprocess
from collections import namedtuple
import os
import pytest
import can

CANMsgID = namedtuple('CANMsgID', ['arbitration_id', 'is_extended_id'])

RP2040_IFACE = os.environ.get('RP2040_IFACE', 'can0')
TEST_IFACE = os.environ.get('TEST_IFACE', 'can1')

def create_can_channel(rp2040_sender):
    a = can.Bus(interface='socketcan', channel=RP2040_IFACE)
    b = can.Bus(interface='socketcan', channel=TEST_IFACE)

    if rp2040_sender:
        return (a, b)
    else:
        return (b, a)

def rp2040_sender_ids(rp2040_sender):
    return f'rp2040_{"sender" if rp2040_sender else "receiver"}'

@pytest.mark.parametrize("dlc", range(0, 9))
@pytest.mark.parametrize("rp2040_sender", [True, False], ids=rp2040_sender_ids)
def test_dlc(rp2040_sender, dlc):
    [tx, rx] = create_can_channel(rp2040_sender)

    tx.send(can.Message(arbitration_id=42, is_extended_id=0, data=range(dlc)))

    received = rx.recv(1)
    assert received
    assert list(received.data) == list(range(dlc))


@pytest.mark.parametrize("msgid", [
    CANMsgID(0, False),
    CANMsgID(1, True),
    CANMsgID(0x42, True),
    CANMsgID(0x42, False),
    CANMsgID(0x7FF, True),
    CANMsgID(0x7FF, False),
    CANMsgID(0x800, True),
    CANMsgID(0x1fffffff, True),
], ids=lambda msg: f"0x{msg.arbitration_id:x}-{'extended' if msg.is_extended_id else 'std'}")
@pytest.mark.parametrize("rp2040_sender", [True, False], ids=rp2040_sender_ids)
def test_msgids(rp2040_sender, msgid: CANMsgID):
    [tx, rx] = create_can_channel(rp2040_sender)

    tx.send(can.Message(arbitration_id=msgid.arbitration_id, is_extended_id=msgid.is_extended_id))
    received = rx.recv(1)
    assert received
    assert received.arbitration_id == msgid.arbitration_id
    assert received.is_extended_id == msgid.is_extended_id


@pytest.mark.parametrize("bitrate",
    [125000, 250000, 500000],
)
@pytest.mark.parametrize("rp2040_sender", [True, False], ids=rp2040_sender_ids)
def test_bitrates(rp2040_sender, bitrate):
    for dev in [RP2040_IFACE, TEST_IFACE]:
        subprocess.call(["sudo", "ip", "link", "set", dev, "down"])
        subprocess.check_call(["sudo", "ip", "link", "set", dev, "type", "can", "bitrate", str(bitrate)])
        subprocess.check_call(["sudo", "ip", "link", "set", dev, "up"])

    [tx, rx] = create_can_channel(rp2040_sender)

    tx.send(can.Message(arbitration_id=0x345, data=[1, 2, 3, 4, 5, 6, 7, 8]))
    received = rx.recv(1)
    assert received.arbitration_id == 0x345
    assert list(received.data) == [1, 2, 3, 4, 5, 6, 7, 8]
