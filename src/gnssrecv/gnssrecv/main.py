"""
Read the position of the u-blox GNSS receiver.
"""
import json

import pyubx2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
import serial.tools.list_ports
from std_msgs.msg import String


class GNSSReceiver:
    """u-blox GNSS receiver."""

    def __init__(self) -> None:
        self.portname = self._find_portname()
        self.port = serial.Serial(self.portname, baudrate=115200, timeout=1)
        self.reader = pyubx2.UBXReader(datastream=self.port, protfilter=2)
        self.ubxmsg = pyubx2.UBXMessage("NAV", "NAV-POSLLH", pyubx2.POLL)

    def _find_portname(self):
        """Find the name of the serial port of the receiver."""
        portname = None
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.description == "u-blox GNSS receiver":
                portname = f"/dev/{port.name}"
                break
        if portname is None:
            raise RuntimeError("No serial port for GNSS receiver")
        return portname
    
    def _parse_ubxmessage(self, ubxmessage):
        """Convert the UBX message into dictionary."""
        msgstr = str(ubxmessage)
        index1 = msgstr.find("iTOW")
        index2 = msgstr.find(")")
        message = msgstr[index1:index2]
        items = [items.lstrip().split("=") for items in message.split(",")]
        data = {item[0]: item[1] for item in items}
        for key, value in list(data.items())[1:]:
            data[key] = float(value) if value.find(".") != 1 else int(value)
        return data

    def read(self):
        """Read UBX-messages from the receiver."""
        self.port.write(self.ubxmsg.serialize())
        (rawdata, ubxmessage) = self.reader.read()
        message = self._parse_ubxmessage(ubxmessage)
        return json.dumps(message)


class PositionPublisher(Node):
    """Publish the position of the GNSS receiver."""

    def __init__(self):
        super().__init__("pospub")
        self.pub = self.create_publisher(String, "gnsspos", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gnss = GNSSReceiver()

    def timer_callback(self):
        msg = String()
        msg.data = self.gnss.read()
        self.pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    pospub = PositionPublisher()
    try:
        rclpy.spin(node=pospub)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutting down the node...")
    finally:
        pospub.gnss.port.close()
        pospub.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
