from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .core import DataFormatError, decode, diagnose_stub, encode, normalize, sample, validate_report

RAW = "/telemetry/raw"
NORMALIZED = "/telemetry/normalized"
REPORT = "/diagnostics/report"
STATUS = "/diagnostics/status"


def qos():


    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Source(Node):
    def __init__(self):
        super().__init__("synthetic_source")
        self.run_id = self.declare_parameter("run_id", "demo").value
        self.vehicle_id = self.declare_parameter("vehicle_id", "auv-01").value
        self.count = self.declare_parameter("count", 5).value
        if type(self.count) is not int or not 1 <= self.count <= 100:
            raise ValueError("count must be between 1 and 100")
        sample(self.run_id, self.vehicle_id, 0)
        self.publisher = self.create_publisher(String, RAW, qos())
        self.sequence = 0
        self.create_timer(0.2, self.tick)

    def tick(self):
        if self.sequence >= self.count or self.publisher.get_subscription_count() == 0:
            return
        message = sample(self.run_id, self.vehicle_id, self.sequence)
        self.publisher.publish(String(data=encode(message)))
        self.sequence += 1


class Transform(Node):
    def __init__(self, name, input_topic, output_topic, transform):
        super().__init__(name)
        self.transform = transform
        self.publisher = self.create_publisher(String, output_topic, qos())
        self.status = self.create_publisher(String, STATUS, qos())
        self.create_subscription(String, input_topic, self.receive, qos())

    def receive(self, message):
        try:
            output = self.transform(decode(message.data))
        except DataFormatError as error:
            self.get_logger().error(str(error))
            self.status.publish(String(data=encode({"node": self.get_name(),
                                                    "status": "invalid_input", "error": str(error)})))
            return
        self.publisher.publish(String(data=encode(output)))


class Recorder(Node):
    def __init__(self):
        super().__init__("report_recorder")
        self.count = self.declare_parameter("count", 5).value
        self.run_id = self.declare_parameter("run_id", "demo").value
        self.vehicle_id = self.declare_parameter("vehicle_id", "auv-01").value
        if type(self.count) is not int or not 1 <= self.count <= 100:
            raise ValueError("count must be between 1 and 100")
        output = Path(self.declare_parameter("output_dir", "/tmp/auv-demo").value)
        output.mkdir(parents=True, exist_ok=False)
        self.file = (output / "reports.jsonl").open("x", encoding="utf-8")
        self.seen = set()
        self.done = False
        self.create_subscription(String, REPORT, self.receive, qos())

    def receive(self, message):
        report = validate_report(decode(message.data))
        if (report["run_id"], report["vehicle_id"]) != (self.run_id, self.vehicle_id):
            return
        sequence = report["sequence"]
        if sequence >= self.count:
            raise DataFormatError("Unexpected sequence for this run")
        if sequence in self.seen:
            return
        self.file.write(encode(report) + "\n")
        self.file.flush()
        self.seen.add(sequence)
        self.get_logger().info(f"REPORT run_id={self.run_id} sequence={sequence} status=stub")
        self.done = len(self.seen) == self.count


def spin(factory):
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def source_main():
    spin(Source)


def adapter_main():
    spin(lambda: Transform("telemetry_adapter", RAW, NORMALIZED, normalize))


def stub_main():
    spin(lambda: Transform("diagnostic_stub", NORMALIZED, REPORT, diagnose_stub))


def recorder_main():
    rclpy.init()
    node = None
    try:
        node = Recorder()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.done:
            node.get_logger().info("DEMO_COMPLETE")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
