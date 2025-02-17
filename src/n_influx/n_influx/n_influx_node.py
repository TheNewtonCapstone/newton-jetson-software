#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from rcl_interfaces.msg import ParameterDescriptor


class InfluxNode(Node):
    def __init__(self):
        super().__init__("influx_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "influxdb.url",
                    "http://localhost:8086",
                    ParameterDescriptor(description="InfluxDB server URL"),
                ),
                (
                    "influxdb.org",
                    "newton",
                    ParameterDescriptor(description="InfluxDB organization"),
                ),
                (
                    "influxdb.bucket",
                    "newton_metrics",
                    ParameterDescriptor(description="InfluxDB bucket name"),
                ),
                (
                    "influxdb.token",
                    "",
                    ParameterDescriptor(description="InfluxDB authentication token"),
                ),
                (
                    "influxdb.username",
                    "",
                    ParameterDescriptor(description="InfluxDB username"),
                ),
                (
                    "influxdb.password",
                    "",
                    ParameterDescriptor(description="InfluxDB password"),
                ),
            ],
        )

        self.setup_influxdb_connection()

    def setup_influxdb_connection(self):
        try:
            self.url = self.get_parameter("influxdb.url").value
            self.org = self.get_parameter("influxdb.org").value
            self.bucket = self.get_parameter("influxdb.bucket").value
            self.token = self.get_parameter("influxdb.token").value

            self.get_logger().info(f"Configuring InfluxDB connection to {self.url}")

            self.client = InfluxDBClient(url=self.url, token=self.token, org=self.org)

            self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
            self.test_connection()

        except Exception as e:
            self.get_logger().error(f"Failed to initialize InfluxDB client: {str(e)}")
            raise

    def test_connection(self):
        try:
            health = self.client.health()
            if health.status != "pass":
                raise ConnectionError(f"InfluxDB health check failed: {health.message}")

            buckets_api = self.client.buckets_api()
            bucket = buckets_api.find_bucket_by_name(self.bucket)
            if not bucket:
                raise ValueError(f'Bucket "{self.bucket}" not found')

            self.get_logger().info("Successfully connected to InfluxDB")

        except Exception as e:
            self.get_logger().error(f"Connection test failed: {str(e)}")
            raise

    def write_metric(self, measurement, tags, fields):
        try:
            point = Point(measurement)

            for key, value in tags.items():
                point = point.tag(key, value)

            for key, value in fields.items():
                point = point.field(key, value)

            self.write_api.write(bucket=self.bucket, org=self.org, record=point)

        except Exception as e:
            self.get_logger().error(f"Failed to write metric {measurement}: {str(e)}")
            raise


def main():
    rclpy.init()
    node = InfluxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "client"):
            node.client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
