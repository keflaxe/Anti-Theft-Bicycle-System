import rclpy
from rclpy.node import Node
from supabase import create_client, Client
import os
import sys
import time
from std_msgs.msg import Float32MultiArray as rosarray


class READER_NODE(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.url = 'https://xtjvcasaxiaunasbfhra.supabase.co'
        self.key = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Inh0anZjYXNheGlhdW5hc2JmaHJhIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDQ4OTQ0ODEsImV4cCI6MjA2MDQ3MDQ4MX0.KBW-GDDh328Vu6J-pRJ8QhfWl6y5weL2fMaHM9aBIYM'
        self.supabase: Client = create_client(self.url, self.key)

        self.timer = self.create_timer(1.0, self.read_data_periodically)  # read every 3 seconds

    def read_data_periodically(self):
        try:
            response = self.supabase.table("gps").select("*").order("timestamp", desc=True).limit(1).execute()
            data = response.data

            if data:
                self.dash_pub_data = []  
                for key, value in data[0].items():  
                    if isinstance(value, (int, float)):  # Only keep numeric fields
                        self.dash_pub_data.append(float(value))
                self.get_logger().info(f"Fetched data: {self.dash_pub_data}")
            else:
                self.get_logger().warn("No data found in gps table.")
        except Exception as e:
            self.get_logger().error(f"Error reading from Supabase: {str(e)}")

    def init_dash_data_publisher(self, topic, timer_period):
        """Initialize ROS publisher for dashboard data."""
        self.dash_data_publisher = self.create_publisher(rosarray, topic, 150)
        self.dash_data_timer = self.create_timer(timer_period, self.publish_dash_data)
        self.dash_pub_data = None

    def publish_dash_data(self):
        if self.dash_pub_data is not None:
            msg = rosarray()
            msg.data = self.dash_pub_data        
            self.dash_data_publisher.publish(msg)
            self.dash_pub_data = msg.data  # Update last published data


def main(args=None):
    rclpy.init(args=args)
    reader_node = READER_NODE("reader_node")
    reader_node.init_dash_data_publisher('dash_data',1)
    rclpy.spin(reader_node)
    reader_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
