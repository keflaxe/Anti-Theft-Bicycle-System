import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from flask import Flask, jsonify
from flask_cors import CORS

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'dash_data',
            self.dash_data_callback,
            10
        )
        
        # Shared latitude/longitude variables with a lock
        self.lock = threading.Lock()
        self.coordinates = [0.0, 0.0]  # Default [lat, lon]

        # Start Flask in a separate thread
        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()

    def dash_data_callback(self, msg):
        """Extracts latitude and longitude from dash_data (index 18 & 19)."""
        if len(msg.data) == 2:
            lat, lon = msg.data[0], msg.data[1]

            with self.lock:
                self.coordinates = [lat, lon]

    def run_flask(self):
        """Runs the Flask API to serve coordinates."""
        app = Flask(__name__)
        CORS(app)

        @app.route('/get_coordinates')
        def get_coordinates():
            with self.lock:
                return jsonify(self.coordinates)

        app.run(debug=False, host='127.0.0.1', port=5000, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
