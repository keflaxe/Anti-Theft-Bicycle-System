import sys
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class PlotterNode(Node):
    def __init__(self):
        super().__init__('Plotter_Node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'dash_data',
            self.dash_data_callback,
            10
        )

        # Storage for plotting
        self.times = []
        self.values_50 = []
        self.values_40 = []
        self.values_30 = []
        self.start_time = time.time()

        self.lock = threading.Lock()

        # Start the plotting thread
        threading.Thread(target=self.plot_thread, daemon=True).start()

    def dash_data_callback(self, msg):
        with self.lock:
            current_time = time.time() - self.start_time  # Time since start
            if len(msg.data) > 50:  # Make sure indexes exist
                self.times.append(current_time)
                self.values_50.append(msg.data[30])
                self.values_40.append(msg.data[34])
                self.values_30.append(msg.data[68])

    def plot_thread(self):
        plt.ion()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))  # 3 rows, 1 column

        # Line objects
        line1, = ax1.plot([], [], 'b-', label='Voltage')
        line2, = ax2.plot([], [], 'r-', label='Speed')
        line3, = ax3.plot([], [], 'g-', label='Current')

        # Set titles and labels
        ax1.set_title('Voltage vs Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Voltage (V)')
        ax1.grid(True)
        ax1.legend()

        ax2.set_title('Speed vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (kmph)')
        ax2.grid(True)
        ax2.legend()

        ax3.set_title('Current vs Time')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Current (A)')
        ax3.grid(True)
        ax3.legend()

        plt.tight_layout()

        while rclpy.ok():
            with self.lock:
                # Update data for each line
                line1.set_xdata(self.times)
                line1.set_ydata(self.values_50)

                line2.set_xdata(self.times)
                line2.set_ydata(self.values_40)

                line3.set_xdata(self.times)
                line3.set_ydata(self.values_30)

                if self.times:
                    # Adjust x and y limits dynamically
                    for ax, values in zip([ax1, ax2, ax3], [self.values_50, self.values_40, self.values_30]):
                        ax.set_xlim(self.times[0], self.times[-1] + 1)
                        ax.set_ylim(min(values) - 1, max(values) + 1)

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)  # Update every 100 ms

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
