#!/usr/bin/env python3

# Python Libraries
import array

# ROS2 Libraries
import rclpy
from acoustics_data_record_lib import AcousticsDataRecordLib
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class AcousticsDataRecordNode(Node):
    def __init__(self) -> None:
        # Variables for setting upp logging correctly
        hydrophone_data_size = (2**10) * 3  # 1 hydrophone buffer is 2^10 long, Each hydrophone data has 3 buffers full of this data
        dsp_data_size = 2**10  # DSP (Digital Signal Processing) has 2^10 long data
        tdoa_data_size = 5  # TDOA (Time Difference Of Arrival) has 5 hydrophones it has times for
        position_data_size = 3  # position only has X, Y, Z basically 3 elements

        # Initialize ROS2 node
        super().__init__("acoustics_data_record_node")

        # Initialize Subscribers ----------
        # Start listening to Hydrophone data
        self.subscriber_hydrophone1 = self.create_subscription(Int32MultiArray, "/acoustics/hydrophone1", self.hydrophone1_callback, 5)
        self.hydrophone1_data = array.array("i", [0] * hydrophone_data_size)

        self.subscriber_hydrophone2 = self.create_subscription(Int32MultiArray, "/acoustics/hydrophone2", self.hydrophone2_callback, 5)
        self.hydrophone2_data = array.array("i", [0] * hydrophone_data_size)

        self.subscriber_hydrophone3 = self.create_subscription(Int32MultiArray, "/acoustics/hydrophone3", self.hydrophone3_callback, 5)
        self.hydrophone3_data = array.array("i", [0] * hydrophone_data_size)

        self.subscriber_hydrophone4 = self.create_subscription(Int32MultiArray, "/acoustics/hydrophone4", self.hydrophone4_callback, 5)
        self.hydrophone4_data = array.array("i", [0] * hydrophone_data_size)

        self.subscriber_hydrophone5 = self.create_subscription(Int32MultiArray, "/acoustics/hydrophone5", self.hydrophone5_callback, 5)
        self.hydrophone5_data = array.array("i", [0] * hydrophone_data_size)

        # Start listening to DSP (Digital Signal Processing) data
        self.subscriber_filter_response = self.create_subscription(
            Int32MultiArray,
            "/acoustics/filter_response",
            self.filter_response_callback,
            5,
        )
        self.filter_response_data = array.array("i", [0] * dsp_data_size)

        self.subscriber_fft = self.create_subscription(Int32MultiArray, "/acoustics/fft", self.fft_callback, 5)
        self.fft_data = array.array("i", [0] * dsp_data_size)

        self.subscriber_peaks = self.create_subscription(Int32MultiArray, "/acoustics/peaks", self.peaks_callback, 5)
        self.peaks_data = array.array("i", [0] * dsp_data_size)

        # Start listening to Multilateration data
        self.subscriber_tdoa_response = self.create_subscription(
            Float32MultiArray,
            "/acoustics/time_difference_of_arrival",
            self.tdoa_callback,
            5,
        )
        self.tdoa_data = array.array("f", [0.0] * tdoa_data_size)

        self.subscriber_position_response = self.create_subscription(Float32MultiArray, "/acoustics/position", self.position_callback, 5)
        self.position_data = array.array("f", [0.0] * position_data_size)

        # Initialize logger ----------
        # Get package directory location
        ros2_package_directory_location = get_package_share_directory("acoustics_data_record")
        ros2_package_directory_location = ros2_package_directory_location + "/../../../../"  # go back to workspace
        ros2_package_directory_location = (
            ros2_package_directory_location + "src/vortex-auv/acoustics/acoustics_data_record/"
        )  # Navigate to this package

        # Make blackbox logging file
        self.acoustics_data_record = AcousticsDataRecordLib(ros2_package_directory=ros2_package_directory_location)

        # Logs all the newest data 1 time(s) per second
        self.declare_parameter("acoustics.data_logging_rate", 1.0)  # Providing a default value 1.0 => 1 samplings per second, very slow
        data_loging_rate = self.get_parameter("acoustics.data_logging_rate").get_parameter_value().double_value
        timer_period = 1.0 / data_loging_rate
        self.logger_timer = self.create_timer(timer_period, self.logger)

        # Debugging ----------
        self.get_logger().info(
            "Started logging data for topics: \n"
            "/acoustics/hydrophone1 [Int32MultiArray] \n"
            "/acoustics/hydrophone2 [Int32MultiArray] \n"
            "/acoustics/hydrophone3 [Int32MultiArray] \n"
            "/acoustics/hydrophone4 [Int32MultiArray] \n"
            "/acoustics/hydrophone5 [Int32MultiArray] \n"
            "/acoustics/filter_response [Int32MultiArray] \n"
            "/acoustics/fft [Int32MultiArray] \n"
            "/acoustics/peaks [Int32MultiArray] \n"
            "/acoustics/time_difference_of_arrival [Float32MultiArray] \n"
            "/acoustics/position [Float32MultiArray] \n"
        )

    # Callback methods for different topics
    def hydrophone1_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for hydrophone1 topic.

        Args:
            msg (Int32MultiArray): Message containing hydrophone1 data.
        """
        self.hydrophone1_data = msg.data

    def hydrophone2_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for hydrophone2 topic.

        Args:
            msg (Int32MultiArray): Message containing hydrophone2 data.
        """
        self.hydrophone2_data = msg.data

    def hydrophone3_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for hydrophone3 topic.

        Args:
            msg (Int32MultiArray): Message containing hydrophone3 data.
        """
        self.hydrophone3_data = msg.data

    def hydrophone4_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for hydrophone4 topic.

        Args:
            msg (Int32MultiArray): Message containing hydrophone4 data.
        """
        self.hydrophone4_data = msg.data

    def hydrophone5_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for hydrophone5 topic.

        Args:
            msg (Int32MultiArray): Message containing hydrophone5 data.
        """
        self.hydrophone5_data = msg.data

    def filter_response_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for filter_response topic.

        Args:
            msg (Int32MultiArray): Message containing filter response data.
        """
        self.filter_response_data = msg.data

    def fft_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for fft topic.

        Args:
            msg (Int32MultiArray): Message containing FFT data.
        """
        self.fft_data = msg.data

    def peaks_callback(self, msg: Int32MultiArray) -> None:
        """
        Callback method for peaks topic.

        Args:
            msg (Int32MultiArray): Message containing peaks data.
        """
        self.peaks_data = msg.data

    def tdoa_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback method for time_difference_of_arrival topic.

        Args:
            msg (Float32MultiArray): Message containing TDOA data.
        """
        self.tdoa_data = msg.data

    def position_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback method for position topic.

        Args:
            msg (Float32MultiArray): Message containing position data.
        """
        self.position_data = msg.data

    # The logger that logs all the data
    def logger(self) -> None:
        """
        Logs all the data to a CSV file using the AcousticsDataRecordLib.

        This method is called periodically based on the data logging rate.
        """
        self.acoustics_data_record.log_data_to_csv_file(
            hydrophone1=self.hydrophone1_data,
            hydrophone2=self.hydrophone2_data,
            hydrophone3=self.hydrophone3_data,
            hydrophone4=self.hydrophone4_data,
            hydrophone5=self.hydrophone5_data,
            filter_response=self.filter_response_data,
            fft=self.fft_data,
            peaks=self.peaks_data,
            tdoa=self.tdoa_data,
            position=self.position_data,
        )


def main() -> None:
    """
    Main function to initialize and run the ROS2 node for acoustics data recording.

    This function performs the following steps:
    1. Initializes the ROS2 communication.
    2. Creates an instance of the AcousticsDataRecordNode.
    3. Spins the node to keep it running until an external shutdown signal is received.
    4. Destroys the node explicitly once ROS2 stops running.
    5. Shuts down the ROS2 communication.

    Returns:
        None
    """
    # Initialize ROS2
    rclpy.init()

    # Start the ROS2 node and continue forever until exit
    acoustics_data_record_node = AcousticsDataRecordNode()
    rclpy.spin(acoustics_data_record_node)

    # Destroy the node explicitly once ROS2 stops running
    acoustics_data_record_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
