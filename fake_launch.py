
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([

        # Camera Node
        Node(
            package='usb_cam', # Replace with your camera driver package name (e.g., 'usb_cam')
            executable='usb_cam_node', # Replace with your camera driver executable (e.g., 'usb_cam_node')
            name='camera_node',
            output='screen',
            parameters=[
                # Add any camera specific parameters here (e.g., device_id, image_width, image_height, etc.)
                # {'device_id': 0},
                # {'image_width': 640},
                # {'image_height': 480},
            ]
        ),

        # GPS Node
        Node(
            package='<YOUR_GPS_PACKAGE_NAME>', # Replace with your GPS driver package name
            executable='<YOUR_GPS_NODE_EXECUTABLE>', # Replace with your GPS driver executable
            name='gps_node',
            output='screen',
            parameters=[
                # Add any GPS specific parameters here (e.g., port, baud_rate)
                # {'port': '/dev/ttyUSB0'},
                # {'baud_rate': 115200},
            ]
        ),

        # Image Processing Node (gps_aruco.py)
        Node(
            package='nova_vision', # Replace with the package where gps_aruco.py resides
            executable='gps_aruco.py',
            name='gps_aruco_node',
            output='screen',
            # You can remap topics here if your script uses different topic names
            # remappings=[
            #     ('/image_raw', '/camera/image_raw'),
            #     ('/gps/fix', '/your_gps_topic'),
            # ]
        ),
    ])