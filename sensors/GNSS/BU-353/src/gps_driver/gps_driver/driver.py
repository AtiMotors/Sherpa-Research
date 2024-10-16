import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from msg_package.msg import GPSmsg
from utm.error import OutOfRangeError
from datetime import datetime,timezone
from builtin_interfaces.msg import Time
import utm
import serial

ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('GPS_Data_Pub')
        SENSOR_NAME ="BU_353_GPS"
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', '4800')
        self.declare_parameter('sampling_rate', 10)
        self.publisher_ = self.create_publisher(GPSmsg, '/gps', 10)
        self.get_logger().debug("Initialization complete")
        self.get_logger().info("Publishing GPS Data")

        ser_port = self.get_parameter('port').value
        ser_baud = self.get_parameter('baudrate').value
        sampling_rate = self.get_parameter('sampling_rate').value
        self.port = serial.Serial(ser_port, ser_baud, timeout=3.0)
        self.get_logger().debug("Starting " + SENSOR_NAME + "on the serial port" + ser_port + " at baud rate " + str(ser_baud))

        self.sleep_time = 1 / sampling_rate
        self.timer = self.create_timer(self.sleep_time, self.gps_data_publish)

    def gps_data_publish(self):
        
        inc_data = self.port.readline().decode('ascii', errors='replace').strip()
        if inc_data.startswith('$GPGGA'):

            or_str = GPSmsg()
            new_str = inc_data.split(',', 10)
            gpgga_time_data = new_str[1]
            gpgga_time_conv = self.gpgga_ros_timer(gpgga_time_data)
            
            gpgga_latitude_data = new_str[2]
            gpgga_latitude_direction = new_str[3]
            gpgga_longitude_data = new_str[4]
            gpgga_longitude_direction = new_str[5]
        
            latitude = lat_min_deg(gpgga_latitude_data, gpgga_latitude_direction)
            longitude = lon_min_deg(gpgga_longitude_data, gpgga_longitude_direction)
 	
            utm_data = utm.from_latlon(latitude, longitude)
            utm_easting = utm_data[0]
            utm_northing = utm_data[1]
            zone = utm_data[2]
            letter = utm_data[3]

            if not 1 <= zone <= 60:
                raise OutOfRangeError('zone number out of range (must be between 1 and 60)')
            
            if letter:
                letter = letter.upper()

            if not 'C' <= letter <= 'X' or letter in ['I', 'O']:
                raise OutOfRangeError('zone letter out of range (must be between C and X)')
            
            or_str.header = Header()
            or_str.header.frame_id = 'GPS1_Frame'
            or_str.header.stamp = gpgga_time_conv
            or_str.latitude = latitude
            or_str.longitude = longitude
            or_str.utm_easting = utm_easting 
            or_str.utm_northing = utm_northing
            or_str.zone = zone 
            or_str.letter = letter 

            self.publisher_.publish(or_str)
            self.get_logger().info(f'Published GPS data: {or_str}')

    def gpgga_ros_timer(self, gpgga_time_data):

        hh = int(gpgga_time_data[0:2])
        mm = int(gpgga_time_data[2:4])
        ss = float(gpgga_time_data[4:])
        now = datetime.now (timezone.utc)
        time_struct = datetime(now.year, now.month, now.day, hh, mm, int(ss), int((ss % 1) * 1e6 ))
        
        ros_timer = Time()
        ros_timer.sec = int(time_struct.timestamp())
       
        return ros_timer

# Convert latitude to degrees

def lat_min_deg(latitude_data, latitude_direction):

    if not latitude_data or len(latitude_data) < 4:
        return 0.0
    d_lat = latitude_data[:2]
    m_lat = latitude_data[2:] 
    lat_d = float(d_lat) + float(m_lat)/60  
    if latitude_direction == 'S':
        lat_d *= -1
    return lat_d

# Convert longitude to degrees

def lon_min_deg(longitude_data, longitude_direction):

    if not longitude_data or len(longitude_data) < 5:
        return 0.0
    d_lon = longitude_data[:3]
    m_lon = longitude_data[3:]
    lon_d = float(d_lon) + float(m_lon)/60
    if longitude_direction == 'W':
        lon_d *= -1
    return lon_d

def main(args=None):

    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_publisher.port.close()
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
