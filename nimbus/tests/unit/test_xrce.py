"""
Unit tests for the XRCE-DDS module.

Tests CDR serialization, message parsing, and protocol structures.
"""

import pytest
import struct
from nimbus.core.xrce.cdr import CDRReader, CDRWriter, parse_encapsulation, add_encapsulation
from nimbus.core.xrce.messages import (
    Time, Header, Vector3, Point, Quaternion, Pose,
    Twist, LaserScan, Odometry
)


class TestCDRReader:
    """Tests for CDR deserialization."""

    def test_read_uint8(self):
        reader = CDRReader(bytes([0x42]))
        assert reader.read_uint8() == 0x42

    def test_read_int8(self):
        reader = CDRReader(bytes([0xFF]))
        assert reader.read_int8() == -1

    def test_read_uint16(self):
        reader = CDRReader(struct.pack('<H', 0x1234))
        assert reader.read_uint16() == 0x1234

    def test_read_uint16_with_alignment(self):
        # One byte data + one byte padding + uint16
        reader = CDRReader(bytes([0x00, 0x00]) + struct.pack('<H', 0x1234))
        reader.read_uint8()  # Read one byte
        assert reader.read_uint16() == 0x1234  # Should align to 2 bytes (skipping 1 padding byte)

    def test_read_uint32(self):
        reader = CDRReader(struct.pack('<I', 0x12345678))
        assert reader.read_uint32() == 0x12345678

    def test_read_uint32_with_alignment(self):
        # Two bytes padding, then uint32
        data = bytes([0x01, 0x02, 0x00, 0x00]) + struct.pack('<I', 0xDEADBEEF)
        reader = CDRReader(data)
        reader.read_uint8()
        reader.read_uint8()
        assert reader.read_uint32() == 0xDEADBEEF

    def test_read_float32(self):
        reader = CDRReader(struct.pack('<f', 3.14159))
        assert abs(reader.read_float32() - 3.14159) < 0.00001

    def test_read_float64(self):
        reader = CDRReader(struct.pack('<d', 3.141592653589793))
        assert abs(reader.read_float64() - 3.141592653589793) < 1e-10

    def test_read_string(self):
        # String format: 4-byte length (including null) + chars + null
        data = struct.pack('<I', 6) + b'hello\x00'
        reader = CDRReader(data)
        assert reader.read_string() == 'hello'

    def test_read_empty_string(self):
        data = struct.pack('<I', 1) + b'\x00'
        reader = CDRReader(data)
        assert reader.read_string() == ''

    def test_read_float32_array(self):
        values = [1.0, 2.0, 3.0]
        data = struct.pack('<I', 3)  # count
        for v in values:
            data += struct.pack('<f', v)
        reader = CDRReader(data)
        result = reader.read_float32_array()
        assert len(result) == 3
        assert abs(result[0] - 1.0) < 0.0001
        assert abs(result[1] - 2.0) < 0.0001
        assert abs(result[2] - 3.0) < 0.0001


class TestCDRWriter:
    """Tests for CDR serialization."""

    def test_write_uint8(self):
        writer = CDRWriter()
        writer.write_uint8(0x42)
        assert writer.get_data() == bytes([0x42])

    def test_write_int8(self):
        writer = CDRWriter()
        writer.write_int8(-1)
        assert writer.get_data() == bytes([0xFF])

    def test_write_uint16(self):
        writer = CDRWriter()
        writer.write_uint16(0x1234)
        assert writer.get_data() == struct.pack('<H', 0x1234)

    def test_write_uint16_with_alignment(self):
        writer = CDRWriter()
        writer.write_uint8(0x01)
        writer.write_uint16(0x1234)
        data = writer.get_data()
        assert data[0] == 0x01
        assert data[1] == 0x00  # Padding
        assert struct.unpack_from('<H', data, 2)[0] == 0x1234

    def test_write_uint32(self):
        writer = CDRWriter()
        writer.write_uint32(0xDEADBEEF)
        assert writer.get_data() == struct.pack('<I', 0xDEADBEEF)

    def test_write_float32(self):
        writer = CDRWriter()
        writer.write_float32(3.14159)
        data = writer.get_data()
        assert abs(struct.unpack('<f', data)[0] - 3.14159) < 0.00001

    def test_write_float64(self):
        writer = CDRWriter()
        writer.write_float64(3.141592653589793)
        data = writer.get_data()
        assert abs(struct.unpack('<d', data)[0] - 3.141592653589793) < 1e-10

    def test_write_string(self):
        writer = CDRWriter()
        writer.write_string('hello')
        data = writer.get_data()
        length = struct.unpack_from('<I', data, 0)[0]
        assert length == 6  # 'hello' + null
        assert data[4:9] == b'hello'
        assert data[9] == 0  # Null terminator

    def test_write_float32_array(self):
        writer = CDRWriter()
        writer.write_float32_array([1.0, 2.0, 3.0])
        data = writer.get_data()
        count = struct.unpack_from('<I', data, 0)[0]
        assert count == 3
        values = [struct.unpack_from('<f', data, 4 + i*4)[0] for i in range(3)]
        assert abs(values[0] - 1.0) < 0.0001
        assert abs(values[1] - 2.0) < 0.0001
        assert abs(values[2] - 3.0) < 0.0001


class TestCDRRoundtrip:
    """Test that write/read round-trips work correctly."""

    def test_roundtrip_primitives(self):
        writer = CDRWriter()
        writer.write_uint8(0x42)
        writer.write_uint16(0x1234)
        writer.write_uint32(0xDEADBEEF)
        writer.write_float32(3.14159)
        writer.write_float64(2.718281828)

        reader = CDRReader(writer.get_data())
        assert reader.read_uint8() == 0x42
        assert reader.read_uint16() == 0x1234
        assert reader.read_uint32() == 0xDEADBEEF
        assert abs(reader.read_float32() - 3.14159) < 0.00001
        assert abs(reader.read_float64() - 2.718281828) < 1e-8

    def test_roundtrip_string(self):
        writer = CDRWriter()
        writer.write_string("test string")

        reader = CDRReader(writer.get_data())
        assert reader.read_string() == "test string"

    def test_roundtrip_array(self):
        values = [1.5, 2.5, 3.5, 4.5]
        writer = CDRWriter()
        writer.write_float32_array(values)

        reader = CDRReader(writer.get_data())
        result = reader.read_float32_array()
        assert len(result) == len(values)
        for i in range(len(values)):
            assert abs(result[i] - values[i]) < 0.0001


class TestEncapsulation:
    """Test CDR encapsulation header handling."""

    def test_parse_little_endian(self):
        data = bytes([0x00, 0x01, 0x00, 0x00]) + b'payload'
        is_le, payload = parse_encapsulation(data)
        assert is_le is True
        assert payload == b'payload'

    def test_parse_big_endian(self):
        data = bytes([0x00, 0x00, 0x00, 0x00]) + b'payload'
        is_le, payload = parse_encapsulation(data)
        assert is_le is False
        assert payload == b'payload'

    def test_add_encapsulation(self):
        payload = b'test'
        result = add_encapsulation(payload, little_endian=True)
        assert result[:4] == bytes([0x00, 0x01, 0x00, 0x00])
        assert result[4:] == b'test'


class TestTwistMessage:
    """Tests for Twist message serialization."""

    def test_create_twist(self):
        twist = Twist.create(linear_x=0.5, angular_z=1.0)
        assert twist.linear.x == 0.5
        assert twist.linear.y == 0.0
        assert twist.linear.z == 0.0
        assert twist.angular.x == 0.0
        assert twist.angular.y == 0.0
        assert twist.angular.z == 1.0

    def test_twist_serialize_deserialize(self):
        original = Twist.create(linear_x=0.25, angular_z=-0.5)
        data = original.serialize()

        restored = Twist.deserialize(data)
        assert abs(restored.linear.x - 0.25) < 1e-6
        assert abs(restored.angular.z - (-0.5)) < 1e-6


class TestVector3:
    """Tests for Vector3 message."""

    def test_vector3_from_cdr(self):
        writer = CDRWriter()
        writer.write_float64(1.0)
        writer.write_float64(2.0)
        writer.write_float64(3.0)

        reader = CDRReader(writer.get_data())
        vec = Vector3.from_cdr(reader)
        assert vec.x == 1.0
        assert vec.y == 2.0
        assert vec.z == 3.0

    def test_vector3_to_cdr(self):
        vec = Vector3(x=1.0, y=2.0, z=3.0)
        writer = CDRWriter()
        vec.to_cdr(writer)

        reader = CDRReader(writer.get_data())
        assert reader.read_float64() == 1.0
        assert reader.read_float64() == 2.0
        assert reader.read_float64() == 3.0


class TestTimeAndHeader:
    """Tests for Time and Header messages."""

    def test_time_roundtrip(self):
        original = Time(sec=12345, nanosec=678900000)
        writer = CDRWriter()
        original.to_cdr(writer)

        reader = CDRReader(writer.get_data())
        restored = Time.from_cdr(reader)
        assert restored.sec == 12345
        assert restored.nanosec == 678900000

    def test_header_roundtrip(self):
        original = Header(
            stamp=Time(sec=100, nanosec=500000000),
            frame_id="base_link"
        )
        writer = CDRWriter()
        original.to_cdr(writer)

        reader = CDRReader(writer.get_data())
        restored = Header.from_cdr(reader)
        assert restored.stamp.sec == 100
        assert restored.stamp.nanosec == 500000000
        assert restored.frame_id == "base_link"


class TestLaserScan:
    """Tests for LaserScan message."""

    def test_laserscan_roundtrip(self):
        ranges = [1.0, 1.5, 2.0, float('inf'), 0.5]
        original = LaserScan(
            header=Header(stamp=Time(sec=1, nanosec=0), frame_id="laser"),
            angle_min=0.0,
            angle_max=3.14159,
            angle_increment=0.01745,
            time_increment=0.0,
            scan_time=0.1,
            range_min=0.1,
            range_max=10.0,
            ranges=ranges,
            intensities=[100.0] * len(ranges)
        )

        data = original.serialize()
        restored = LaserScan.deserialize(data)

        assert restored.header.frame_id == "laser"
        assert abs(restored.angle_max - 3.14159) < 1e-5
        assert len(restored.ranges) == 5
        assert abs(restored.ranges[0] - 1.0) < 1e-5


class TestOdometry:
    """Tests for Odometry message."""

    def test_pose_roundtrip(self):
        original = Pose(
            position=Point(x=1.0, y=2.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
        )
        writer = CDRWriter()
        original.to_cdr(writer)

        reader = CDRReader(writer.get_data())
        restored = Pose.from_cdr(reader)
        assert abs(restored.position.x - 1.0) < 1e-6
        assert abs(restored.position.y - 2.0) < 1e-6
        assert abs(restored.orientation.z - 0.707) < 1e-6
        assert abs(restored.orientation.w - 0.707) < 1e-6


class TestMessageCompatibility:
    """Tests that direct messages are compatible with existing processors."""

    def test_laserscan_has_ranges_attribute(self):
        """LaserScan must have ranges attribute for LidarProcessor."""
        scan = LaserScan(ranges=[1.0, 2.0, 3.0])
        assert hasattr(scan, 'ranges')
        assert scan.ranges == [1.0, 2.0, 3.0]

    def test_odometry_has_pose_structure(self):
        """Odometry must have pose.pose.position/orientation for OdometryProcessor."""
        odom = Odometry()
        # Check the nested structure exists
        assert hasattr(odom, 'pose')
        assert hasattr(odom.pose, 'pose')
        assert hasattr(odom.pose.pose, 'position')
        assert hasattr(odom.pose.pose, 'orientation')
        assert hasattr(odom.pose.pose.position, 'x')
        assert hasattr(odom.pose.pose.position, 'y')
        assert hasattr(odom.pose.pose.orientation, 'x')
        assert hasattr(odom.pose.pose.orientation, 'w')

    def test_odometry_has_twist_structure(self):
        """Odometry must have twist.twist.linear/angular for OdometryProcessor."""
        odom = Odometry()
        assert hasattr(odom, 'twist')
        assert hasattr(odom.twist, 'twist')
        assert hasattr(odom.twist.twist, 'linear')
        assert hasattr(odom.twist.twist, 'angular')
        assert hasattr(odom.twist.twist.linear, 'x')
        assert hasattr(odom.twist.twist.angular, 'z')

    def test_twist_has_linear_angular(self):
        """Twist must have linear.x and angular.z for velocity commands."""
        twist = Twist.create(linear_x=0.5, angular_z=1.0)
        assert hasattr(twist, 'linear')
        assert hasattr(twist, 'angular')
        assert hasattr(twist.linear, 'x')
        assert hasattr(twist.angular, 'z')
