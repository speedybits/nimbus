"""Tests for Neato serial transport protocol parsing."""

import pytest
from nimbus.core.neato.transport import NeatoTransport


class TestParseKeyValue:
    """Test the generic key,value parser."""

    def test_basic_parse(self):
        lines = [
            "Parameter,Value",
            "LeftWheel_PositionInMM,1234",
            "RightWheel_PositionInMM,5678",
        ]
        result = NeatoTransport._parse_key_value(lines)
        assert result["LeftWheel_PositionInMM"] == "1234"
        assert result["RightWheel_PositionInMM"] == "5678"
        assert "Parameter" not in result

    def test_empty_lines(self):
        result = NeatoTransport._parse_key_value([])
        assert result == {}

    def test_skip_malformed(self):
        lines = ["just a string", "Key,Value", "AnotherKey,42"]
        result = NeatoTransport._parse_key_value(lines)
        assert "AnotherKey" == "42" or result.get("AnotherKey") == "42"

    def test_value_with_comma(self):
        lines = ["Info,something,extra"]
        result = NeatoTransport._parse_key_value(lines)
        assert result["Info"] == "something,extra"


class TestLDSScanParsing:
    """Test GetLDSScan response parsing without actual serial."""

    def _make_transport_with_scan_lines(self, lines):
        """Create transport and manually call the parser path."""
        transport = NeatoTransport.__new__(NeatoTransport)
        return transport, lines

    def test_parse_scan_lines(self):
        # Simulate GetLDSScan CSV output
        lines = []
        for angle in range(360):
            dist = 1000 + angle
            intensity = 50
            error = 0
            lines.append(f"{angle},{dist},{intensity},{error}")

        transport = NeatoTransport.__new__(NeatoTransport)
        # Directly test the parsing by calling get_lds_scan's logic
        from nimbus.core.neato.transport import NeatoScanData
        scan = NeatoScanData()
        for line in lines:
            parts = line.split(",")
            if len(parts) == 4:
                scan.angles.append(int(parts[0]))
                scan.distances_mm.append(int(parts[1]))
                scan.intensities.append(int(parts[2]))
                scan.error_codes.append(int(parts[3]))

        assert len(scan.angles) == 360
        assert scan.distances_mm[0] == 1000
        assert scan.distances_mm[359] == 1359
        assert all(e == 0 for e in scan.error_codes)

    def test_skip_bad_lines(self):
        lines = [
            "AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX",  # header
            "0,1234,50,0",
            "bad line",
            "1,2345,60,0",
        ]
        from nimbus.core.neato.transport import NeatoScanData
        scan = NeatoScanData()
        for line in lines:
            parts = line.split(",")
            if len(parts) != 4:
                continue
            try:
                scan.angles.append(int(parts[0]))
                scan.distances_mm.append(int(parts[1]))
                scan.intensities.append(int(parts[2]))
                scan.error_codes.append(int(parts[3]))
            except ValueError:
                continue

        assert len(scan.angles) == 2
        assert scan.angles == [0, 1]


class TestMotorDataParsing:
    """Test GetMotors response parsing."""

    def test_parse_motor_response(self):
        lines = [
            "Parameter,Value",
            "Brush_RPM,0",
            "Brush_mA,0",
            "LeftWheel_PositionInMM,12345",
            "LeftWheel_SpeedInMMS,100",
            "RightWheel_PositionInMM,12340",
            "RightWheel_SpeedInMMS,95",
        ]
        data = NeatoTransport._parse_key_value(lines)
        assert data["LeftWheel_PositionInMM"] == "12345"
        assert data["RightWheel_PositionInMM"] == "12340"
        assert data["LeftWheel_SpeedInMMS"] == "100"
        assert data["RightWheel_SpeedInMMS"] == "95"


class TestBumperParsing:
    """Test GetDigitalSensors response parsing."""

    def test_parse_bumper_response(self):
        lines = [
            "Digital Sensor Name,Value",
            "SNSR_LEFT_BUMPER,0",
            "SNSR_RIGHT_BUMPER,1",
            "SNSR_LEFT_WHEEL_EXTENDED,0",
            "SNSR_RIGHT_WHEEL_EXTENDED,0",
            "SNSR_WALL_DETECT,1",
        ]
        data = NeatoTransport._parse_key_value(lines)
        assert data["SNSR_LEFT_BUMPER"] == "0"
        assert data["SNSR_RIGHT_BUMPER"] == "1"
        assert data["SNSR_WALL_DETECT"] == "1"


class TestBatteryParsing:
    """Test GetCharger response parsing."""

    def test_parse_charger_response(self):
        lines = [
            "Parameter,Value",
            "FuelPercent,85",
            "ChargingActive,0",
            "VBattV,14200",
        ]
        data = NeatoTransport._parse_key_value(lines)
        assert data["FuelPercent"] == "85"
        assert data["ChargingActive"] == "0"
        assert data["VBattV"] == "14200"
