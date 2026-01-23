import serial
import can
import time
import struct
import math

# -----------------------------
# CONFIGURATION
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

CAN_INTERFACE = "can0"

# SAE J1939 PGNs
PGN_VEHICLE_POSITION = 0xFEF3      # 65267 - Vehicle Position
PGN_VEHICLE_DIRECTION_SPEED = 0xFEE8  # 65256 - Vehicle Direction/Speed

J1939_PRIORITY = 6
SOURCE_ADDRESS = 0x80
# -----------------------------


def j1939_build_can_id(pgn: int, sa: int, priority: int = 6) -> int:
    """
    Build a 29-bit SAE J1939 CAN ID
    bits 26-28: priority (3 bits)
    bit 25: reserved (0)
    bit 24: data page (0)
    bits 16-23: PF (PDU Format)
    bits 8-15: PS (PDU Specific / Group Extension)
    bits 0-7: source address
    """
    pf = (pgn >> 8) & 0xFF
    ps = pgn & 0xFF
    dp = 0  # Data page 0
    
    return ((priority & 0x7) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | (sa & 0xFF)


def encode_vehicle_position(lat_deg: float, lon_deg: float) -> bytes:
    """
    SAE J1939 Vehicle Position (PGN 0xFEF3 / 65267)
    
    Byte 1-4: Latitude  (resolution: 1e-7 deg, offset: -210 deg)
    Byte 5-8: Longitude (resolution: 1e-7 deg, offset: -210 deg)
    
    Per J1939-71:
    Latitude:  offset = -210, scale = 1e-7 deg/bit
    Longitude: offset = -210, scale = 1e-7 deg/bit
    """
    # Apply offset and scale
    # Value = (raw * resolution) + offset
    # raw = (Value - offset) / resolution
    lat_raw = int((lat_deg + 210.0) / 1e-7)
    lon_raw = int((lon_deg + 210.0) / 1e-7)
    
    # Clamp to uint32 range
    lat_raw = max(0, min(0xFFFFFFFF, lat_raw))
    lon_raw = max(0, min(0xFFFFFFFF, lon_raw))
    
    return struct.pack("<II", lat_raw, lon_raw)


def encode_vehicle_direction_speed(cog_deg: float, sog_ms: float) -> bytes:
    """
    SAE J1939 Vehicle Direction/Speed (PGN 0xFEE8 / 65256)
    
    Byte 1-2: Compass Bearing (resolution: 1/128 deg, 0-360 deg)
    Byte 3-4: Navigation-based Vehicle Speed (resolution: 1/256 km/h)
    Byte 5-6: Pitch (resolution: 1/128 deg, offset: -200 deg) - set to 0xFFFF (not available)
    Byte 7-8: Altitude (resolution: 0.125 m, offset: -2500 m) - set to 0xFFFF (not available)
    """
    # Compass Bearing: resolution 1/128 deg
    # raw = deg * 128
    bearing_raw = int(cog_deg * 128)
    bearing_raw = max(0, min(0xFFFF, bearing_raw))
    
    # Speed: resolution 1/256 km/h
    # Convert m/s to km/h first
    sog_kmh = sog_ms * 3.6
    speed_raw = int(sog_kmh * 256)
    speed_raw = max(0, min(0xFFFF, speed_raw))
    
    # Pitch and Altitude: not available
    pitch_raw = 0xFFFF
    altitude_raw = 0xFFFF
    
    return struct.pack("<HHHH", bearing_raw, speed_raw, pitch_raw, altitude_raw)


def parse_gprmc(nmea_line: str):
    """
    Parse GPRMC sentence
    Returns: (lat, lon, sog_knots, cog_deg) or (None, None, None, None)
    """
    try:
        parts = nmea_line.split(",")
        if not parts[0].endswith("RMC"):
            return None, None, None, None

        status = parts[2]
        if status != "A":
            return None, None, None, None

        lat_raw = parts[3]
        lat_dir = parts[4]
        lon_raw = parts[5]
        lon_dir = parts[6]
        sog_knots_raw = parts[7]
        cog_deg_raw = parts[8]

        if not lat_raw or not lon_raw:
            return None, None, None, None

        # Parse latitude
        lat_val = float(lat_raw)
        lat_deg = int(lat_val // 100)
        lat_min = lat_val - lat_deg * 100
        lat = lat_deg + lat_min / 60.0
        if lat_dir == "S":
            lat = -lat

        # Parse longitude
        lon_val = float(lon_raw)
        lon_deg = int(lon_val // 100)
        lon_min = lon_val - lon_deg * 100
        lon = lon_deg + lon_min / 60.0
        if lon_dir == "W":
            lon = -lon

        # Parse speed (knots) and course (degrees)
        sog_knots = float(sog_knots_raw) if sog_knots_raw else 0.0
        cog_deg = float(cog_deg_raw) if cog_deg_raw else 0.0

        return lat, lon, sog_knots, cog_deg

    except Exception:
        return None, None, None, None


def main():
    print(f"Opening serial {SERIAL_PORT} @ {SERIAL_BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

    print(f"Opening CAN bus on {CAN_INTERFACE}...")
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype="socketcan")

    print("NMEA0183 -> SAE J1939 bridge running...")

    # Build J1939 CAN IDs
    can_position = j1939_build_can_id(PGN_VEHICLE_POSITION, SOURCE_ADDRESS, J1939_PRIORITY)
    can_direction = j1939_build_can_id(PGN_VEHICLE_DIRECTION_SPEED, SOURCE_ADDRESS, J1939_PRIORITY)

    print(f"J1939 PGN 0x{PGN_VEHICLE_POSITION:04X} (Vehicle Position)")
    print(f"  CAN ID = 0x{can_position:08X} (Priority={J1939_PRIORITY}, SA=0x{SOURCE_ADDRESS:02X})")
    print(f"J1939 PGN 0x{PGN_VEHICLE_DIRECTION_SPEED:04X} (Vehicle Direction/Speed)")
    print(f"  CAN ID = 0x{can_direction:08X} (Priority={J1939_PRIORITY}, SA=0x{SOURCE_ADDRESS:02X})")
    print()

    while True:
        try:
            line_bytes = ser.readline()
            if not line_bytes:
                continue

            try:
                line = line_bytes.decode(errors="ignore").strip()
            except UnicodeDecodeError:
                continue

            if not line.startswith("$"):
                continue

            lat, lon, sog_knots, cog_deg = parse_gprmc(line)
            if lat is None:
                continue

            # Convert SOG from knots to m/s
            sog_ms = sog_knots * 0.514444

            # Prepare payloads
            payload_pos = encode_vehicle_position(lat, lon)
            payload_dir = encode_vehicle_direction_speed(cog_deg, sog_ms)

            # Send each message multiple times to achieve 5Hz
            # If GPS updates at ~1Hz, send 5x with 200ms delay
            REPEAT_COUNT = 5
            
            for i in range(REPEAT_COUNT):
                # Send Vehicle Position (PGN 0xFEF3)
                try:
                    bus.send(can.Message(
                        arbitration_id=can_position,
                        data=payload_pos,
                        is_extended_id=True,
                    ))
                except Exception as e:
                    print(f"ERROR sending PGN 0x{PGN_VEHICLE_POSITION:04X}:", e)
                    break

                # Send Vehicle Direction/Speed (PGN 0xFEE8)
                try:
                    bus.send(can.Message(
                        arbitration_id=can_direction,
                        data=payload_dir,
                        is_extended_id=True,
                    ))
                except Exception as e:
                    print(f"ERROR sending PGN 0x{PGN_VEHICLE_DIRECTION_SPEED:04X}:", e)
                    break

                # Delay between repeats (~200ms for 5Hz)
                time.sleep(0.2)

            print(f">>> SENT {REPEAT_COUNT}x: Pos=({lat:.6f}, {lon:.6f}) COG={cog_deg:.1f}° SOG={sog_knots:.2f}kn ({sog_ms*3.6:.2f}km/h)")

        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error:", e)
            time.sleep(0.1)


if __name__ == "__main__":
    main()
