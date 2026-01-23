import serial
import can
import time
import struct
import math
from datetime import datetime, timezone
from haversine import haversine, Unit   # For COG/SOG calculation

# -----------------------------
# CONFIGURATION
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

CAN_INTERFACE = "can0"

# NMEA 2000 PGNs
PGN_POSITION_RAPID = 129025       # 0x1F801 - Position, Rapid Update
PGN_COG_SOG_RAPID = 129026        # 0x1F802 - COG & SOG, Rapid Update
PGN_GNSS_POSITION = 129029        # 0x1F805 - GNSS Position Data

NMEA2000_PRIORITY = 3  # Standard priority for navigation data
SOURCE_ADDRESS = 0x80  # Source address that CCI recognizes
# -----------------------------

def nmea2000_build_can_id(pgn: int, sa: int, priority: int = 6) -> int:
    """Build CAN ID for NMEA 2000 message"""
    pf = (pgn >> 8) & 0xFF
    ps = pgn & 0xFF
    dp = (pgn >> 16) & 0x1

    # PDU1 format (destination specific): PF < 240
    # PDU2 format (broadcast): PF >= 240
    if pf < 240:
        # PDU1: PS is destination address
        da = ps
    else:
        # PDU2: PS is group extension
        da = 0xFF

    # 29-bit CAN ID format:
    # Priority (3) | Reserved (1) | DP (1) | PF (8) | PS (8) | SA (8)
    return ((priority & 0x7) << 26) | ((dp & 0x1) << 24) | (pf << 16) | (ps << 8) | sa




# ================================================================
# NMEA 2000 PGN 129025 (0x1F801) — Position, Rapid Update
# ================================================================
def encode_position_rapid(lat_deg: float, lon_deg: float) -> bytes:
    """
    NMEA 2000 Position Rapid Update (PGN 129025)
    Byte 1-4: Latitude (1e-7 degrees, signed int32)
    Byte 5-8: Longitude (1e-7 degrees, signed int32)
    """
    lat_i = int(lat_deg * 1e7)
    lon_i = int(lon_deg * 1e7)
    return struct.pack("<ii", lat_i, lon_i)


# ================================================================
# Parse RMC - Extract lat, lon, speed, and course
# ================================================================
def parse_gprmc(nmea_line: str):
    """
    Parse GPRMC sentence
    Returns: (lat, lon, sog_knots, cog_deg) or (None, None, None, None)

    GPRMC format:
    $GPRMC,hhmmss.ss,A,ddmm.mmmm,N/S,dddmm.mmmm,E/W,speed_knots,course_deg,ddmmyy,...
           [1]      [2] [3]      [4] [5]        [6] [7]         [8]
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
        sog_knots_raw = parts[7]  # Speed over ground in knots
        cog_deg_raw = parts[8]     # Course over ground in degrees

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


# ================================================================
# ADDED FOR ISOBUS — COG/SOG COMPUTATION + ENCODERS
# ================================================================

last_lat = None
last_lon = None
last_t = None


def compute_cog_sog(lat, lon):
    global last_lat, last_lon, last_t

    now = time.time()
    if last_lat is None:
        last_lat = lat
        last_lon = lon
        last_t = now
        return 0.0, 0.0

    dt = now - last_t
    if dt <= 0:
        dt = 0.1

    dist_m = haversine((last_lat, last_lon), (lat, lon), unit=Unit.METERS)
    sog = dist_m / dt

    cog = compute_bearing(last_lat, last_lon, lat, lon)

    last_lat = lat
    last_lon = lon
    last_t = now

    return sog, cog


def compute_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ================================================================
# NMEA 2000 PGN 129026 (0x1F802) — COG & SOG, Rapid Update
# ================================================================
def encode_cog_sog_rapid(cog_deg, sog_ms, sid=0):
    """
    NMEA 2000 COG & SOG Rapid Update (PGN 129026)
    Based on ttlappalainen/NMEA2000 library:
    Byte 1: SID (Sequence ID)
    Byte 2: COG Reference (2 bits) + Reserved (6 bits set to 1)
    Byte 3-4: COG (0.0001 radians, uint16)
    Byte 5-6: SOG (0.01 m/s, uint16)
    Byte 7-8: Reserved (0xFF)
    """
    # COG Reference: 0=True, 1=Magnetic
    cog_reference = 0  # True North
    ref_byte = (cog_reference & 0x03) | 0xFC  # Reserved bits 2-7 set to 1

    # Convert degrees to radians, then scale (0.0001 radians resolution)
    cog_rad = math.radians(cog_deg)
    cog_scaled = int(cog_rad / 0.0001)  # Same as * 10000

    # Speed in 0.01 m/s units
    sog_scaled = int(sog_ms / 0.01)  # Same as * 100

    # Clamp to valid uint16 range (0-65535)
    cog_scaled = max(0, min(65535, cog_scaled))
    sog_scaled = max(0, min(65535, sog_scaled))

    return struct.pack("<BBHHBB", sid, ref_byte, cog_scaled, sog_scaled, 0xFF, 0xFF)

# ================================================================
# NMEA 2000 Fast Packet Protocol
# ================================================================
def send_fast_packet(bus, can_id: int, data: bytes, sequence_counter: int = 0):
    """
    Send multi-frame NMEA 2000 Fast Packet message
    Frame 0: [seq_counter, total_len, data[0:6]]
    Frame N: [seq_counter | (frame_num << 5), data[6+(N-1)*7 : 6+N*7]]
    """
    total_len = len(data)
    frame_counter = sequence_counter & 0x1F  # 5-bit counter

    # First frame
    frame0 = bytes([frame_counter, total_len]) + data[0:6]
    bus.send(can.Message(arbitration_id=can_id, data=frame0, is_extended_id=True))
    time.sleep(0.001)  # Small delay between frames

    # Remaining frames (7 bytes of data each)
    offset = 6
    frame_num = 1
    while offset < total_len:
        header = frame_counter | (frame_num << 5)
        chunk = data[offset:offset+7]
        # Pad to 8 bytes if needed
        frame_data = bytes([header]) + chunk + bytes([0xFF] * (7 - len(chunk)))
        bus.send(can.Message(arbitration_id=can_id, data=frame_data, is_extended_id=True))
        time.sleep(0.001)  # Small delay between frames
        offset += 7
        frame_num += 1


# ================================================================
# NMEA 2000 PGN 129029 (0x1F805) — GNSS Position Data
# ================================================================
def encode_gnss_position(lat_deg: float, lon_deg: float, altitude_m: float = 0.0) -> bytes:
    """
    NMEA 2000 GNSS Position Data (PGN 129029)
    Byte 1: SID (Sequence ID)
    Byte 2-3: Date (days since Jan 1, 1970, uint16)
    Byte 4-7: Time (seconds since midnight, uint32, 0.0001s resolution)
    Byte 8-15: Latitude (1e-16 degrees, int64)
    Byte 16-23: Longitude (1e-16 degrees, int64)
    Byte 24-31: Altitude (1e-6 meters, int64)
    Byte 32: GNSS type
    Byte 33: Method
    Byte 34: Integrity
    Byte 35: Number of SVs
    Byte 36-37: HDOP (0.01, uint16)
    Byte 38-39: PDOP (0.01, uint16)
    Byte 40-43: Geoidal Separation (0.01m, int32)
    Byte 44: Reference Stations
    ... (additional fields)
    """
    now = datetime.now(timezone.utc)

    sid = 0  # Use real sequence ID (increment each call if needed)

    # Date: days since Jan 1, 1970
    epoch_1970 = datetime(1970, 1, 1, tzinfo=timezone.utc)
    days_since_1970 = (now - epoch_1970).days
    days_since_1970 = max(0, min(65535, days_since_1970))

    # Time: seconds since midnight (0.0001s resolution)
    seconds_since_midnight = (now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 1e6)
    time_scaled = int(seconds_since_midnight * 10000)

    # Latitude/Longitude as int64 with 1e-16 resolution (per official NMEA2000 library)
    lat_scaled = int(lat_deg * 1e16)
    lon_scaled = int(lon_deg * 1e16)

    # Altitude in 1e-6 meters (micrometers)
    alt_scaled = int(altitude_m * 1e6)

    # GNSS fields (bit fields need to be packed together)
    gnss_type = 0  # GPS (4 bits)
    method = 1  # GNSS fix (4 bits: 0=no fix, 1=GNSS fix, 2=DGNSS fix)
    integrity = 1  # Safe (2 bits: 0=No check, 1=Safe, 2=Caution)
    reserved = 0x3F  # 6 reserved bits set to 1

    # Pack bit fields into 2 bytes: [gnss_type(4) | method(4)] [integrity(2) | reserved(6)]
    byte_gnss_method = (gnss_type & 0x0F) | ((method & 0x0F) << 4)
    byte_integrity_reserved = (integrity & 0x03) | ((reserved & 0x3F) << 2)

    num_svs = 8  # Number of satellites
    hdop = 100  # HDOP * 100 (1.0)
    pdop = 200  # PDOP * 100 (2.0)
    geoidal_sep = 0  # Geoidal separation in 0.01m
    ref_stations = 1  # Try with 1 reference station

    # Pack: SID(1) + Date(2) + Time(4) + Lat(8) + Lon(8) + Alt(8) + 
    #       GNSS/Method(1) + Integrity/Reserved(1) + NumSVs(1) + HDOP(2) + PDOP(2) + Geoidal(4) + RefStations(1)
    # Total: 43 bytes minimum

    payload = struct.pack(
        "<BHIqqq BBB hh iB",
        sid, days_since_1970, time_scaled,
        lat_scaled, lon_scaled, alt_scaled,
        byte_gnss_method, byte_integrity_reserved, num_svs,
        hdop, pdop,
        geoidal_sep, ref_stations
    )

    # Add reference station data (Type + ID + Age)
    # Type(4 bits)=0xF (Null), ID(12 bits)=0xFFF, packed into 2 bytes
    ref_type_id = 0xFFFF  # All bits set = Null station
    ref_age = 0xFFFF  # Age = unavailable
    payload += struct.pack("<HH", ref_type_id, ref_age)

    return payload


# ================================================================
# MAIN LOOP — NMEA 2000 GPS BRIDGE
# ================================================================
def main():
    print(f"Opening serial {SERIAL_PORT} @ {SERIAL_BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

    print(f"Opening CAN bus on {CAN_INTERFACE}...")
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype="socketcan")

    print("NMEA0183 -> NMEA 2000 bridge running...")

    # Build NMEA 2000 CAN IDs with correct priorities per NMEA2000 library
    # PGN 129025 and 129026: Priority 2
    # PGN 129029: Priority 3
    can_pos_rapid = nmea2000_build_can_id(PGN_POSITION_RAPID, SOURCE_ADDRESS, 2)
    can_cog_sog = nmea2000_build_can_id(PGN_COG_SOG_RAPID, SOURCE_ADDRESS, 2)
    can_gnss_pos = nmea2000_build_can_id(PGN_GNSS_POSITION, SOURCE_ADDRESS, 3)

    print(f"NMEA 2000 PGN {PGN_POSITION_RAPID} (0x{PGN_POSITION_RAPID:X}) - Position Rapid")
    print(f"  CAN ID = {hex(can_pos_rapid)} (Priority=2, SA=0x{SOURCE_ADDRESS:02X})")
    print(f"NMEA 2000 PGN {PGN_COG_SOG_RAPID} (0x{PGN_COG_SOG_RAPID:X}) - COG/SOG Rapid")
    print(f"  CAN ID = {hex(can_cog_sog)} (Priority=2, SA=0x{SOURCE_ADDRESS:02X})")
    print(f"NMEA 2000 PGN {PGN_GNSS_POSITION} (0x{PGN_GNSS_POSITION:X}) - GNSS Position")
    print(f"  CAN ID = {hex(can_gnss_pos)} (Priority=3, SA=0x{SOURCE_ADDRESS:02X})")
    print(f"\nNote: Configure CCI to listen for Source Address 0x{SOURCE_ADDRESS:02X}")

    # Fast Packet sequence counter and message counter
    fast_packet_seq = 0
    msg_count = 0
    sid = 0  # Sequence ID for linking related PGNs

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

            msg_count += 1

            #
            # ---- Convert SOG from knots to m/s ----
            #
            sog_ms = sog_knots * 0.514444  # 1 knot = 0.514444 m/s
            cog = cog_deg

            # Prepare all payloads with same SID to link them together
            payload_pos = encode_position_rapid(lat, lon)
            payload_cog_sog = encode_cog_sog_rapid(cog, sog_ms, sid)
            payload_gnss = encode_gnss_position(lat, lon)

            # Increment SID for next set of messages (0-252)
            sid = (sid + 1) % 253

            # Send each message multiple times to meet 5Hz requirement
            # If GPS is 0.5Hz, send 10x; if 1Hz, send 5x, etc.
            REPEAT_COUNT = 10  # Adjust based on your GPS update rate

            for i in range(REPEAT_COUNT):
                #
                # ---- PGN 129025: Position Rapid Update (at least 5Hz) ----
                #
                try:
                    bus.send(can.Message(
                        arbitration_id=can_pos_rapid,
                        data=payload_pos,
                        is_extended_id=True,
                    ))
                except Exception as e:
                    print(f"ERROR during PGN {PGN_POSITION_RAPID}:", e)
                    break

                #
                # ---- PGN 129026: COG & SOG Rapid Update (at least 5Hz) ----
                #
                try:
                    bus.send(can.Message(
                        arbitration_id=can_cog_sog,
                        data=payload_cog_sog,
                        is_extended_id=True,
                    ))
                except Exception as e:
                    print(f"ERROR during PGN {PGN_COG_SOG_RAPID}:", e)
                    break

                # Small delay between repeats (~100ms for 10Hz rate)
                time.sleep(0.1)

            print(f">>> SENT {REPEAT_COUNT}x PGN {PGN_POSITION_RAPID} (Position Rapid) + {PGN_COG_SOG_RAPID} (COG={cog:.1f}° SOG={sog_knots:.2f}kn)")

            #
            # ---- PGN 129029: GNSS Position Data (CCI shows this as "POS") ----
            #
            # Send once per GPS update (1Hz)
            print(f">>> SENDING PGN {PGN_GNSS_POSITION} (GNSS Position/POS) via Fast Packet")
            try:
                send_fast_packet(bus, can_gnss_pos, payload_gnss, fast_packet_seq)
                fast_packet_seq = (fast_packet_seq + 1) & 0x1F
            except Exception as e:
                print(f"ERROR during PGN {PGN_GNSS_POSITION}:", e)

        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error:", e)
            time.sleep(0.1)



if __name__ == "__main__":
    main()

