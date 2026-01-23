#!/usr/bin/env python3
import serial
import time
from datetime import datetime, timezone
import math

# ---------------- CONFIG YOU PROBABLY WANT TO EDIT ----------------

# Serial device and baud rate used on ceol to talk to the Pi
# Change this if your serial port is different
SERIAL_PORT = "/dev/ttyUSB1"
SERIAL_BAUD = 9600

# Base "true" position in degrees (LLH)
# Change these to your test location
BASE_LAT_DEG = 51.987900    # degrees North
BASE_LON_DEG = 5.663000     # degrees East
BASE_ALT_M   = 10.0         # meters above mean sea level

# Base velocity and heading for RMC
# Change SOG (knots) and COG (degrees) as you like
BASE_SOG_KNOTS = 1.2        # speed over ground in knots
BASE_COG_DEG   = 45.0       # course over ground in degrees

# Fix quality (GGA)
# 0 = invalid, 1 = GPS fix, 2 = DGPS, 4 = RTK fixed, 5 = RTK float, etc.
GGA_FIX_QUALITY = 4         # use 4 to emulate RTK fixed

# Number of satellites used in solution
GGA_NUM_SATS = 10

# Horizontal dilution of precision (HDOP)
GGA_HDOP = 0.8

# GST 1-sigma LLH errors in meters
GST_SIGMA_LAT_M = 0.02      # 2 cm
GST_SIGMA_LON_M = 0.02
GST_SIGMA_ALT_M = 0.05

# GNS mode indicators per constellation (GPS, GLONASS, Galileo, BeiDou, QZSS)
# Valid modes: N,A,D,P,R,F (see spec text)
GNS_MODE_GPS      = "R"     # e.g. RTK fixed
GNS_MODE_GLONASS  = "N"
GNS_MODE_GALILEO  = "R"
GNS_MODE_BEIDOU   = "N"
GNS_MODE_QZSS     = "N"

# PHTG authentication message defaults
# TimeTag format choice: "week,seconds" or "dd:mm:yyyy,hh:mm:ss.ss"
# This implementation uses the date/time format for readability
PHTG_SYSTEM  = "GAL"        # GPS or GAL
PHTG_SERVICE = "HAS"        # HAS or OSNMA or HO
PHTG_AUTH_RESULT = 0        # 0..4 (see spec)
PHTG_STATUS      = 0        # 0..3 (see spec)
PHTG_WARNING     = 0        # 0..3 (see spec)

# Talker IDs used:
# GGA/GST/GSV/RMC use GP (GPS receiver)
# GNS uses GN as per your spec text (combined GNSS)
TALKER_GPS = "GP"
TALKER_GN  = "GN"

# -----------------------------------------------------------------


def nmea_checksum(body: str) -> str:
    """
    Compute NMEA-0183 checksum (two hex digits) for the string between '$' and '*'.
    """
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def format_lat_nmea(lat_deg: float):
    """
    Convert latitude in degrees to NMEA ddmm.mmmmmmmm format + N/S.
    Uses 8 decimal digits in minutes for mm-level resolution.
    """
    hemi = "N" if lat_deg >= 0 else "S"
    lat_abs = abs(lat_deg)
    deg = int(lat_abs)
    minutes = (lat_abs - deg) * 60.0
    # 2 digits degrees, 2 digits minutes + 8 decimal places
    return f"{deg:02d}{minutes:011.8f}", hemi


def format_lon_nmea(lon_deg: float):
    """
    Convert longitude in degrees to NMEA dddmm.mmmmmmmm format + E/W.
    """
    hemi = "E" if lon_deg >= 0 else "W"
    lon_abs = abs(lon_deg)
    deg = int(lon_abs)
    minutes = (lon_abs - deg) * 60.0
    # 3 digits degrees, 2 digits minutes + 8 decimal places
    return f"{deg:03d}{minutes:011.8f}", hemi


def now_utc():
    """
    Return (time_str, date_str) for NMEA:
    time_str: hhmmss.sss
    date_str: ddmmyy
    """
    now = datetime.now(timezone.utc)
    time_str = now.strftime("%H%M%S") + f".{int(now.microsecond/1000):03d}"
    date_str = now.strftime("%d%m%y")
    return time_str, date_str, now


def build_gga(lat_deg: float, lon_deg: float, alt_m: float) -> str:
    """
    Build a GGA sentence (Global Positioning System Fix Data) with mm-level LLH resolution.
    """
    t_str, _, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)

    fix_q = GGA_FIX_QUALITY
    num_sats = GGA_NUM_SATS
    hdop = GGA_HDOP
    geoid_sep = 0.0  # you can change this if you care about geoid vs ellipsoid

    body = (
        f"{TALKER_GPS}GGA,"
        f"{t_str},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{fix_q},"
        f"{num_sats:02d},"
        f"{hdop:.1f},"
        f"{alt_m:.2f},M,"
        f"{geoid_sep:.2f},M,,"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_rmc(lat_deg: float, lon_deg: float, sog_knots: float, cog_deg: float) -> str:
    """
    Build an RMC sentence (Recommended Minimum Specific GNSS Data) including SOG and COG.
    """
    t_str, d_str, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)

    status = "A"  # A = valid, V = void
    mag_var = ""  # empty if unknown
    mag_hemi = ""

    body = (
        f"{TALKER_GPS}RMC,"
        f"{t_str},"
        f"{status},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{sog_knots:.1f},"
        f"{cog_deg:.1f},"
        f"{d_str},"
        f"{mag_var},{mag_hemi}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gns(lat_deg: float, lon_deg: float, alt_m: float) -> str:
    """
    Build a GNS sentence (GNSS fix data) with GN talker ID and mode identifiers.
    Formats: $GNGNS,time,lat,NS,lon,EW,modeGPS,modeGLONASS,modeGalileo,modeBeiDou,modeQZSS,HDOP,alt,sep,...*
    This is simplified but structurally correct.
    """
    t_str, _, _ = now_utc()
    lat_str, lat_hemi = format_lat_nmea(lat_deg)
    lon_str, lon_hemi = format_lon_nmea(lon_deg)

    # Mode string per constellation (see config)
    mode_str = (
        f"{GNS_MODE_GPS}{GNS_MODE_GLONASS}"
        f"{GNS_MODE_GALILEO}{GNS_MODE_BEIDOU}"
        f"{GNS_MODE_QZSS}"
    )

    hdop = GGA_HDOP
    geoid_sep = 0.0
    # number of SVs, DOP, etc. left simple / empty for now

    body = (
        f"{TALKER_GN}GNS,"
        f"{t_str},"
        f"{lat_str},{lat_hemi},"
        f"{lon_str},{lon_hemi},"
        f"{mode_str},"
        f"{hdop:.1f},"
        f"{alt_m:.2f},"
        f"{geoid_sep:.2f},,,"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gst(lat_deg: float, lon_deg: float) -> str:
    """
    Build a GST sentence (Position error statistics).
    Uses configurable 1-sigma LLH errors in meters.
    """
    t_str, _, _ = now_utc()
    # For simplicity, treat sigma_lat/lon as in meters directly
    # NMEA GST expects RMS, sigma_major, sigma_minor, orientation, sigma_lat, sigma_lon, sigma_alt
    sigma_lat = GST_SIGMA_LAT_M
    sigma_lon = GST_SIGMA_LON_M
    sigma_alt = GST_SIGMA_ALT_M
    rms = math.sqrt((sigma_lat**2 + sigma_lon**2) / 2.0)
    sigma_major = sigma_lat
    sigma_minor = sigma_lon
    orientation = 0.0

    body = (
        f"{TALKER_GPS}GST,"
        f"{t_str},"
        f"{rms:.3f},"
        f"{sigma_major:.3f},"
        f"{sigma_minor:.3f},"
        f"{orientation:.3f},"
        f"{sigma_lat:.3f},"
        f"{sigma_lon:.3f},"
        f"{sigma_alt:.3f}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_gsv() -> str:
    """
    Build a very simple GSV sentence (GNSS Satellites in View).
    This is a placeholder; edit to match your satellite layout if needed.
    """
    num_sats_view = max(GGA_NUM_SATS, 1)
    num_msgs = 1
    msg_idx = 1

    # Example single satellite with dummy values
    sat_id = 11
    elevation = 45  # degrees
    azimuth = 120   # degrees
    snr = 40        # dB-Hz

    body = (
        f"{TALKER_GPS}GSV,"
        f"{num_msgs},"
        f"{msg_idx},"
        f"{num_sats_view},"
        f"{sat_id},{elevation},{azimuth},{snr}"
    )
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def build_phtg(now: datetime) -> str:
    """
    Build the proprietary authentication NMEA message for HASHTAG:
    $PHTG,TimeTag,System,Service,Status,Warning*hh

    TimeTag here uses "dd:mm:yyyy,hh:mm:ss.ss" format for readability.
    Change format if you prefer GPS week+seconds.
    """
    # TimeTag in dd:mm:yyyy,hh:mm:ss.ss format
    # Change this block if you want GPS week + seconds-of-week instead
    date_part = now.strftime("%d:%m:%Y")
    time_part = now.strftime("%H:%M:%S") + ".00"
    timetag = f"{date_part},{time_part}"

    system = PHTG_SYSTEM
    service = PHTG_SERVICE
    status = PHTG_AUTH_RESULT
    warning = PHTG_WARNING

    body = f"PHTG,{timetag},{system},{service},{status},{warning}"
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"


def main():
    print(f"Opening serial {SERIAL_PORT} @ {SERIAL_BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

    print("Sending PVT NMEA (GGA, RMC, GNS, GST, GSV, PHTG) at 1 Hz. CTRL+C to stop.")
    while True:
        try:
            t_str, d_str, now = now_utc()  # we reuse 'now' for PHTG

            lat = BASE_LAT_DEG
            lon = BASE_LON_DEG
            alt = BASE_ALT_M
            sog = BASE_SOG_KNOTS
            cog = BASE_COG_DEG

            sentences = [
                build_gga(lat, lon, alt),
                build_rmc(lat, lon, sog, cog),
                build_gns(lat, lon, alt),
                build_gst(lat, lon),
                build_gsv(),
                build_phtg(now),
            ]

            for s in sentences:
                # If you only want to send a subset, comment/remove items in `sentences` above
                ser.write(s.encode("ascii"))
                print("TX:", s.strip())

            time.sleep(1.0)

        except KeyboardInterrupt:
            print("Stopping NMEA output.")
            break
        except Exception as e:
            print("Error:", e)
            time.sleep(0.2)


if __name__ == "__main__":
    main()


