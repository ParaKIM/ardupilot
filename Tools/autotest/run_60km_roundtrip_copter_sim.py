#!/usr/bin/env python3
import math
import time

from pymavlink import mavutil


EARTH_RADIUS_M = 6378137.0


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    lat = math.radians(lat_deg)
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(lat))
    return math.degrees(lat + dlat), lon_deg + math.degrees(dlon)


def distance_m(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def connect():
    master = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        raise RuntimeError("heartbeat timeout")
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent() or 1
    print(f"heartbeat system={master.target_system} component={master.target_component}", flush=True)
    return master


def request_streams(master):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        100000,
        0,
        0,
        0,
        0,
        0,
    )


def set_param(master, name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.strip("\x00") == name:
            print(f"param {name}={msg.param_value}", flush=True)
            return
    print(f"param {name} set requested", flush=True)


def set_mode(master, mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.set_mode(mode_id)
    deadline = time.time() + 20
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            print(f"mode {mode_name}", flush=True)
            return
    raise RuntimeError(f"mode change to {mode_name} timeout")


def command_long(master, command, params):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *params,
    )
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if msg and msg.command == command:
            print(f"ack {command} result={msg.result}", flush=True)
            return msg.result
    raise RuntimeError(f"command {command} ack timeout")


def get_position(master):
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    raise RuntimeError("GLOBAL_POSITION_INT timeout")


def send_guided_target(master, lat, lon, alt_m):
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def fly_leg(master, leg_name, start_lat, start_lon, target_lat, target_lon, alt_m, base_total_m):
    print(f"leg {leg_name} target lat={target_lat:.7f} lon={target_lon:.7f}", flush=True)
    last_bucket = int(base_total_m // 5000)
    last_send = 0.0
    while True:
        now = time.time()
        if now - last_send > 1.0:
            send_guided_target(master, target_lat, target_lon, alt_m)
            last_send = now

        lat, lon, alt = get_position(master)
        leg_flown = distance_m(start_lat, start_lon, lat, lon)
        remaining = distance_m(lat, lon, target_lat, target_lon)
        total = base_total_m + min(leg_flown, 30000.0)
        bucket = int(total // 5000)
        if bucket != last_bucket:
            last_bucket = bucket
            print(
                f"progress total={total/1000:.2f}km leg={leg_name} remaining={remaining/1000:.2f}km alt={alt:.1f}m",
                flush=True,
            )
        if remaining < 100:
            total = base_total_m + 30000.0
            print(
                f"leg_complete {leg_name} total={total/1000:.2f}km remaining={remaining:.1f}m alt={alt:.1f}m",
                flush=True,
            )
            return total


def main():
    master = connect()
    request_streams(master)

    # This is a navigation simulation. It intentionally bypasses hardware pre-arm checks.
    set_param(master, "ARMING_CHECK", 0)
    set_param(master, "WPNAV_SPEED", 1500)
    set_param(master, "WPNAV_SPEED_UP", 500)
    set_param(master, "WPNAV_SPEED_DN", 300)
    set_param(master, "RTL_ALT", 10000)

    set_mode(master, "GUIDED")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("armed", flush=True)

    alt_m = 150.0
    result = command_long(
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        [0, 0, 0, 0, 0, 0, alt_m],
    )
    if result not in (0, 4):
        raise RuntimeError(f"takeoff rejected: {result}")

    start_lat = start_lon = None
    while True:
        lat, lon, alt = get_position(master)
        if start_lat is None:
            start_lat, start_lon = lat, lon
            print(f"home lat={start_lat:.7f} lon={start_lon:.7f}", flush=True)
        if alt >= alt_m * 0.9:
            print(f"takeoff alt={alt:.1f}m", flush=True)
            break

    far_lat, far_lon = offset_latlon(start_lat, start_lon, 30000.0, 0.0)
    print(f"route outbound=30.00km return=30.00km total=60.00km altitude_relative_home={alt_m:.0f}m", flush=True)

    command_long(
        master,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        [1, 15, -1, 0, 0, 0, 0],
    )

    started = time.time()
    total = fly_leg(master, "outbound", start_lat, start_lon, far_lat, far_lon, alt_m, 0.0)
    total = fly_leg(master, "return", far_lat, far_lon, start_lat, start_lon, alt_m, total)
    elapsed = time.time() - started
    print(f"roundtrip_complete total={total/1000:.2f}km wall_time={elapsed:.1f}s", flush=True)


if __name__ == "__main__":
    main()
