#!/usr/bin/env python3
"""Estimate battery needs for a 60 km multicopter round trip.

This is a first-order sizing tool for the 30 km outbound + 30 km return
scenario used by run_60km_roundtrip_copter_sim.py. It is not a substitute for
motor test-stand data, propeller curves, or flight logs.
"""

from __future__ import annotations

from dataclasses import dataclass


ROUTE_DISTANCE_M = 60_000.0
LEG_DISTANCE_M = 30_000.0
CRUISE_AIRSPEED_MPS = 15.0
BATTERY_USABLE_FRACTION = 0.75
LI_ION_PACK_WH_PER_KG = 200.0


@dataclass(frozen=True)
class AirframeCase:
    name: str
    dry_mass_kg: float
    payload_kg: float
    battery_wh: float
    propulsion_power_w: float
    avionics_w: float = 8.0
    telemetry_w: float = 8.0
    video_w: float = 15.0
    ai_w: float = 25.0

    @property
    def battery_mass_kg(self) -> float:
        return self.battery_wh / LI_ION_PACK_WH_PER_KG

    @property
    def takeoff_mass_kg(self) -> float:
        return self.dry_mass_kg + self.payload_kg + self.battery_mass_kg

    @property
    def accessory_power_w(self) -> float:
        return self.avionics_w + self.telemetry_w + self.video_w + self.ai_w

    @property
    def total_power_w(self) -> float:
        return self.propulsion_power_w + self.accessory_power_w

    @property
    def usable_battery_wh(self) -> float:
        return self.battery_wh * BATTERY_USABLE_FRACTION


@dataclass(frozen=True)
class WindCase:
    name: str
    outbound_along_track_mps: float
    return_along_track_mps: float


def leg_time_s(airspeed_mps: float, along_track_wind_mps: float) -> float:
    ground_speed_mps = airspeed_mps + along_track_wind_mps
    if ground_speed_mps <= 1.0:
        raise ValueError("ground speed is too low for this estimate")
    return LEG_DISTANCE_M / ground_speed_mps


def estimate_energy_wh(case: AirframeCase, wind: WindCase) -> tuple[float, float]:
    outbound_s = leg_time_s(CRUISE_AIRSPEED_MPS, wind.outbound_along_track_mps)
    return_s = leg_time_s(CRUISE_AIRSPEED_MPS, wind.return_along_track_mps)
    hours = (outbound_s + return_s) / 3600.0
    return hours, case.total_power_w * hours


def status(required_wh: float, case: AirframeCase) -> str:
    margin_wh = case.usable_battery_wh - required_wh
    margin_pct = margin_wh / case.usable_battery_wh * 100.0
    if margin_pct >= 20.0:
        return "PASS"
    if margin_pct >= 0.0:
        return "TIGHT"
    return "FAIL"


def print_table(cases: list[AirframeCase], winds: list[WindCase]) -> None:
    print("60 km roundtrip energy estimate")
    print(f"route={ROUTE_DISTANCE_M/1000:.0f}km airspeed={CRUISE_AIRSPEED_MPS:.1f}m/s usable_battery={BATTERY_USABLE_FRACTION:.0%}")
    print()
    header = (
        "case",
        "wind",
        "mass_kg",
        "battery_wh",
        "usable_wh",
        "power_w",
        "time_min",
        "need_wh",
        "margin_wh",
        "status",
    )
    print(",".join(header))
    for case in cases:
        for wind in winds:
            hours, need_wh = estimate_energy_wh(case, wind)
            margin_wh = case.usable_battery_wh - need_wh
            row = (
                case.name,
                wind.name,
                f"{case.takeoff_mass_kg:.1f}",
                f"{case.battery_wh:.0f}",
                f"{case.usable_battery_wh:.0f}",
                f"{case.total_power_w:.0f}",
                f"{hours * 60.0:.1f}",
                f"{need_wh:.0f}",
                f"{margin_wh:.0f}",
                status(need_wh, case),
            )
            print(",".join(row))


def main() -> None:
    cases = [
        AirframeCase(
            name="multicopter_10kg_payload_2kwh",
            dry_mass_kg=12.0,
            payload_kg=10.0,
            battery_wh=2_000.0,
            propulsion_power_w=1_800.0,
        ),
        AirframeCase(
            name="multicopter_10kg_payload_3kwh",
            dry_mass_kg=13.0,
            payload_kg=10.0,
            battery_wh=3_000.0,
            propulsion_power_w=2_200.0,
        ),
        AirframeCase(
            name="efficient_vtol_reference_2kwh",
            dry_mass_kg=10.0,
            payload_kg=10.0,
            battery_wh=2_000.0,
            propulsion_power_w=650.0,
        ),
    ]
    winds = [
        WindCase("calm", 0.0, 0.0),
        WindCase("5mps_head_then_tail", -5.0, 5.0),
        WindCase("5mps_tail_then_head", 5.0, -5.0),
        WindCase("10mps_head_then_tail", -10.0, 10.0),
        WindCase("10mps_tail_then_head", 10.0, -10.0),
    ]
    print_table(cases, winds)


if __name__ == "__main__":
    main()
