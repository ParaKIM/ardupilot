#!/usr/bin/env python3
"""Monte Carlo energy estimate for a 60 km VTOL round trip.

Scenario:
- 30 km outbound + 30 km return
- 10 kg payload
- AGL 150 m mission altitude
- VTOL takeoff/landing energy plus fixed-wing cruise energy

This is a sizing tool. It intentionally uses broad distributions because real
results depend on the selected airframe, propellers, battery cells, payload
drag, and flight logs.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from statistics import mean


LEG_DISTANCE_M = 30_000.0
TAKEOFF_LAND_SECONDS = 240.0
TRANSITION_SECONDS = 60.0
MIN_GROUND_SPEED_MPS = 8.0


@dataclass(frozen=True)
class BatteryCase:
    name: str
    battery_wh: float


@dataclass(frozen=True)
class Trial:
    energy_wh: float
    flight_min: float
    wind_mps: float
    wind_component_mps: float
    airspeed_mps: float
    cruise_power_w: float
    hover_power_w: float
    accessory_power_w: float
    usable_fraction: float


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def triangular(rng: random.Random, low: float, mode: float, high: float) -> float:
    return rng.triangular(low, high, mode)


def sample_trial(rng: random.Random) -> Trial | None:
    # Wind speed distribution: mostly moderate, sometimes strong.
    wind_mps = min(rng.weibullvariate(5.5, 1.7), 16.0)
    wind_angle_rad = rng.uniform(0.0, math.tau)
    wind_component_mps = wind_mps * math.cos(wind_angle_rad)

    airspeed_mps = triangular(rng, 22.0, 25.0, 29.0)
    outbound_ground_mps = airspeed_mps + wind_component_mps
    return_ground_mps = airspeed_mps - wind_component_mps
    if outbound_ground_mps < MIN_GROUND_SPEED_MPS or return_ground_mps < MIN_GROUND_SPEED_MPS:
        return None

    cruise_power_w = triangular(rng, 520.0, 700.0, 980.0)
    hover_power_w = triangular(rng, 2_600.0, 3_500.0, 5_000.0)
    accessory_power_w = triangular(rng, 35.0, 58.0, 95.0)
    usable_fraction = triangular(rng, 0.65, 0.75, 0.84)

    cruise_seconds = LEG_DISTANCE_M / outbound_ground_mps + LEG_DISTANCE_M / return_ground_mps
    total_seconds = cruise_seconds + TAKEOFF_LAND_SECONDS + TRANSITION_SECONDS
    energy_wh = (
        cruise_power_w * cruise_seconds
        + hover_power_w * TAKEOFF_LAND_SECONDS
        + accessory_power_w * total_seconds
    ) / 3600.0

    return Trial(
        energy_wh=energy_wh,
        flight_min=total_seconds / 60.0,
        wind_mps=wind_mps,
        wind_component_mps=wind_component_mps,
        airspeed_mps=airspeed_mps,
        cruise_power_w=cruise_power_w,
        hover_power_w=hover_power_w,
        accessory_power_w=accessory_power_w,
        usable_fraction=usable_fraction,
    )


def run(samples: int, seed: int) -> None:
    rng = random.Random(seed)
    batteries = [
        BatteryCase("2.0kWh", 2_000.0),
        BatteryCase("2.5kWh", 2_500.0),
        BatteryCase("3.0kWh", 3_000.0),
    ]

    trials: list[Trial] = []
    rejected = 0
    for _ in range(samples):
        trial = sample_trial(rng)
        if trial is None:
            rejected += 1
            continue
        trials.append(trial)

    energies = [trial.energy_wh for trial in trials]
    flight_times = [trial.flight_min for trial in trials]
    print("VTOL 60km Monte Carlo energy estimate")
    print(f"samples={samples} accepted={len(trials)} rejected_low_groundspeed={rejected} seed={seed}")
    print("mission=30km outbound + 30km return, payload=10kg, altitude=AGL 150m")
    print()
    print("energy_wh_p50,energy_wh_p90,energy_wh_p95,flight_min_p50,flight_min_p90")
    print(
        f"{percentile(energies, 50):.0f},"
        f"{percentile(energies, 90):.0f},"
        f"{percentile(energies, 95):.0f},"
        f"{percentile(flight_times, 50):.1f},"
        f"{percentile(flight_times, 90):.1f}"
    )
    print()
    print("battery,success_pct,margin_wh_p10,margin_wh_p50,margin_wh_p90")
    for battery in batteries:
        margins = [battery.battery_wh * trial.usable_fraction - trial.energy_wh for trial in trials]
        success = sum(1 for margin in margins if margin >= 0.0) / len(margins) * 100.0
        print(
            f"{battery.name},"
            f"{success:.1f},"
            f"{percentile(margins, 10):.0f},"
            f"{percentile(margins, 50):.0f},"
            f"{percentile(margins, 90):.0f}"
        )
    print()
    print("sample_means")
    print(f"wind_mps={mean(t.wind_mps for t in trials):.1f}")
    print(f"airspeed_mps={mean(t.airspeed_mps for t in trials):.1f}")
    print(f"cruise_power_w={mean(t.cruise_power_w for t in trials):.0f}")
    print(f"hover_power_w={mean(t.hover_power_w for t in trials):.0f}")
    print(f"accessory_power_w={mean(t.accessory_power_w for t in trials):.0f}")
    print(f"usable_fraction={mean(t.usable_fraction for t in trials):.2f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    run(args.samples, args.seed)


if __name__ == "__main__":
    main()
