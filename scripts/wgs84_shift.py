#!/usr/bin/env python3
"""
Compute a rough WGS84 "shift" in meters (East/North) from an input coordinate.

Interpretation:
- Input is latitude/longitude (degrees) on WGS84.
- Shift vector is in local tangent plane meters: +x East, +y North.
- We convert (east, north) into a (bearing, distance) and run an ellipsoidal
  geodesic "direct" solve (Vincenty) on WGS84 to get the output coordinate.

Notes / caveats:
- This is intended for small/medium offsets (meters to a few km). For very long
  distances or near-antipodal points, Vincenty may fail to converge.
- "East/North" is local to the start point; this treats the shift as a single
  geodesic segment with initial azimuth derived from that vector.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B_M = WGS84_A_M * (1.0 - WGS84_F)


@dataclass(frozen=True)
class Wgs84Coord:
    lat_deg: float
    lon_deg: float


def _wrap_lon_deg(lon_deg: float) -> float:
    lon = (lon_deg + 180.0) % 360.0 - 180.0
    # Avoid printing "-180" for values that are effectively +180.
    if lon == -180.0:
        return 180.0
    return lon


def _vincenty_direct_wgs84(
    lat1_deg: float, lon1_deg: float, azimuth_deg: float, distance_m: float
) -> Wgs84Coord:
    """
    Vincenty direct formula (WGS84).

    Returns the destination lat/lon in degrees.
    """
    if distance_m == 0.0:
        return Wgs84Coord(lat_deg=lat1_deg, lon_deg=_wrap_lon_deg(lon1_deg))

    a = WGS84_A_M
    b = WGS84_B_M
    f = WGS84_F

    phi1 = math.radians(lat1_deg)
    lam1 = math.radians(lon1_deg)
    alpha1 = math.radians(azimuth_deg)

    sin_alpha1 = math.sin(alpha1)
    cos_alpha1 = math.cos(alpha1)

    tan_u1 = (1.0 - f) * math.tan(phi1)
    u1 = math.atan(tan_u1)
    sin_u1 = math.sin(u1)
    cos_u1 = math.cos(u1)

    sigma1 = math.atan2(tan_u1, cos_alpha1)
    sin_alpha = cos_u1 * sin_alpha1
    cos_sq_alpha = 1.0 - sin_alpha * sin_alpha

    # u^2 (see Vincenty)
    a_sq_minus_b_sq = a * a - b * b
    u_sq = (cos_sq_alpha * a_sq_minus_b_sq) / (b * b)

    A = 1.0 + (u_sq / 16384.0) * (
        4096.0 + u_sq * (-768.0 + u_sq * (320.0 - 175.0 * u_sq))
    )
    B = (u_sq / 1024.0) * (256.0 + u_sq * (-128.0 + u_sq * (74.0 - 47.0 * u_sq)))

    sigma = distance_m / (b * A)
    sigma_prev = math.inf

    # Iterate until change is tiny (in radians).
    for _ in range(200):
        cos_2sigma_m = math.cos(2.0 * sigma1 + sigma)
        sin_sigma = math.sin(sigma)
        cos_sigma = math.cos(sigma)
        delta_sigma = B * sin_sigma * (
            cos_2sigma_m
            + (B / 4.0)
            * (
                cos_sigma * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m)
                - (B / 6.0)
                * cos_2sigma_m
                * (-3.0 + 4.0 * sin_sigma * sin_sigma)
                * (-3.0 + 4.0 * cos_2sigma_m * cos_2sigma_m)
            )
        )
        sigma_prev, sigma = sigma, (distance_m / (b * A)) + delta_sigma
        if abs(sigma - sigma_prev) < 1e-12:
            break
    else:
        raise RuntimeError("Vincenty direct did not converge (distance too large?)")

    cos_2sigma_m = math.cos(2.0 * sigma1 + sigma)
    sin_sigma = math.sin(sigma)
    cos_sigma = math.cos(sigma)

    tmp = sin_u1 * sin_sigma - cos_u1 * cos_sigma * cos_alpha1
    phi2 = math.atan2(
        sin_u1 * cos_sigma + cos_u1 * sin_sigma * cos_alpha1,
        (1.0 - f) * math.sqrt(sin_alpha * sin_alpha + tmp * tmp),
    )

    lam = math.atan2(
        sin_sigma * sin_alpha1,
        cos_u1 * cos_sigma - sin_u1 * sin_sigma * cos_alpha1,
    )

    C = (f / 16.0) * cos_sq_alpha * (4.0 + f * (4.0 - 3.0 * cos_sq_alpha))
    L = lam - (1.0 - C) * f * sin_alpha * (
        sigma
        + C
        * sin_sigma
        * (cos_2sigma_m + C * cos_sigma * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m))
    )

    lam2 = lam1 + L

    lat2_deg = math.degrees(phi2)
    lon2_deg = _wrap_lon_deg(math.degrees(lam2))
    return Wgs84Coord(lat_deg=lat2_deg, lon_deg=lon2_deg)


def _shift_en_to_azimuth_distance(east_m: float, north_m: float) -> tuple[float, float]:
    distance_m = math.hypot(east_m, north_m)
    if distance_m == 0.0:
        return 0.0, 0.0
    # Bearing: clockwise from North. atan2(E, N) gives that.
    azimuth_deg = math.degrees(math.atan2(east_m, north_m))
    if azimuth_deg < 0.0:
        azimuth_deg += 360.0
    return azimuth_deg, distance_m


def _parse_lat_lon_pair(text: str) -> tuple[float, float]:
    parts = [p.strip() for p in text.replace(" ", ",").split(",") if p.strip()]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Expected 'lat,lon' (e.g. '38.5,-110.79').")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Invalid lat/lon numbers.") from exc


def _validate_lat_lon(lat_deg: float, lon_deg: float) -> None:
    if not (-90.0 <= lat_deg <= 90.0):
        raise ValueError(f"Latitude out of range [-90, 90]: {lat_deg}")
    if not math.isfinite(lon_deg):
        raise ValueError(f"Longitude must be finite: {lon_deg}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert (WGS84 lat,lon) + (east,north meters) -> new WGS84 coordinate."
    )

    parser.add_argument(
        "--wgs84",
        type=_parse_lat_lon_pair,
        help="Input coordinate as 'lat,lon' in degrees (alternative to positional lat lon).",
    )
    parser.add_argument("lat", nargs="?", type=float, help="Latitude in degrees.")
    parser.add_argument("lon", nargs="?", type=float, help="Longitude in degrees.")
    parser.add_argument("east", type=float, help="Shift east (meters).")
    parser.add_argument("north", type=float, help="Shift north (meters).")
    parser.add_argument(
        "--format",
        choices=("text", "csv", "json"),
        default="text",
        help="Output format.",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=8,
        help="Decimal precision for text/csv output.",
    )

    args = parser.parse_args()

    if args.wgs84 is not None:
        if args.lat is not None or args.lon is not None:
            raise SystemExit("Use either --wgs84 or positional lat lon (not both).")
        lat1_deg, lon1_deg = args.wgs84
    else:
        if args.lat is None or args.lon is None:
            raise SystemExit("Missing lat/lon. Provide positional lat lon or --wgs84 'lat,lon'.")
        lat1_deg, lon1_deg = args.lat, args.lon

    _validate_lat_lon(lat1_deg, lon1_deg)

    azimuth_deg, distance_m = _shift_en_to_azimuth_distance(args.east, args.north)
    out = _vincenty_direct_wgs84(lat1_deg, lon1_deg, azimuth_deg, distance_m)

    if args.format == "json":
        print(
            json.dumps(
                {
                    "in": {"lat_deg": lat1_deg, "lon_deg": _wrap_lon_deg(lon1_deg)},
                    "shift": {"east_m": args.east, "north_m": args.north},
                    "geodesic": {"azimuth_deg": azimuth_deg, "distance_m": distance_m},
                    "out": {"lat_deg": out.lat_deg, "lon_deg": out.lon_deg},
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    prec = max(0, args.precision)
    fmt = f"{{:.{prec}f}}"
    if args.format == "csv":
        print(f"{fmt.format(out.lat_deg)},{fmt.format(out.lon_deg)}")
        return 0

    # text
    print(f"azimuth_deg={azimuth_deg:.6f} distance_m={distance_m:.3f}")
    print(f"lat_deg={fmt.format(out.lat_deg)} lon_deg={fmt.format(out.lon_deg)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
