# MR2 base-local chrony setup

This setup makes the base system clock the experiment's local reference. It
does not require Internet time, GNSS, PTP, or changes to nginx. The rover is the
only NTP client allowed by the generated base configuration, and the base is
the rover's only configured time source.

This provides relative base/rover synchronization. The displayed UTC date may
be wrong after the base loses RTC power, but matched RTP one-way latency remains
valid when both hosts are locked to the same base clock.

## Safety and behavior

- The generator prints a candidate by default and does not mutate the host.
- `--output` refuses to overwrite unless `--force` is supplied.
- `--install` must be run as root, validates with `chronyd -p`, backs up the
  current complete `/etc/chrony/chrony.conf`, replaces it atomically, and
  restarts only `chrony.service`.
- A failed restart automatically restores the previous configuration and
  attempts to restart the service again.
- It does not install packages, change firewall rules, change nginx, or run
  `chronyc makestep` from the dashboard.
- The rover configuration permits stepping offsets over 100 ms only during the
  first ten clock updates after chronyd starts. Normal operation uses slewing.

## Prerequisites

Install chrony on both computers using the operating-system package manager.
For Ubuntu:

```bash
sudo apt-get update
sudo apt-get install chrony
```

Confirm `MR2_BASE_IP` and `MR2_ROVER_IP` are the addresses routed through the
Rocket M2 link. UDP port 123 from rover to base must be allowed by host
firewalls. No remote chronyc command port needs to be exposed.

## Preview and validate

From the repository root on the base:

```bash
python3 scripts/latency/chrony/configure_chrony.py \
  --role base \
  --base-ip "$MR2_BASE_IP" \
  --rover-ip "$MR2_ROVER_IP" \
  --output /tmp/mr2-base-chrony.conf

chronyd -p -f /tmp/mr2-base-chrony.conf >/dev/null
```

On the rover:

```bash
python3 scripts/latency/chrony/configure_chrony.py \
  --role rover \
  --base-ip "$MR2_BASE_IP" \
  --output /tmp/mr2-rover-chrony.conf

chronyd -p -f /tmp/mr2-rover-chrony.conf >/dev/null
```

Review both complete files. They intentionally contain no distribution pool or
Internet server directives.

## Install

Install the base first:

```bash
sudo python3 scripts/latency/chrony/configure_chrony.py \
  --role base \
  --base-ip "$MR2_BASE_IP" \
  --rover-ip "$MR2_ROVER_IP" \
  --install
```

Then install the rover configuration on the rover:

```bash
sudo python3 scripts/latency/chrony/configure_chrony.py \
  --role rover \
  --base-ip "$MR2_BASE_IP" \
  --install
```

Each command prints the timestamped backup path when it replaces an existing
configuration.

## Verify before capture

On the base, `chronyc tracking` should report the local reference, stratum 8,
and a normal leap status:

```bash
chronyc -n tracking
```

On the rover, the base should become the selected source (`^*`) and tracking
should normally report stratum 9:

```bash
chronyc -n sources -v
chronyc -n tracking
chronyc waitsync 30 0.002 0 1
```

The final command waits up to about 30 seconds for synchronization and a
remaining correction below 2 ms. The dashboard `Check chrony sync` button is
read-only and checks the same runtime state. It marks the experiment ready only
when both status samples are fresh and the rover offset/error bound are within
2 ms.

When ready, analyze synchronized captures with:

```text
--clock-offset-us 0
```

## Rollback

If a previous configuration was backed up, restore the exact path printed by
the installer:

```bash
sudo cp /etc/chrony/chrony.conf.mr2-backup-<timestamp> /etc/chrony/chrony.conf
sudo systemctl restart chrony.service
chronyc tracking
```

## Accuracy boundary

chrony filters measurements continuously and tracks oscillator frequency, but
it cannot completely remove persistent forward/reverse delay asymmetry in the
Rocket M2 path. The dashboard error bound is:

```text
root_dispersion + 0.5 * abs(root_delay)
```

Capture this status in the exported latency JSONL. For experiments requiring an
error substantially below the displayed bound, use an independent PPS/PTP
reference at both hosts.
