# TX2i ROS2 Auto-Run Setup

This document captures the final working auto-start setup for a TX2i so it launches robot sensors automatically on boot.

## Goal

On power-on, the TX2i should automatically:

1. Prepare ROS environment
2. Launch robot sensors bringup
3. Configure and activate the Microstrain lifecycle node

Equivalent manual workflow:

- `rosboot` (includes **`lidar_net`**: static **`192.168.1.5/24`** on the Ethernet port wired to the MID360 — required for Livox topics)
- `cd ~/ROS_Robot`
- `ros2 launch ros_robot_bringup robot_sensors.launch.py > ~/robot_sensors.log 2>&1 &`
- wait 5 seconds
- `ros2 lifecycle set /microstrain_inertial_driver configure`
- `ros2 lifecycle set /microstrain_inertial_driver activate`

---

## Files and Locations

- Boot script: `/home/ubuntu/robot_autostart.sh`
- systemd service: `/etc/systemd/system/robot-autostart.service`
- Boot log: `/home/ubuntu/robot_autostart_boot.log`
- Launch log: `/home/ubuntu/robot_sensors.log`

---

## Livox MID360: NIC before launch

`MID360_config.json` expects the **host** side (Jetson) at **`192.168.1.5`** on the **physical Ethernet** that goes to the lidar (subnet **`192.168.1.0/24`**). That is what **`lidar_net`** in `rosboot` configures.

The autostart script below duplicates that **before** `ros2 launch`.

1. On the TX2i, set **`ETH_IFACE`** to the interface that is cabled to the MID360 (not Wi‑Fi):

   ```bash
   ip -br link
   ```

2. **`LIVOX_IP`** must match **`host_net_info`** in your JSON (typically **`192.168.1.5/24`**).

3. systemd runs **non-interactive** `sudo`, so configure **`sudoers`** (see next section) or the `ip` commands will fail silently with `sudo -n`.

---

## sudoers (passwordless `ip` for `ubuntu`)

Create `/etc/sudoers.d/robot_network` with **`visudo`**:

```bash
sudo visudo -f /etc/sudoers.d/robot_network
```

Replace **`enPxxxx`** with your real **`ETH_IFACE`** (output of `ip -br link`):

```text
# Livox NIC — match ETH_IFACE on this machine
ubuntu ALL=(root) NOPASSWD: /sbin/ip addr flush dev enPxxxx
ubuntu ALL=(root) NOPASSWD: /sbin/ip addr add 192.168.1.5/24 dev enPxxxx
ubuntu ALL=(root) NOPASSWD: /sbin/ip link set dev enPxxxx up
```

If `ip` lives under `/usr/sbin` on your image, run `command -v ip` and use that full path in all three lines.

---

## Final Boot Script

Create/update `/home/ubuntu/robot_autostart.sh`:

```bash
#!/usr/bin/env bash
set -u

exec >> /home/ubuntu/robot_autostart_boot.log 2>&1
echo "=== robot_autostart $(date) ==="

export HOME=/home/ubuntu
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export COLCON_TRACE=0

# --- EDIT: physical Ethernet to MID360 (must match your TX2i) ---
ETH_IFACE="enP8p1s0"
LIVOX_IP="192.168.1.5/24"

# Wait for hotspot/default route (optional; adjust gateway IP if not iPhone hotspot)
for i in {1..60}; do
  if ip route | grep -q '^default' && ping -c1 -W1 172.20.10.1 >/dev/null 2>&1; then
    echo "Network ready."
    break
  fi
  sleep 1
done

# lidar_net (same role as in rosboot) — must run before Livox driver
if sudo -n ip addr flush dev "$ETH_IFACE" \
  && sudo -n ip addr add "$LIVOX_IP" dev "$ETH_IFACE" \
  && sudo -n ip link set dev "$ETH_IFACE" up; then
  echo "[OK] Livox NIC $ETH_IFACE -> $LIVOX_IP"
else
  echo "[WARN] Livox NIC setup failed (check ETH_IFACE, cable, sudoers). Livox topics may be missing."
fi

# Optional cleanup (non-fatal)
pkill -f "ros2 launch ros_robot_bringup robot_sensors.launch.py" 2>/dev/null || true
pkill -f livox_ros_driver2_node 2>/dev/null || true
pkill -f microstrain_inertial_driver 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/PHS-* 2>/dev/null || true

# Source environments (disable nounset while sourcing colcon setup files)
set +u
source /home/ubuntu/ros2_foxy/install/setup.bash
source /home/ubuntu/livox_ws/install/setup.bash
source /home/ubuntu/ROS_Robot/install/setup.bash
set -u

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[OK] Environments initialised"

cd /home/ubuntu/ROS_Robot || exit 1

nohup ros2 launch ros_robot_bringup robot_sensors.launch.py > /home/ubuntu/robot_sensors.log 2>&1 &
sleep 5

# Wait for lifecycle service to appear
for i in {1..60}; do
  if ros2 service list | grep -q "/microstrain_inertial_driver/change_state"; then
    echo "Lifecycle service is available."
    break
  fi
  sleep 1
done

ros2 lifecycle set /microstrain_inertial_driver configure || true
sleep 2
ros2 lifecycle set /microstrain_inertial_driver activate || true

echo "Autostart sequence complete."
```

Make executable:

```bash
chmod +x /home/ubuntu/robot_autostart.sh
```

---

## Final systemd Service

Create/update `/etc/systemd/system/robot-autostart.service`:

```ini
[Unit]
Description=Robot ROS bringup autostart
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=ubuntu
WorkingDirectory=/home/ubuntu/ROS_Robot
Environment=HOME=/home/ubuntu
ExecStart=/home/ubuntu/robot_autostart.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Notes:

- Do not set `Restart=` with `Type=oneshot` on this device image; it was rejected as invalid.
- Keep `RemainAfterExit=yes` so the unit remains active after script completion.

---

## Enable and Run

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-autostart.service
sudo systemctl reset-failed robot-autostart.service
sudo systemctl restart robot-autostart.service
```

Check status and logs:

```bash
sudo systemctl status robot-autostart.service -l --no-pager
sudo journalctl -u robot-autostart.service -b --no-pager
tail -n 200 /home/ubuntu/robot_autostart_boot.log
tail -n 200 /home/ubuntu/robot_sensors.log
```

---

## Verification After Reboot

After power cycle:

```bash
sudo systemctl status robot-autostart.service --no-pager
ps aux | grep "ros2 launch ros_robot_bringup" | grep -v grep
ros2 topic list
```

Expected:

- `robot-autostart.service` is `active (exited)` with `status=0/SUCCESS`
- launch process is running
- expected topics are visible (for example `/livox/lidar`, `/imu/data`)

---

## Troubleshooting Notes (Observed During Setup)

- `status=203/EXEC`: script not executable or bad file format
  - fix with `chmod +x /home/ubuntu/robot_autostart.sh`
  - if needed: `dos2unix /home/ubuntu/robot_autostart.sh`

- `/opt/ros/humble/setup.bash: No such file or directory`
  - this setup uses local ROS path: `/home/ubuntu/ros2_foxy/install/setup.bash`

- `a password is required` from `sudo` inside service
  - remove/avoid interactive sudo in boot script
  - use non-sudo cleanup or configure explicit NOPASSWD sudoers entry

- Livox works after manual `rosboot` but not after cold boot
  - autostart was missing **`lidar_net`**; add the **`ip addr add 192.168.1.5/24`** block and **`sudoers`** for `ip` as above

- `/home/ubuntu/ros2_foxy/install/setup.bash: line 11: COLCON_TRACE: unbound variable`
  - source setup scripts under `set +u`, then restore `set -u`

- `not found: ".../pcl_ros/local_setup.bash"`
  - non-fatal for current launch, but indicates stale overlay reference in ROS install/workspace

---

## Related User Shell Helpers

These were also added on the laptop side for convenience:

- `txconnect` to auto-discover SSH host with open port 22 on active subnet and connect as `ubuntu`
- `rosboot` shell function configured for:
  - `ROS_DOMAIN_ID=10`
  - `ROS_LOCALHOST_ONLY=0`

