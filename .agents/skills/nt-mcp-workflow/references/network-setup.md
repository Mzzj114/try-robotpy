# Real robot network setup

Use this reference when bringing up a live connection to a **real robot** (not the sim). The goal is simultaneous access: the agent reaches the robot's NetworkTables synchronously while the laptop keeps its internet connection. Robot-bound traffic and internet traffic must leave through separate adapters.

## Topology

- **Preferred**: an ethernet cable from the laptop to the radio, leaving the wireless adapter free for internet. A USB network adapter works as the second adapter too.
- The robot network is `10.TE.AM.x` (radio `.1`, roboRIO `.2`), where `TE.AM` is the team number.
- `python -m robotpy deploy` does not need the Driver Station: a dev laptop and a test laptop (with DS) can be connected at the same time, one for deploying, the other for running.

## Static routing

With two adapters, add a static route so traffic to `10.0.0.0/8` goes through the robot adapter while everything else keeps the default (internet) route. All commands require admin privileges. Replace `<gateway>` with the robot adapter's gateway, and `<idx>` / `"<service>"` with the robot adapter's identity.

### Windows 11

Collect the inputs first (admin terminal):

```cmd
ipconfig
route print -4
```

`ipconfig` shows the robot adapter's IPv4, subnet mask, and gateway; `route print -4` shows the interface list — note the robot adapter's interface index (`Idx`).

Persistent route (survives reboot; drop `-p` for a temporary test route):

```cmd
route -p add 10.0.0.0 mask 255.0.0.0 <gateway> IF <idx>
```

To route a single IP only, replace the target with `10.TE.AM.2 mask 255.255.255.255`.

### macOS

Collect the exact service name of the robot adapter (it must match this output verbatim):

```bash
networksetup -listallnetworkservices
```

Persistent route:

```bash
sudo networksetup -setadditionalroutes "<service>" 10.0.0.0 255.0.0.0 <gateway>
```

Additional routes append as more `<network> <mask> <gateway>` triplets on the same command. Temporary route (lost on reboot): `sudo route add -net 10.0.0.0 <gateway> 255.0.0.0`.

## Verify and connect

- Windows: `route print` lists the new route under persistent routes; then `ping 10.TE.AM.2`.
- macOS: `route get 10.TE.AM.2` shows the robot gateway and interface; then `ping 10.TE.AM.2`.

Back up the routing table before changing it (`route print` on Windows, `netstat -rn` on macOS) so it can be restored if traffic breaks. Once `ping` succeeds, run `nt_connect` with `team_number` or `server_ip="10.TE.AM.2"` and continue with `automated-workflow.md`.
