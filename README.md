# Add telemetry agent functions to your Aira instance
Aira source package to input data from sensors. ROS-enabled telemetry agent

# Module For Your Aira Instance. Add Telemetry Agent

The Aira package allows you to read data from a SDS011 sensor (and a few others) and publish to different output channels.
That said Aira is able to form Demand and Result messages and a few other channels.
Also it includes Datalog feature which is still experimental. It could be used to publish data to Substrate based blockchain by [Robonomics](https://parachain.robonomics.network/).

## Get a Package And Build

Assuming you work under [AIRA](https://wiki.robonomics.network/docs/aira-installation-on-vb/) do the following:

```
su - liability
git clone https://github.com/airalab/sensors-connectivity
cd sensors-connectivity
nix build -f release.nix
```

> **Allow liability user to open port `/dev/ttyUSB0`**
>
> Most probably a user liability is not in the dialout group
>
> Either do `sudo adduser liability dialout` or add the line to `/etc/nixos/configuration.nix` on NixOS:
> `users.users.liability.extraGroups = [ "dialout" ];`

## Documentation

To prepare a sensor for the work with the package follow instructions on [Robonomics Wiki](https://wiki.robonomics.network/docs/connect-sensor-to-robonomics/) and use [another](https://wiki.robonomics.network/docs/configuration-options-description/) article to set a proper configuration for your instance.


## Install Robonomics

From `root` user do:

```
echo "https://github.com/airalab/airapkgs/archive/nixos-unstable.tar.gz nixos" > /root/.nix-channels
nix-channel --update
```

Then edit `/etc/nixos/configuration.nix` and add:

```
  environment.systemPackages = with pkgs; [
        substrate-node-robonomics-bin
  ];
```

Run rebuild and find out where `robonomics` is:
```
nixos-rebuild switch
whereis robonomics
```

Let's assume you got the following path: `/nix/store/2gz2ik17w5xad8w819bsb05a23pbjbya-system-path/bin/robonomics`. Use the path in `datalog/path` option.

## Make a Service

It's way more convenient to launch the code as a systemd service. Add the following in `/etc/nixos/configuration.nix`:

```
systemd.services.connectivity = {
    requires = [ "roscore.service" ];
    after = ["roscore.service" ];
    wantedBy = [ "multi-user.target" ];
    environment.ROS_MASTER_URI =  "http://localhost:11311";
    script = ''
        source /var/lib/liability/sensors-connectivity/result/setup.bash \
        && roslaunch sensors_connectivity agent.launch config:=/var/lib/liability/sensors-connectivity/config/my.json
    '';
    serviceConfig = {
        Restart = "on-failure";
        StartLimitInterval = 0;
        RestartSec = 60;
        User = "liability";
    };
};
```

Technically it's not necessary to specify the `default.json` configuration file, but if you did changes please put the absolute path to `config` parameter.

After that run `nixos-rebuild switch`. The service should be up and running

## Check Your Connectivity Service 

Usually log files are stored in `/var/lib/liability/.ros/log/latest/connectivity-worker-1.log`
To view logs do:
```
tail -f /var/lib/liability/.ros/log/latest/connectivity-worker-1.log
```

Also you can use `journalctl` but remember output is buffered so it could take some time before output appears
```
journalctl -u connectivity -f
```

Example of output:
```bash
root@hq-nuc-sds011> tail /var/lib/liability/.ros/log/latest/connectivity-worker-1.log -f 
[rosout][INFO] 2020-12-07 12:00:02,536: Starting process...
[rosout][INFO] 2020-12-07 12:00:02,541: {14542862: Measurement(public='0ed57cf15050d5fa56c8da5e382ec5af5e923d7ee2b931a81ef8381fc5ac921d', model=2, pm25=14.6, pm10=22.33, geo_lat='53.541009', geo_lon='49.402139', timestamp=1607338677), 15695587: Measurement(public='a8cdc84231d0dd62549a3cb697f2928976eb3b0852066f8c873e9851e0e2840f', model=2, pm25=3.15, pm10=8.43, geo_lat='59.936439', geo_lon='30.499921', timestamp=1607338693), 11117092: Measurement(public='20e67a8db1a66b3def367c737a6faf9a622a4c832efc9f91b37c358119653a04', model=2, pm25=7.97, pm10=15.32, geo_lat='55.741614', geo_lon='37.537618', timestamp=1607338714), 2630646: Measurement(public='b2add8c5b5ac9f63bf01932a732b397221e79305d0b767b1879afb72861b8469', model=2, pm25=18.73, pm10=31.62, geo_lat='48.828908', geo_lon='2.370269', timestamp=1607338744), 12624680: Measurement(public='c55b3d2a6b042203ed2e23989b2308052c2d9a0db61932a1b14f0f291860cd74', model=2, pm25=7.3, pm10=10.0, geo_lat='46.856023', geo_lon='40.314081', timestamp=1607338751)}
[rosout][INFO] 2020-12-07 12:00:02,545: HTTPServer status: True
[rosout][INFO] 2020-12-07 12:00:02,548: airalab-http-v0.3.0: [{MAC: a6802ccd9f70, Uptime: 4 days, 0:21:04.491420, M: {Public: 0ed57cf15050d5fa56c8da5e382ec5af5e923d7ee2b931a81ef8381fc5ac921d, PM2.5: 14.6, PM10: 22.33, geo: (53.541009,49.402139), timestamp: 1607338677}}, {MAC: a6802ccd9f70, Uptime: 4 days, 0:21:04.491431, M: {Public: a8cdc84231d0dd62549a3cb697f2928976eb3b0852066f8c873e9851e0e2840f, PM2.5: 3.15, PM10: 8.43, geo: (59.936439,30.499921), timestamp: 1607338693}}, {MAC: a6802ccd9f70, Uptime: 4 days, 0:21:04.491433, M: {Public: 20e67a8db1a66b3def367c737a6faf9a622a4c832efc9f91b37c358119653a04, PM2.5: 7.97, PM10: 15.32, geo: (55.741614,37.537618), timestamp: 1607338714}}, {MAC: a6802ccd9f70, Uptime: 4 days, 0:21:04.491435, M: {Public: b2add8c5b5ac9f63bf01932a732b397221e79305d0b767b1879afb72861b8469, PM2.5: 18.73, PM10: 31.62, geo: (48.828908,2.370269), timestamp: 1607338744}}, {MAC: a6802ccd9f70, Uptime: 4 days, 0:21:04.491437, M: {Public: c55b3d2a6b042203ed2e23989b2308052c2d9a0db61932a1b14f0f291860cd74, PM2.5: 7.3, PM10: 10.0, geo: (46.856023,40.314081), timestamp: 1607338751}}]
[rosout][INFO] 2020-12-07 12:00:02,551: RobonomicsFeeder: {"0ed57cf15050d5fa56c8da5e382ec5af5e923d7ee2b931a81ef8381fc5ac921d": {"model": 2, "timestamp": 1607338677, "measurement": {"pm25": 14.6, "pm10": 22.33, "geo": "53.541009,49.402139"}}}
[rosout][INFO] 2020-12-07 12:00:02,565: RobonomicsFeeder: {"a8cdc84231d0dd62549a3cb697f2928976eb3b0852066f8c873e9851e0e2840f": {"model": 2, "timestamp": 1607338693, "measurement": {"pm25": 3.15, "pm10": 8.43, "geo": "59.936439,30.499921"}}}
[rosout][INFO] 2020-12-07 12:00:02,578: RobonomicsFeeder: {"20e67a8db1a66b3def367c737a6faf9a622a4c832efc9f91b37c358119653a04": {"model": 2, "timestamp": 1607338714, "measurement": {"pm25": 7.97, "pm10": 15.32, "geo": "55.741614,37.537618"}}}
[rosout][INFO] 2020-12-07 12:00:02,592: RobonomicsFeeder: {"b2add8c5b5ac9f63bf01932a732b397221e79305d0b767b1879afb72861b8469": {"model": 2, "timestamp": 1607338744, "measurement": {"pm25": 18.73, "pm10": 31.62, "geo": "48.828908,2.370269"}}}
[rosout][INFO] 2020-12-07 12:00:02,604: RobonomicsFeeder: {"c55b3d2a6b042203ed2e23989b2308052c2d9a0db61932a1b14f0f291860cd74": {"model": 2, "timestamp": 1607338751, "measurement": {"pm25": 7.3, "pm10": 10.0, "geo": "46.856023,40.314081"}}}
[rosout][INFO] 2020-12-07 12:00:02,616: DatalogFeeder:
[rosout][INFO] 2020-12-07 12:00:02,618: Still collecting measurements...
```
