# JamSense
### _Interference and Jamming Classification for Low-power Wireless Networks_

While [SpeckSense] can detect multiple sources of interference,
[JamSense] extends SpeckSense with the ability to identify and classify
jamming attacks in low-power wireless sensor networks. JamSense runs
as an [Contiki-NG] application on resource-constrained devices.

JamSense is currently configured only for [nRF52840] devices. Setup
instructions for the toolchain to build and program devices with
JamSense can be found in [Contiki-NG toolchain] and
[Contiki-NG for nRF52840].

### Building JamSense

Ensure all git submodules have been initialized:
```sh
git submodule update --init --recursive
```

Compile and program a nRF52840 device with JamSense
(replace <BOARD> and <PORT> with your device board and connected port)

```sh
cd jamsense
make TARGET=nrf52840 BOARD=<BOARD> -j
sudo make TARGET=nrf52840 BOARD=<BOARD> jamsense.dfu-upload PORT=<PORT>
make TARGET=nrf52840 BOARD=<BOARD> login PORT=<PORT>
```

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [SpeckSense]: <https://doi.org/10.1007/978-3-319-15582-1_3>
   [JamSense]: <https://doi.org/10.23919/WMNC53478.2021.9619007>
   [Contiki-NG]: <https://github.com/contiki-ng/contiki-ng>
   [nRF52840]: <https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840>
   [Contiki-NG toolchain]: <https://github.com/contiki-ng/contiki-ng/wiki/Toolchain-installation-on-Linux>
   [Contiki-NG for nRF52840]: <https://github.com/contiki-ng/contiki-ng/wiki/Platform-nrf52840>
