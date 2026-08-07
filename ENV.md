make aet_h743-basic_default -j4 2>&1 | tail -n 50


PX4_SIM_HOST_ADDR=192.168.50.15 \
PX4_GCS_HOST_ADDR=192.168.50.15 \
make px4_sitl none_iris



### SIM
git clone --branch v1.12.3 --recursive \
  https://github.com/PX4/PX4-Autopilot.git \
  PX4-Autopilot-v1.12.3


bash Tools/setup/ubuntu.sh --no-nuttx


PX4_SIM_HOST_ADDR=192.168.50.15 \
make px4_sitl none_iris



make cuav_fmu-v6x_default



enp5s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.50.160  netmask 255.255.255.0  broadcast 192.168.50.255
        ether 2a:19:6f:02:18:11  txqueuelen 1000  (以太网)
        RX packets 6028694  bytes 5229706287 (5.2 GB)
        RX errors 0  dropped 3891  overruns 0  frame 0
        TX packets 2326337  bytes 412324690 (412.3 MB)
        TX errors 25  dropped 3 overruns 0  carrier 0  collisions 0
