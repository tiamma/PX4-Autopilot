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
