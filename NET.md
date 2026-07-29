docker network create -d macvlan \
  --subnet=192.168.50.0/24 \
  --gateway=192.168.50.1 \
  -o parent=enp5s0 \
  my_macvlan



docker run --rm -it \
  -p 14550:14550/udp \
  --name px4_sitl \
  --add-host=host.docker.internal:192.168.50.160 \
  -e PX4_SIM_MODEL=sihsim_quadx \
  px4io/px4-sitl:latest
