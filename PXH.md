2. 重新建立 PX4遥测实例
在 PX4的 pxh> 中先清理可能残留的实例：
mavlink stop -u 18580
然后执行：
mavlink start -u 18571 -r 4000000 -m custom -o 14550 -t 192.168.50.15
依次开启消息：
mavlink stream -u 18571 -s HEARTBEAT -r 1
mavlink stream -u 18571 -s ATTITUDE_QUATERNION -r 30
mavlink stream -u 18571 -s LOCAL_POSITION_NED -r 30
mavlink stream -u 18571 -s GLOBAL_POSITION_INT -r 10
检查：
mavlink status streams
需要看到本地端口：
18580
以及远端：
127.0.0.1:14560


make px4_sitl sihsim_quadx


docker run --rm -it   --network my_macvlan   --ip 192.168.50.240   --name px4_sitl   --add-host=host.docker.internal:192.168.50.15   -e PX4_SIM_MODEL=sihsim_quadx   px4io/px4-sitl:latest


PX4_SIM_HOST_ADDR=192.168.50.15 make px4_sitl none_iris
