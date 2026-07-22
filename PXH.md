2. 重新建立 PX4遥测实例
在 PX4的 pxh> 中先清理可能残留的实例：
mavlink stop -u 18580
然后执行：
mavlink start -u 18580 -r 4000000 -m custom -o 14560 -t 127.0.0.1
依次开启消息：
mavlink stream -u 18580 -s HEARTBEAT -r 1
mavlink stream -u 18580 -s ATTITUDE_QUATERNION -r 30
mavlink stream -u 18580 -s LOCAL_POSITION_NED -r 30
mavlink stream -u 18580 -s GLOBAL_POSITION_INT -r 10
检查：
mavlink status streams
需要看到本地端口：
18580
以及远端：
127.0.0.1:14560


make px4_sitl sihsim_quadx
