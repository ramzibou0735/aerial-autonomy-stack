# PX4 Setup

## Configure PX4's Network and DDS Client

Access QGroundControl -> Analyze Tools -> MAVLink console

```sh
mkdir /fs/microsd/etc
echo "uxrce_dds_client stop" > /fs/microsd/etc/extras.txt
echo "sleep 3" >> /fs/microsd/etc/extras.txt
# With the NX on 10.10.1.5
echo "uxrce_dds_client start -p 8888 -h 10.10.1.5 -n Drone1" >> /fs/microsd/etc/extras.txt

echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.10.1.10 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo ROUTER=10.10.1.254 >> /fs/microsd/net.cfg
echo DNS=10.10.1.254 >> /fs/microsd/net.cfg

#check files
cat /fs/microsd/etc/extras.txt
cat /fs/microsd/net.cfg

netman update
```