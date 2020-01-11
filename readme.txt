# char_device_driver

```
make
sudo insmod mylog_driver.ko
dmesg
sudo chmod 666 /dev/mylog
./app_test
dmesg
sudo rmmod mylog_driver

```

Can check module by command:
```
lsmod: show module
rmmod: delete module
```
Check major number: 
```
cat /proc/devices
```
Note: 4 file quan trong mylog_driver.c &.h & Makefile & app_test.c con lai thi co the xoa di dc



