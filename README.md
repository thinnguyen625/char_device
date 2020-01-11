# char_device_driver

Tạo và khởi tạo 1 thiết bị ảo nằm trên RAM, và viết driver để điều khiển đọc ghi thiết bị đó

```
make
sudo dmesg -C
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
Note: 4 file quan trong mylogg_driver.c &.h & Makefile & app_test.c con lai thi co the xoa di

Reference: 
```
https://www.tldp.org/LDP/lkmpg/2.4/html/x579.html
https://vimentor.com/vi/lesson/dang-ky-entry-point-read-va-write
https://vimentor.com/vi/lesson/gioi-thieu-character-driver
```


