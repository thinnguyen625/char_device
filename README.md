# char_device_driver

Tạo và khởi tạo 1 thiết bị ảo nằm trên RAM, và viết driver để điều khiển đọc ghi thiết bị đó

```
make
sudo dmesg -C
sudo insmod vchar_driver.ko
dmesg
sudo chmod 666 /dev/vchar_dev
./app_test
dmes
sudo rmmod vchar_driver

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
