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
Note: 4 file quan trong vchar_driver.c &.h & Makefile & app_test.c con lai thi co the xoa di

Reference: 
```
https://vimentor.com/vi/lesson/dang-ky-entry-point-read-va-write
https://www.tldp.org/LDP/lkmpg/2.4/html/x579.html
https://vimentor.com/vi/lesson/gioi-thieu-character-driver
```

Yêu cầu:

Tạo một character device có tên /dev/mylog hộ trợ hai thao tác read và write như sau

-	write sẽ cho phép tiến trình ghi thêm dữ liệu vào phần cuối của buffer.
-	read sẽ bắt đầu đọc dữ liệu từ đầu buffer và tăng offset, khi không còn dữ liệu để đọc read sẽ bị block cho tới khi có một tiến trình khác ghi thêm dữ liệu mới

Buffer của device có thể tăng vô hạn (nghĩa là các bạn không cần phải quan tâm tới việc clean up memory).

Yêu cầu nâng cao: Hỗ trợ nonblocking io và các hàm select, epoll.

Thời gian làm bài: 4 tuần (28/12/2019)

Bài nộp gồm:

-	.doc(docx): mô tả các cấu trúc dữ liệu sử dụng, thiết kế của chương trình, hướng dẫn sử dụng
-	README: hướng dẫn build và cài đặt
-	Thư mục src, chứa source code.

