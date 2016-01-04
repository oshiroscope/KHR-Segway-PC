modprobe ftdi_sio
echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
