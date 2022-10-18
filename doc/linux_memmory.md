查看内存峰值和当前值
cat /proc/2699/status | grep -A 1 VmHWM
查看系统日志
dmesg