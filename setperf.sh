echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

for i in 1 2 3 4 5 6 7 8 9
do
    echo 1 > /sys/devices/system/cpu/cpu$i/online
	cat /sys/devices/system/cpu/cpu$i/online
    echo performance > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
	cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
done
