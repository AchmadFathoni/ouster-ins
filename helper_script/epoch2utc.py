import time, sys

epoch_time = int(sys.argv[1])
utc = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch_time))
print(utc)
