now=$(python3 -c "import datetime;print(datetime.datetime.now().isoformat(sep=' ', timespec='seconds'))")
ssh pi@10.42.0.1 "sudo timedatectl set-time '$now'"
