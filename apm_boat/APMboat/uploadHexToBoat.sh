#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: uploadHexToBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" scp /tmp/APMboat.build/APMboat.hex ubuntu@$1:/home/ubuntu/APMboat/APMHexFile/APMboat.hex
sshpass -p "mtecsail" ssh ubuntu@$1 /home/ubuntu/APMboat/APMHexFile/upload.sh
