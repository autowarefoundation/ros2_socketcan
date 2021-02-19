# If running in docker/ade need the following arguments:
# --privileged --cap-add=ALL -v /lib/modules:/lib/modules
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
