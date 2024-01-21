# AMC7812 Driver (Kernel Module)
This is the source code for my AMC7812 driver. It was developed to work on a Raspberry Pi, but can easily be modified to work on other embedded linux systems. It was built with linux v5.4.51 because it was what I already had installed on my RPi 3 at the time.

## How to Use
Compile with `$ make`. Next run `$ sudo ./install_script.sh` which copies the binary into the location where modprobe can find it. Then you can run `$ sudo modprobe amc7812`.