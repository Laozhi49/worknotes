# Using STM32_Programmer Linux Arm
## Download STM32_Programmer
site: https://www.st.com/en/development-tools/stm32cubeprog.html

download version: Linux Arm
## Install STM32_Programmer
```sh
# step 1: copy stm32cubeprogrammer_2.23.0_arm64.deb.zip to your board

# step 2: unzip 
unzip stm32cubeprogrammer_2.23.0_arm64.deb.zip

# step 3 : install
dpkg -i stm32cubeprogrammer_2.23.0_arm64.deb
```
## Verify
```sh
# check install
which STM32_Programmer_CLI

# check version
STM32_Programmer_CLI --version
```

## Download
```sh
STM32_Programmer_CLI \
  -c port=SWD \
  -d xxx.bin 0x08000000 \
  -v \
  -rst
```

# Using STM32_Programmer Generic Linux
## install box64
```sh
apt install box64
```
## download STM32_Programmer Linux Version
site: https://www.st.com/en/development-tools/stm32cubeprog.html

download version : Generic Linux
## install
```sh
# step 1: unzip in PC

# step 2: install in PC

# step 3: copy to Linux board

```

## creat a wrapper
```sh
# Step 1: Backup the original binary
cd ~/workspace1/stm32_programmer/bin
mv STM32_Programmer_CLI STM32_Programmer_CLI.real

# Step 2: Create a wrapper script
cat > STM32_Programmer_CLI << 'EOF'
#!/bin/bash
exec /usr/bin/box64 "$(dirname "$0")/STM32_Programmer_CLI.real" "$@"
EOF

# Step 3: Make it executable
chmod +x STM32_Programmer_CLI

# Step 4: Test the wrapper
./STM32_Programmer_CLI --version

```

## Debug

```sh
# flash
box64 ./stm32_programmer/bin/STM32_Programmer_CLI \
  --connect port=SWD speed=fast ap=1 mode=UR reset=hwRst \
  --download <firmware>

# gdbserver
box64 ./stm32_gdbserver/bin/ST-LINK_gdbserver --swd --halt -m 1 -cp stm32_programmer/bin

# gdb
gdb-multiarch <firmware>.elf

```