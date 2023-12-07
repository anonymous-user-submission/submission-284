## submission-284

Here, we detail the instructions to build both hardware and software parts.

### Dependencies
1. We use vivado v2020.2 to build QingNiao FPGA prototype.
2. To build QingNiao library, DPDK v21.11 is required.

### Hardware prototype

Instructions to build QingNiao hardware prototype, which is based on [Corundum](https://github.com/corundum/corundum). And we use a [open-source CAM implementation](https://github.com/alexforencich/verilog-cam) where we fix a bug in implementing QingNiao.

```bash
# clone this repo
git clone https://github.com/anonymous-user-submission/submission-284 submission-284

# patch souce code
git clone https://github.com/corundum/corundum.git qn-hw-prototype
cd qn-hw-prototype
git checkout 56fe10f27d9b42f1ff9abe4d735b113008e4be9d

cd fpga/common
patch -i ~/submission-284/hardware/src-code.diff -p2

# get the CAM library, cd back to folder qn-hw-prototype
cd fpga/common/lib
cp -r ~/submission-284/hardware/verilog-cam ./

# patch configuration scripts for AU250, cd back to folder qn-hw-prototype
cd fpga/mqnic/AU250/fpga_100g
patch -i ~/submission-284/hardware/conf.diff

# start to build
make all
```

### Software

We provide driver and a simple echo application to run QingNiao under `software` folder.

```bash
# edit CMakeLists.txt to find DPDK lib
cd software
mkdir -p build
cd build
cmake .. && make -j
```



