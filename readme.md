# 先占个位置，后面写readme
所有的仓库依赖均给了正确的版本
# C++ 和 python 混编
怎么做到的，我要学习一下
# 编译和运行
```bash
# 1.克隆仓库
mkdir -p simple-mpc_ws/src
cd simple-mpc_ws/src
git clone git@github.com:Simple-Robotics/simple-mpc.git --recursive

# 2.用mamba
conda install mamba -n base -c conda-forge
mamba --version
mamba env create -f simple-mpc/environment-devel.yaml
# 每个新打开的终端都要初始化一下，否则无法用mamba打开环境
eval "$(mamba shell hook --shell bash)"  
mamba activate simple-mpc-devel

# 3.自动git一些conda没有的库 
# 库的版本均已配置好
vcs import --recursive < simple-mpc/devel-git-deps.yaml

# 4.将pinocchio库中 package.xml文件中的 hpp-fcl改为coal
 
# 5.编译依赖
export MAKEFLAGS="-j3"   # 这里尽量别选择4，否则会卡死
cd ..
colcon build --event-handlers console_direct+ --packages-ignore simple-mpc --cmake-args \
-DCMAKE_BUILD_TYPE=Release             \
-DCMAKE_PREFIX_PATH=$CONDA_PREFIX      \
-DPYTHON_EXECUTABLE=$(which python)    \
-DCMAKE_CXX_COMPILER_LAUNCHER='ccache' \
-DBUILD_TESTING=OFF                    \
-DBUILD_DOCUMENTATION=OFF              \
-DBUILD_EXAMPLES=OFF                   \
-DBUILD_BENCHMARK=OFF                  \
-DBUILD_BENCHMARKS=OFF                 \
-DBUILD_WITH_COLLISION_SUPPORT=ON      \
-DGENERATE_PYTHON_STUBS=OFF            \
-DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON

# 6.安装simple-mpc
source install/setup.bash
colcon build --packages-select simple-mpc

# 注意，动态库没有安装到根目录，每次运行代码时，记得source一下
source install/setup.bash
```
## 注意
1. fatal: 无法访问 'https://github.com/jrl-umi3218/jrl-cmakemodules.git/'：gnutls_handshake() failed: The TLS connection was non-properly terminated.
```bash
# 更新证书
sudo apt update
sudo apt install --reinstall ca-certificates
sudo update-ca-certificates
```
## 放在台式机上没有编译通过