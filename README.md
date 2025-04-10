# README

一个非常重要的地方需要我说一下就是这个代码实际上是根据NCHU南昌航空大学洪鹰战队的gitee开源改造得来(https://gitee.com/LitzJ/basic_framework_mc02) , 原网址放这里大家可以去给他们点点star.

## 开发流

本作品采用的是 stm32cubemx + vscode + ozone 开发流, 编译工具链选用的是 mingw32 和 arm-none-eabi-objcopy, 具体的教学可以看湖大开源上的教学(https://gitee.com/hnuyuelurm/basic_framework) , 同时本作品也是以湖大为基础框架更改得来的.

## 更新

新增了buffer作为消除电控视觉通信时由于没有时间戳导致的相位差的工具, 该工具可以保存某个变量0到20个周期的数据
新增了
新增了功率限制
