Project: MUYUAN_ICAS_BRAKE4.1
JWD Automotive Co., Ltd. 

MUYUAN_ICAS_BRAKE4.1
git@github.com:gao-zhengzhong/iCAS_Brake_MuYuan.git

软件版本号规则：
<硬件版本号>_<项目><日期><版本>
例：HW01_MuYuan20000101V1.0

2021.01.20
版本信息：
HW05_MuYuan20210120V1.4
版本说明：
牧原生产批次：JWD202101002
1.改用0.5~4.5V范围压力传感器
2.增加保护逻辑


2020.12.30
版本信息：
HW05_MuYuan20201230V1.4
版本说明：
根据Polyspace工具扫描出的潜在风险问题进行修复
1.task3.c模块,不可靠的指针转换处理问题
2.energy_recovery.c,rte.c模块,数组越界问题
3.energy_recovery.c模块,操作符优先级潜在风险问题
4.desc_ser.c模块,字符串数组中缺少null问题
5.can_com.c,pid.c模块,无符号整数转换溢出问题
6.booster.c,ccp.c模块,符号更改整数转换溢出问题
7.can_com.c,booster.c模块,无效if逻辑问题
8.desc_ser.c,rte.c模块,死代码问题
9.booster.c,wrie_control.c,desc.c,modecontrol.c模块,无用的定义变量


2020.12.25
版本信息：
HW05_MuYuan20201225V1.3
版本说明：
1.增加CCP标定
2.实现压力闭环控制
3.更新诊断功能模块