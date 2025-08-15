在v1.11.3固件中，需要更改的参数为：
[安全检查]
CBRK_USB_CHK = 197848       //插USB时可解锁
CBRK_IO_SAFETY = 22027      //无安全开关可解锁
CBRK_SUPPLY_CHK = 894281    //无电流计可解锁

[定位数据]
EKF2_AID_MASK = 24          //定位数据来自视觉
EKF2_HGT_MODE = 3           //高度数据主要来源为视觉


在v1.15.0固件中，需要更改的参数为：

[安全检查]
CBRK_USB_CHK = 197848       //插USB时可解锁
CBRK_IO_SAFETY = 22027      //无安全开关可解锁
CBRK_SUPPLY_CHK = 894281    //无电流计可解锁

[定位数据]
EKF2_GPS_CTRL = 0           //禁用GPS数据融合
EKF2_EV_CTRL = 15           //启用视觉数据融合
EKF2_HGT_REF = 3            //高度数据主要来源为视觉