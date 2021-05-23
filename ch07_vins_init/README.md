# 第7讲 VINS初始化和VIO系统

-----

[TOC]

## VINS鲁棒初始化

<p align="center">
  <img src="images/ch07_visual_imu_align.jpg" style="width:90%;"/>
</p>

## VINS系统

VINS系统三大块：
* 前端，数据处理：特征提取匹配，imu积分
* 初始化：系统初始状态变量(重力方向,速度,尺度等等)
* 后端：滑动窗口优化

<p align="center">
  <img src="images/ch07_vins.jpg" style="width:90%;"/>
</p>
