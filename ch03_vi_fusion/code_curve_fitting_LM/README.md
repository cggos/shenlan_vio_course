# 说明

## 代码说明

app 文件夹下实现了曲线问题的定义，残差计算，数据产生，以及主程序。

backend 文件夹下主要是最小二乘问题求解的一些函数定义。

### 代码编译

``` c++
cd CurveFitting_LM
mkdir build
cd build
cmake ..
make -j4    
```

代码运行

```c++
./CurveFitting_LM
```

