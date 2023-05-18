这些脚本是哈工大2023年春预测控制课程报告对应的代码。

运行matlab脚本需要安装：

>MPT3
>
>CasADi

运行ipynb中公式推导部分需要安装：

>sympy

下载CasADi后需要将./quadrotor_nmpc/NMPC_problem_formulation.m中

```
addpath('D:\matlab_mpt_tbx_canrm\casadi-3.6.1-windows64-matlab2018b');
```

改成CasADi下载位置

### 四旋翼动力学推导和线性化状态方程输出

quadrotor_modelformulation.ipynb

输出结果在：system.mat

### 四旋翼非线性动力学方程与基于方程的仿真

四旋翼非线性动力学方程由FormulateNolinearStatefunction.m里面的函数构建，并保存在NolinearStatefunction.mat

基于四旋翼非线性动力学方程的仿真：

sim_nolinearquad.m

### 约束预测控制：

运行quadrotor_mpc.m会仿真跟踪一段运动轨迹

之后plot_process_gif.m脚本会生成运动过程gif图

运行plot_test.m脚本会画出系统状态围绕参考轨迹的变化以及系统控制输入图

mpc1.gif mpc2.gif是分别生成两条不同轨迹的跟踪动画

### 显式预测控制：

运行quadrotor_empc.m会仿真镇定在原点的过程

之后plot_process_gif.m脚本会生成运动过程gif图

运行plot_test.m脚本会画出系统状态围绕参考轨迹的变化以及系统控制输入图

empc.gif是生成运动过程动画

### 非线性预测控制

在quadrotor_nmpc文件架内

运行quadrotor_nmpc.m会仿真跟踪一段运动轨迹

之后plot_process_gif.m脚本会生成运动过程gif图

运行plot_test.m脚本会画出系统状态围绕参考轨迹的变化以及系统控制输入图

nmpc1.gif nmpc2.gif是分别生成两条不同参考轨迹的跟踪动画，这里的参考轨迹比约束预测控制更快速，机动性更高，由QuadrotorReferenceTrajectory.m中的函数计算给出参考轨迹

NMPC_problem_formulation.m：是系统的非线性状态方程构建和非线性模型预测控制的优化问题的构建，最后将生成结果保存在NMPC_problem_definition.mat中







