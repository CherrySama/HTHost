# Host 自碰撞模型

`panthera.xml` 派生自 SDK 的 `panthera_python/Panthera-HT_description_V1.1.0/mujoco/scene.xml`。
外观、惯性、关节转轴、限位和力矩执行器沿用 SDK。源资产保持原样。

生成器另外读取 Follower URDF 中可选的 `camera_link` / `camera_joint` 定义，
在 Host 模型中启用相机：固定安装到 `link6`，采用源安装平移和 RPY 姿态、
质量与完整惯性张量，复用 SDK 中相机和支架的两个已转换 OBJ 及材质配色。
相机碰撞体也做 CoACD 分解，启用地面和非相邻连杆接触；与腕部固定装配体的接触沿用焊接体和父子体过滤。
这里启用的是相机实体，不创建图像传感器或连接相机驱动。

Host 为左右夹指添加 slide 关节，沿用源 URDF 的 ±41 mm 行程，并以 equality 约束保证对称运动。两个线性执行器由位置 PD 和偏置补偿驱动，仿真力限幅为每侧 5 N；这是展示参数，不是夹爪电机额定力矩或经标定的夹持力。控制输入的电机角度通过测量表换算为总开口，再换算为每侧位移，未建模真实传动机构的电机转动惯量。

为达到确认的 0-mm 闭合端点，左右夹指中三处会提前重叠的近似碰撞分片采用 1 mm 内缩（L:14、28；R:20）。其余夹爪碰撞分片保持原配置，未关闭两侧夹指或对外的接触检测。

`collision/*.obj` 是离线生成的凸碰撞分片，运行网页不需要 Python 或 CoACD。
`contact/*.obj` 是 link2 / link5 用于运行的内缩分片；`collision/` 同时保留这两者的原始分解缓存，供离线重建使用。网页只请求 XML 引用的网格。
XML 中的 `meshes/obj/` 由网页加载器从 SDK 模型路径映射到 MuJoCo 虚拟文件系统。
因此此 XML 不是可直接从当前目录用原生查看器打开的独立模型包。

## 接触配置

- 隐藏碰撞体 `contype=2, conaffinity=3`，启用自身和地面接触；外观网格不参与接触。
- 排除六组直接相邻的关节装配体，固定连接零件的接触由 MuJoCo 的焊接体过滤处理。
- 不排除 link2 / link5 等非相邻连杆接触。
- link2 / link5 的凸体各向内偏移 0.4 mm，避免细分重网格在零位的表面噪声产生假接触；内切半径不足 0.4 mm 的薄碎片不保留。此为近似碰撞几何的建模容差，不改变视觉、惯性或关节限位。
- 接触 margin 为每个碰撞体 0.05 mm；没有通过扩大排除名单或关闭零位附近检测来绕过碰撞。
- 1 ms 物理步长；保留 PD、偏置力补偿及原执行器力矩限幅。
- 接触存在约束柔性，接触处会有数值误差，不能作为真机安全验证依据。

碰撞近似不是 CAD 表面的逐三角形精确检测。大部分组件采用 CoACD 凸分解；
link2 和 link5 为保护零位的狭窄装配间隙，先拆分不连通 CAD 零件，
再对产生间隙误判的电机/支架零件做细分和上述内缩。近零体积的退化碎面不生成碰撞体。
分解数量有上限，不能将分解阈值当作全模型精度保证。

## 离线重新生成

安装 Python 工具（与网页运行环境独立）：

```bash
python3 -m pip install --target .collision-tools --no-deps \
  trimesh==4.5.3 coacd==1.0.14 networkx==3.2.1
OMP_NUM_THREADS=4 PYTHONPATH=.collision-tools python3 scripts/build_collision_model.py
```

上述命令还需要环境已有 numpy、scipy 和 mujoco。
`collision-manifest.json` 保存输入网格及参数的 SHA-256 和输出列表；
重新执行会复用已生成的分片，`--rebuild` 强制重建。
更换 SDK 模型后应重新生成，并重新验证零位、自由运动、接触和退出接触。

验证命令：`python3 scripts/check_collision.py` 和 `npm test`。原生检查逐步记录接触力及穿透；网页测试还包含仅关闭自碰撞、保留地面碰撞的对照实验。测试覆盖指定的 J4 折叠姿态，不能据此推断全姿态空间都已得到验证。

技术参考：

- [CoACD](https://github.com/SarahWeiii/CoACD)：凸分解工具。
- [reBot RS 控制器](https://github.com/Seeed-Projects/reBot_RS_Sim/blob/main/web_mujoco/src/pd-control.js)：参考其 PD、偏置补偿和力矩限幅思路，未复制模型或控制器代码。
- [MuJoCo 接触模型](https://mujoco.readthedocs.io/en/stable/computation/index.html#soft-contact-model)。
