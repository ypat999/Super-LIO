# **Super-LIO 漂移问题：独立技术分析**

先给出一个核心判断：**目前的数据已经足以把问题从“普通的 LIO 退化”收敛到“LiDAR 错误观测持续注入姿态状态，而滤波器缺少跨帧一致性约束和错误状态恢复机制”。**

我不建议继续把主要精力放在单纯调 `deg_cond`、`n_effect` 或 `lost_slew` 上。它们分别解决观测质量判断和异常后的状态处理，但没有解决两个更根本的问题：

1. **如何判定一个连续多帧、每帧看起来都合理的错误姿态修正，实际上正在把系统带离正确轨迹？**
2. **判定错误后，如何避免错误状态继续污染地图，并在观测恢复时重新找到可信状态？**

从你给出的实验看，`净姿态修正速率 + LOST 观测有效性门控` 是目前最值得继续验证的方向。但它应当成为一个完整的状态管理机制的入口，而不是最终方案。

下面按五个问题展开。

## **0. 先修正一个测量层面的细节：SO(3) 修正量**

在讨论阈值之前，我认为有必要先检查你现在的净速率实现。

目前使用：

\\[ \Delta r_k=rv_k-rv\_{pred,k} \\]

这个量在修正角度较小时可以作为近似，但它不是严格的 SO(3) 相对旋转。原因是旋转向量的坐标表示并不满足普通向量差的线性运算性质。

建议直接计算：

\\[ \Delta R_k=R\_{pred,k}^{-1}R\_{post,k} \\]

\\[ \delta\theta_k=\operatorname{Log}(\Delta R_k) \\]

这里的 `R_post` 是 LiDAR 观测更新后的姿态，`R_pred` 是更新前的 IMU 预测姿态。

如果需要将不同帧的修正量累加到同一个坐标系，可以使用世界坐标系下的左扰动：

\\[ \delta\theta_k^W = \operatorname{Log}(R\_{post,k}R\_{pred,k}^{-1}) \\]

之后计算：

\\[ \omega\_{corr}(t)= \frac{ \left\\|\sum\_{k\in W}\delta\theta_k^W\right\\| }{ \sum\_{k\in W}\Delta t_k } \\]

对于旋转较小的连续修正，这是合理的近似；更严格的处理是把各帧旋转增量在 SO(3) 上按顺序复合，再取 Log。

这里还有一个值得记录的辅助指标：

\\[ C\_\theta= \frac{\left\\|\sum_k\delta\theta_k^W\right\\|} {\sum_k\\|\delta\theta_k^W\\|+\epsilon} \\]

它表示方向一致性，范围为 0～1。

- 健康状态下，姿态修正可能较大，但是方向经常改变，累计后相互抵消。
- 错误拖拽状态下，姿态修正持续朝同一个方向，累计量接近各帧修正量绝对值之和。

这两个指标组合比单独使用净速率更可靠。

**建议先确认&#x20;**`exp_net3`**&#x20;当前使用的是普通旋转向量差，还是已经转换为 SO(3) 相对旋转。** 如果还是前者，我建议把它作为第一项修正。特别是进入大姿态偏差阶段后，两种计算方式可能出现明显差异。

## **1. 如何在拦住漂移后恢复正确轨迹？**

这是我认为整个问题中最重要的部分。

### **1.1 目前的 LOST 处理存在一个架构缺口**

你现在的逻辑大致是：

```
IMU预测
   ↓
LiDAR配准
   ↓
trust检测
   ↓
接受或拒绝
   ↓
LOST时进行slew/damp
```

问题是：系统没有明确区分以下三件事情：

1. 当前传感器观测是否可靠。
2. 当前状态估计是否可靠。
3. 当前地图是否仍然可信。

这三个事情实际上是独立的。

例如，假设在 272s：

- LiDAR 开始错误配准。
- 272～273s，姿态已经开始缓慢偏移。
- 地图继续使用错误位姿插入点云。
- 273s，trust 检测发现异常，进入 LOST。

此时即使立即冻结速度，地图也可能已经包含了一部分错误结构。

更麻烦的是，后续几何环境恢复正常时，新的扫描点云实际上是在与一个被污染的地图进行配准。

所以简单地退出 LOST，重新开始接受 LiDAR 观测，并不能保证恢复。

### **1.2 建议引入地图可信度隔离**

我建议把地图分成两个逻辑层：

## **地图管理架构**

**Trusted Submap（可信地图）**

已通过稳定性验证的关键帧和局部地图。只读，作为定位和重定位的参考。

通过可信度验证后合并

**Active Submap（活动地图）**

当前正常运行中的局部地图。允许更新，但保留回滚能力。

**Quarantine（隔离区）**

异常期间产生的扫描和位姿，不参与地图更新，不污染可信地图。

关键不是一定要实现完整的多地图 SLAM，而是首先做到：

**任何被 trust 判为可疑的姿态，都不能继续向可信地图写入点云。**

如果现在的 IVox 支持地图版本快照，最好保留周期性 checkpoint。否则也可以先通过关键帧构建可重建的局部地图。

我建议至少保存：

- 每个可信关键帧的位姿。
- 对应的原始点云。
- IMU 状态和零偏。
- 对应时间段的 IMU 数据。
- 地图版本及关键帧插入时间。

这样可以在异常后丢弃问题时间段产生的地图数据。

### **1.3 恢复不能只靠重新接受 LiDAR**

我更倾向于增加一个独立的 RELOCALIZING 状态。

整个状态机可以是：

\#chatgpt-mermaid-\_r_1sp\_{font-family:-apple-system-body,ui-sans-serif,-apple-system,system-ui,"Segoe UI",Helvetica,"Apple Color Emoji",Arial,sans-serif,"Segoe UI Emoji","Segoe UI Symbol";font-size:16px;fill:rgb(13, 13, 13);}@keyframes edge-animation-frame{from{stroke-dashoffset:0;}}@keyframes dash{to{stroke-dashoffset:0;}}#chatgpt-mermaid-\_r_1sp\_ .edge-animation-slow{stroke-dasharray:9,5!important;stroke-dashoffset:900;animation:dash 50s linear infinite;stroke-linecap:round;}#chatgpt-mermaid-\_r_1sp\_ .edge-animation-fast{stroke-dasharray:9,5!important;stroke-dashoffset:900;animation:dash 20s linear infinite;stroke-linecap:round;}#chatgpt-mermaid-\_r_1sp\_ .error-icon{fill:rgb(243, 243, 243);}#chatgpt-mermaid-\_r_1sp\_ .error-text{fill:rgb(13, 13, 13);stroke:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .edge-thickness-normal{stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .edge-thickness-thick{stroke-width:3.5px;}#chatgpt-mermaid-\_r_1sp\_ .edge-pattern-solid{stroke-dasharray:0;}#chatgpt-mermaid-\_r_1sp\_ .edge-thickness-invisible{stroke-width:0;fill:none;}#chatgpt-mermaid-\_r_1sp\_ .edge-pattern-dashed{stroke-dasharray:3;}#chatgpt-mermaid-\_r_1sp\_ .edge-pattern-dotted{stroke-dasharray:2;}#chatgpt-mermaid-\_r_1sp\_ .marker{fill:rgb(143, 143, 143);stroke:rgb(143, 143, 143);}#chatgpt-mermaid-\_r_1sp\_ .marker.cross{stroke:rgb(143, 143, 143);}#chatgpt-mermaid-\_r_1sp\_ svg{font-family:-apple-system-body,ui-sans-serif,-apple-system,system-ui,"Segoe UI",Helvetica,"Apple Color Emoji",Arial,sans-serif,"Segoe UI Emoji","Segoe UI Symbol";font-size:16px;}#chatgpt-mermaid-\_r_1sp\_ p{margin:0;}#chatgpt-mermaid-\_r_1sp\_ .label{font-family:-apple-system-body,ui-sans-serif,-apple-system,system-ui,"Segoe UI",Helvetica,"Apple Color Emoji",Arial,sans-serif,"Segoe UI Emoji","Segoe UI Symbol";color:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .cluster-label text{fill:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .cluster-label span{color:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .cluster-label span p{background-color:transparent;}#chatgpt-mermaid-\_r_1sp\_ .label text,#chatgpt-mermaid-\_r_1sp\_ span{fill:rgb(13, 13, 13);color:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .node rect,#chatgpt-mermaid-\_r_1sp\_ .node circle,#chatgpt-mermaid-\_r_1sp\_ .node ellipse,#chatgpt-mermaid-\_r_1sp\_ .node polygon,#chatgpt-mermaid-\_r_1sp\_ .node path{fill:rgb(222, 234, 251);stroke:rgb(83, 154, 248);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .rough-node .label text,#chatgpt-mermaid-\_r_1sp\_ .node .label text,#chatgpt-mermaid-\_r_1sp\_ .image-shape .label,#chatgpt-mermaid-\_r_1sp\_ .icon-shape .label{text-anchor:middle;}#chatgpt-mermaid-\_r_1sp\_ .node .katex path{fill:#000;stroke:#000;stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .rough-node .label,#chatgpt-mermaid-\_r_1sp\_ .node .label,#chatgpt-mermaid-\_r_1sp\_ .image-shape .label,#chatgpt-mermaid-\_r_1sp\_ .icon-shape .label{text-align:center;}#chatgpt-mermaid-\_r_1sp\_ .node.clickable{cursor:pointer;}#chatgpt-mermaid-\_r_1sp\_ .root .anchor path{fill:rgb(143, 143, 143)!important;stroke-width:0;stroke:rgb(143, 143, 143);}#chatgpt-mermaid-\_r_1sp\_ .arrowheadPath{fill:rgb(143, 143, 143);}#chatgpt-mermaid-\_r_1sp\_ .edgePath .path{stroke:rgb(143, 143, 143);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .flowchart-link{stroke:rgb(143, 143, 143);fill:none;}#chatgpt-mermaid-\_r_1sp\_ .edgeLabel{background-color:rgb(252, 252, 252);text-align:center;}#chatgpt-mermaid-\_r_1sp\_ .edgeLabel p{background-color:rgb(252, 252, 252);}#chatgpt-mermaid-\_r_1sp\_ .edgeLabel rect{opacity:0.5;background-color:rgb(252, 252, 252);fill:rgb(252, 252, 252);}#chatgpt-mermaid-\_r_1sp\_ .labelBkg{background-color:rgba(252, 252, 252, 0.5);}#chatgpt-mermaid-\_r_1sp\_ .cluster rect{fill:rgb(243, 243, 243);stroke:rgba(0, 0, 0, 0.1);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .cluster text{fill:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ .cluster span{color:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ div.mermaidTooltip{position:absolute;text-align:center;max-width:200px;padding:2px;font-family:-apple-system-body,ui-sans-serif,-apple-system,system-ui,"Segoe UI",Helvetica,"Apple Color Emoji",Arial,sans-serif,"Segoe UI Emoji","Segoe UI Symbol";font-size:12px;background:rgb(243, 243, 243);border:1px solid rgba(0, 0, 0, 0.1);border-radius:2px;pointer-events:none;z-index:100;}#chatgpt-mermaid-\_r_1sp\_ .flowchartTitleText{text-anchor:middle;font-size:18px;fill:rgb(13, 13, 13);}#chatgpt-mermaid-\_r_1sp\_ rect.text{fill:none;stroke-width:0;}#chatgpt-mermaid-\_r_1sp\_ .icon-shape,#chatgpt-mermaid-\_r_1sp\_ .image-shape{background-color:rgb(252, 252, 252);text-align:center;}#chatgpt-mermaid-\_r_1sp\_ .icon-shape p,#chatgpt-mermaid-\_r_1sp\_ .image-shape p{background-color:rgb(252, 252, 252);padding:2px;}#chatgpt-mermaid-\_r_1sp\_ .icon-shape .label rect,#chatgpt-mermaid-\_r_1sp\_ .image-shape .label rect{opacity:0.5;background-color:rgb(252, 252, 252);fill:rgb(252, 252, 252);}#chatgpt-mermaid-\_r_1sp\_ .label-icon{display:inline-block;height:1em;overflow:visible;vertical-align:-0.125em;}#chatgpt-mermaid-\_r_1sp\_ .node .label-icon path{fill:currentColor;stroke:revert;stroke-width:revert;}#chatgpt-mermaid-\_r_1sp\_ .node .neo-node{stroke:rgb(83, 154, 248);}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node rect,#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].cluster rect,#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node polygon{stroke:url(#chatgpt-mermaid-\_r_1sp\_-gradient);filter:drop-shadow( 1px 2px 2px rgba(185,185,185,1));}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].swimlane.cluster rect{filter:none;}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node path{stroke:url(#chatgpt-mermaid-\_r_1sp\_-gradient);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node .outer-path{filter:drop-shadow( 1px 2px 2px rgba(185,185,185,1));}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node .neo-line path{stroke:rgb(83, 154, 248);filter:none;}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node circle{stroke:url(#chatgpt-mermaid-\_r_1sp\_-gradient);filter:drop-shadow( 1px 2px 2px rgba(185,185,185,1));}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].node circle .state-start{fill:#000000;}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].icon-shape .icon{fill:url(#chatgpt-mermaid-\_r_1sp\_-gradient);filter:drop-shadow( 1px 2px 2px rgba(185,185,185,1));}#chatgpt-mermaid-\_r_1sp\_ [data-look="neo"].icon-shape .icon-neo path{stroke:url(#chatgpt-mermaid-\_r_1sp\_-gradient);filter:drop-shadow( 1px 2px 2px rgba(185,185,185,1));}#chatgpt-mermaid-\_r_1sp\_ .node text{font-size:14px;font-weight:600;letter-spacing:normal;fill:rgb(0, 79, 153);}#chatgpt-mermaid-\_r_1sp\_ .edgeLabels text{font-size:13px;font-weight:600;letter-spacing:-0.08px;fill:rgb(0, 79, 153);}#chatgpt-mermaid-\_r_1sp\_ .node tspan[font-weight="normal"],#chatgpt-mermaid-\_r_1sp\_ .edgeLabels tspan[font-weight="normal"]{font-weight:600;}#chatgpt-mermaid-\_r_1sp\_ .edgeLabel .label rect{opacity:1;rx:13px;ry:13px;fill:rgb(245, 250, 255);stroke:rgb(206, 219, 229);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .node rect,#chatgpt-mermaid-\_r_1sp\_ .node circle,#chatgpt-mermaid-\_r_1sp\_ .node ellipse,#chatgpt-mermaid-\_r_1sp\_ .node polygon,#chatgpt-mermaid-\_r_1sp\_ .node path{fill:rgb(229, 243, 255);stroke:rgba(0, 0, 0, 0.1);stroke-width:1px;}#chatgpt-mermaid-\_r_1sp\_ .node rect{rx:16px;ry:16px;}#chatgpt-mermaid-\_r_1sp\_ .mermaid-decision .label-container{fill:rgb(245, 250, 255);stroke:rgb(206, 219, 229);stroke-dasharray:2,2;}#chatgpt-mermaid-\_r_1sp\_ .edgePaths .flowchart-link{stroke:rgb(143, 143, 143);stroke-width:1px;stroke-linecap:round;stroke-linejoin:round;}#chatgpt-mermaid-\_r_1sp\_ .marker{fill:rgb(143, 143, 143);stroke:rgb(143, 143, 143);}#chatgpt-mermaid-\_r_1sp\_ :root{--mermaid-font-family:-apple-system-body,ui-sans-serif,-apple-system,system-ui,"Segoe UI",Helvetica,"Apple Color Emoji",Arial,sans-serif,"Segoe UI Emoji","Segoe UI Symbol";}NORMAL正常建图跨帧信任检测SUSPECT暂停地图写入LOST拒绝LiDAR更新RELOCALIZING可信子地图多假设匹配RECOVERED恢复状态创建新活动地图/重新接入通过可疑恢复稳定确认失效匹配不通过连续多帧验证通过

其中：

**SUSPECT：**

- 停止地图写入。
- 逐步降低或拒绝可疑 LiDAR 观测。
- 继续 IMU 状态传播。
- 保留当前状态，等待下一帧验证。

**LOST：**

- 完全拒绝未通过检验的 LiDAR 位姿更新。
- 不更新地图。
- 允许 IMU 继续传播，但增大状态不确定度。
- 对外报告定位质量下降。

**RELOCALIZING：**

用此前的 Trusted Submap 进行多假设匹配。

例如：

1. 当前 IMU 预测位置附近生成候选位姿。
2. 从最近的多个可信关键帧生成其他候选。
3. 分别执行局部 ICP。
4. 比较各候选的残差、有效点分布、几何约束和 IMU 一致性。
5. 要求连续多帧得到稳定、唯一的匹配结果。

只有连续几帧都验证通过，才恢复建图。

这里有一点需要强调：不能简单地把滤波器位置恢复成 272s 之前的 checkpoint，因为设备已经实际移动了。checkpoint 的作用是恢复可信地图和先验，再通过当前扫描重新确定当前位姿。

### **1.4 是否需要重置 IMU 预积分？**

我的判断是：**需要考虑重置或重新初始化滤波器中的受污染状态，但不能简单地清空 IMU 预积分。**

需要区分两种架构：

- 如果是 ESKF：重定位后应当重新设置姿态、位置、速度及其协方差，同时审查陀螺和加计零偏。
- 如果是基于关键帧的因子图：可以保留 IMU 原始测量，重新优化关键帧状态和 IMU 因子。

尤其是你这里已经出现 `bgz≈0.09 rad/s`，虽然它不是水平假加速度的直接来源，但这个估计值已经相当大，不能假定恢复姿态后零偏仍然可信。

不能只修正姿态，不处理已经被污染的速度和零偏。

**我建议先不实现复杂的多假设 ICP，而是先做一个非常有价值的验证：保存 265s 左右的可信地图快照，在 272s 后始终对这份冻结地图进行定位。**

这一个实验就能回答地图污染是不是漂移持续扩大的重要原因。

## **2. 姿态修正速率门：固定阈值还是自适应？**

### **2.1 不建议直接使用 IMU 角速度作为阈值基准**

你提出的：

\\[ \omega\_{corr}>K\\|\omega\_{IMU}\\| \\]

乍看比较合理，但实际上存在比较严重的问题。

LiDAR 修正量和 IMU 角速度不是同一类物理量：

- IMU 角速度描述设备实际旋转速度。
- LiDAR 姿态修正量描述观测对预测姿态的修正程度。

例如：

设备静止时：

\\[ \\|\omega\_{IMU}\\|\approx 0 \\]

但是 LiDAR 仍然可能需要修正 0.5°～1° 的姿态误差。

这样相对阈值会在设备静止时变得极其严格。

反过来，设备快速旋转时，较大的姿态修正也未必意味着错误。

所以 IMU 角速度可以参与判断，但不能直接作为修正速率的分母。

### **2.2 更合适的是基于滤波器协方差的自适应门控**

ESKF 本身就具有预测协方差。

可以计算姿态创新：

\\[ \delta\theta_k = \operatorname{Log}(R\_{pred}^{-1}R\_{post}) \\]

然后计算其归一化创新：

\\[ NIS\_\theta = \delta\theta_k^T S\_\theta^{-1}\delta\theta_k \\]

其中：

\\[ S\_\theta = H\_\theta P\_{pred}H\_\theta^T+R\_\theta \\]

这里：

- \\(P\_{pred}\\)：预测状态协方差。
- \\(H\_\theta\\)：姿态观测对应的观测矩阵。
- \\(R\_\theta\\)：LiDAR 姿态观测噪声协方差。

它的意义是：

不是判断 LiDAR 修正了多少度，而是判断这次修正相对于滤波器原本认为的误差范围，有没有异常。

对于三维姿态创新，可以按照 3 自由度卡方分布设置统计门限。实际实现时需要保证协方差具有合理的标定，尤其不能让退化几何下的 \\(R\_\theta\\) 虚假地保持很小。

### **2.3 建议采用三个指标联合判断**

我建议当前阶段保留 2s 净速率窗口，但增加两个指标。

| 指标      | 作用               | 初步建议      |
| ------- | ---------------- | --------- |
| 净姿态修正速率 | 检测持续拖拽           | 3°/s 候选阈值 |
| 方向一致性   | 排除来回摆动的正常修正      | 0.7～0.8   |
| 姿态 NIS  | 判断观测创新是否超出预测不确定度 | 统计门限      |

这里的 0.7～0.8 和 3°/s 都是建议的初始实验参数，不是经过多场景验证的通用值。

触发逻辑可以先设计为：

```
bool suspicious =
    net_rot_rate > rot_rate_threshold &&
    direction_coherence > coherence_threshold;
```

然后把 NIS 作为额外的拒绝条件和置信度依据，而不是强制要求每一帧 NIS 都异常。毕竟渐进式拖拽也可能每帧都产生看起来不大的创新。

另一个很有价值的方向是使用 CUSUM 累积统计量，检测小幅但长期同向的创新：

\\[ S_k=\max(0,S\_{k-1}+z_k-\nu) \\]

当 \\(S_k\\) 超过阈值时判定存在持续系统性偏差。这里的 \\(z_k\\) 可以是沿固定坐标系中的修正方向投影后的标准化创新。

它比简单累计绝对值更适合检测持续同向的偏差。

### **2.4 3°/s 是否太严格？**

根据你提供的数据：

- 健康段 p90：1.9°/s。
- 健康段 max：4.67°/s。
- 漂移段 mean：5.65°/s。
- 漂移段 p50：6.59°/s。

所以 3°/s 确实有一定区分能力。

但要注意：健康段 max 仍然超过 3°/s。

因此不能仅凭超过 3°/s 就进入 LOST。

我更建议：

- 超过阈值：进入 SUSPECT。
- 超过阈值并持续一定时间，同时方向高度一致：拒绝 LiDAR 更新。
- 几何约束明显下降：进一步提高异常置信度。
- 观测重新稳定：先验证，不立即恢复地图写入。

**我的建议是：保留 3°/s 作为初期 SUSPECT 阈值，不要把它直接作为永久性的硬拒绝阈值。**

## **3. 为什么 272s 突然开始拖姿态？**

我认为至少有三个竞争性假设，需要通过实验区分。

### **假设 A：地图累积误差达到临界点**

这是我目前比较重视的一个方向。

LIO 的局部地图实际上存在反馈：

```
当前位姿
  ↓
将扫描点云插入地图
  ↓
地图几何发生变化
  ↓
下一帧匹配当前地图
  ↓
新的位姿估计
  ↓
继续更新地图
```

假设前面已经存在一些小误差：

- 早期误差不足以影响匹配。
- 地图逐渐出现轻微的平面弯曲、厚度增加或局部重影。
- 到达某个特定时间后，当前扫描与地图之间出现新的局部最优解。
- 后续帧开始沿着错误方向持续配准。
- 地图继续被错误姿态更新，形成正反馈。

这能解释为什么不是某一帧突然跳变，而是持续、同方向地拖动。

**验证方法：冻结地图。**

做两组实验：

| 实验 | 地图策略                  |
| -- | --------------------- |
| A  | 正常实时更新地图              |
| B  | 265s 后停止地图写入，只允许匹配旧地图 |

如果 B 明显好于 A，基本可以确认地图污染是重要放大器。

还可以做第三组：从 265s 的地图快照出发，重新生成局部地图，但使用不同的位姿先验。

### **假设 B：进入了特定几何退化结构**

这与你描述的长走廊、楼梯环境相符。

但是我不建议只用点云包围盒的细长比判断。

例如：

点云包围盒虽然很长，但如果周围有足够丰富的墙面、地面、顶面和角点，仍然可以提供良好的六自由度约束。

真正有价值的是点云对位姿的约束分布。

对于点到平面的配准：

\\[ r_i=n_i^T(Rp_i+t-q_i) \\]

对应的线性化 Jacobian 可以写成：

\\[ J_i= \begin{bmatrix} n_i^T & (Rp_i\times n_i)^T \end{bmatrix} \\]

具体正负号取决于扰动定义。

那么：

\\[ H=\sum_i w_iJ_i^TJ_i \\]

这个矩阵才更直接地反映不同位姿自由度的约束强度。

建议增加：

1. 六维 Hessian 的完整特征值及特征向量。
2. 六个自由度对应的信息量。
3. 法向量分布的协方差矩阵。
4. 点云空间分布的有效覆盖范围。
5. 各空间区域的有效匹配点数量。

尤其建议把当前 `cond` 的计算方式重新审查一下。

如果 Hessian 直接混合米和弧度，条件数会受到单位和尺度影响。最好对平移和旋转列进行归一化，再计算特征值和条件数。

例如以特征尺度 \\(L\\) 将旋转扰动转换为等效位移：

\\[ \delta x = [\delta t,\ L\delta\theta] \\]

这样不同场景的退化程度更有可比性。

有一个值得参考的研究方向：GenZ-ICP 并不是单纯依赖一种距离误差，而是结合点到平面和点到点约束，根据环境几何调整权重；AdaLIO 则专门研究走廊、螺旋楼梯等狭窄场景的参数适应。

[image](https://www.google.com/s2/favicons?domain=https://ieeexplore.ieee.org\&sz=32)

IEEE Xplore+1



另外，2026 年发表的 OCE 研究也指出，单纯依靠静态 Hessian 特征值容易在部分退化、但实际上仍可收敛的场景中出现误判，动态观察优化过程的收敛情况也是值得考虑的方向。

[image](https://www.google.com/s2/favicons?domain=https://link.springer.com\&sz=32)

Springer Link



### **假设 C：时间同步、点云去畸变或传感器运动状态发生变化**

这个方向不能因为目前已经观察到几何退化就排除。

建议重点检查：

- 272s 前后 `frame_dt` 是否存在跳变。
- 每个 CustomMsg 的点时间戳是否连续。
- IMU 插值是否覆盖完整的 LiDAR 扫描周期。
- 扫描内的时间偏移是否正确。
- LiDAR 和 IMU 是否存在实际时间偏差。
- 是否在特定运动状态下去畸变效果突然恶化。

尤其是 Livox 的非重复扫描模式，如果点云的采样时序与设备运动发生组合变化，错误的去畸变可以制造出看似合理、但实际上具有系统性偏差的几何结构。

### **我建议用一个简单的时间相关性实验决定主因**

把以下数据放到一张 265～280s 的共用时间轴：

- `n_effect`
- `fit_mean / fit_p95`
- 归一化 Hessian 最小特征值
- `net_rot_rate`
- `C_theta`
- `frame_dt`
- IMU 去畸变相关时间偏差
- 当前地图插入状态

观察谁最先变化。

我的预期是：

- 如果几何指标先恶化，说明更可能是环境退化或观测质量下降。
- 如果姿态修正先开始一致偏移，而几何指标随后才恶化，说明配准错误可能先出现，地图随后被污染。
- 如果冻结地图后恢复，说明地图自反馈是重要因素。
- 如果冻结地图后仍然漂移，则重点检查扫描数据、对应关系和去畸变。

这个判断比继续尝试不同 `deg_cond` 数字更有价值。

## **4. Huber 收紧为什么有效，但不能根治？**

我认为这组实验是目前最重要的证据之一。

Huber 从 0.20 收紧到 0.05 后：

- 漂移触发时间从 277.4s 推迟到 303s。
- 300s 时位置仍然正确。
- 有效点数量保持在约 130。

这说明问题不仅仅是几何退化，更存在**错误对应关系对优化结果的持续影响**。

### **4.1 Huber 的局限**

Huber 核的权重为：

\\[ w(r)= \begin{cases} 1,& |r|\leq\delta\\\ \frac{\delta}{|r|},& |r|>\delta \end{cases} \\]

当残差超过阈值后，权重会下降，但不会归零。

换句话说，即使是较大的离群点，只要数量足够多、方向足够一致，仍然可能对最终姿态产生明显影响。

更加麻烦的是：

假设错误的对应关系实际上产生了比较小的残差，例如 2～4cm。

那么无论 Huber 阈值设置得多严格，只要这些点仍然落在内点区间，它们就会得到完整权重。

这可能正是为什么 Huber 0.05 能推迟漂移，却不能完全消除漂移。

### **4.2 建议尝试自适应鲁棒核尺度**

我不建议一直固定 `delta=0.05`。

可以对每帧点到平面的残差进行鲁棒尺度估计：

\\[ \sigma_r=1.4826\cdot \operatorname{median}(|r_i-\operatorname{median}(r)|) \\]

这是基于 MAD 的尺度估计。

然后：

\\[ \delta=c\sigma_r \\]

其中 Huber 可以先取 \\(c=1.345\\) 作为初始实验值，再通过数据验证调整。

最好对不同距离、不同入射角的点分别估计测量噪声，而不是假定所有点的残差噪声相同。

还可以考虑：

| 鲁棒策略      | 特点          | 我的评价      |
| --------- | ----------- | --------- |
| Huber     | 大残差降权，但仍有影响 | 保留作为基准    |
| Cauchy    | 大残差进一步降权    | 值得尝试      |
| Tukey     | 超过一定范围后权重归零 | 适合排除明显离群点 |
| 自适应 Huber | 随噪声尺度调整阈值   | 推荐        |
| 自适应 Tukey | 动态尺度与截断相结合  | 后续尝试      |

我更倾向于先实现自适应 Huber，再实现 Tukey。

原因是 Tukey 的红降权特性虽然可以彻底排除离群点，但在点云本来就稀疏的环境中，如果尺度估计不正确，容易把有效点一起删掉。

### **4.3 还需要增加有效点的空间分布指标**

这里有一个很容易忽视的情况：

假设一帧有 180 个有效平面匹配点。

这 180 个点可能：

- 均匀分布在四周；
- 全部集中在左侧墙面；
- 主要集中在一条地面边缘；
- 来自同一个局部平面。

这四种情况下的配准能力完全不同。

因此建议增加：

\\[ N\_{\text{eff}}= \frac{(\sum_i w_i)^2}{\sum_i w_i^2} \\]

它代表权重意义下的有效样本量。

除此以外，把空间划分为多个区域，记录每个区域的有效点数量及法向分布。

这样可以发现：

**有效点数量看起来正常，但实际上全部来自同一种几何约束。**

这比单纯增加 `lost_min_effect_pts` 更有意义。

### **4.4 鲁棒核无法识别所有自洽错误**

这是最重要的限制。

如果扫描与地图之间出现了错误但高度一致的几何对应关系，残差可能仍然很小。

此时：

- Huber 不一定认为这些点是离群点；
- Tukey 也不一定能把它们排除；
- 有效点数量甚至可能看起来很好。

所以鲁棒核主要解决的是残差污染，不是错误局部最优解。

我认为最终方案必须结合：

**鲁棒残差 + 几何可观测性 + IMU 先验一致性 + 跨帧姿态修正趋势。**

四者不能互相替代。

## **5. 速度是否需要独立约束？**

我的答案是：需要，但必须分清它是安全保护，还是状态估计修正。

你目前的失效链条已经很明确：

\\[ \text{姿态错误} \rightarrow \text{重力补偿错误} \rightarrow \text{假加速度} \rightarrow \text{速度积分发散} \\]

这意味着速度是最后一个可以被系统监控的异常量。

但是，直接对速度做硬限幅并不是理想方案。

### **5.1 为什么简单限速存在风险？**

例如：

```
if (state.vel.norm() > max_vel) {
    state.vel *= max_vel / state.vel.norm();
}
```

这样虽然可以避免数百米每秒的发散，但有三个问题：

1. 真实高速运动也可能被误判。
2. 速度被直接修改后，滤波器的协方差不再反映真实状态。
3. 位置仍然会受到后续加速度积分影响，错误姿态并没有修复。

尤其是如果系统还在正常插入地图，限速只会让错误地图生成得慢一点。

### **5.2 我建议建立独立的速度异常监控器**

首先计算 IMU 预测的速度增量：

\\[ \Delta v\_{IMU}= \int\_{t_k}^{t\_{k+1}} \left[ R(t)(a_m(t)-b_a)+g \right]dt \\]

再比较滤波器速度变化：

\\[ e_v= (v\_{k+1}-v_k)-\Delta v\_{IMU} \\]

这可以帮助识别 LiDAR 更新是否通过状态协方差耦合，向速度注入了异常修正。

同时记录：

- 每帧 LiDAR 更新造成的速度修正量。
- IMU 预测的速度增量。
- 速度模长。
- 速度变化率。
- 位置增量与速度积分的一致性。

不过这里必须注意：两者都来自同一个滤波系统时，并不构成完全独立的真值验证。特别是姿态已经错误时，IMU 积分使用的重力投影本身就不可靠。

因此还需要结合平台的运动学上限，以及可用的独立传感器约束，例如光流、视觉里程计、测距或其他定位源。

### **5.3 推荐分成两级保护**

**第一级：估计器内部保护**

当检测到不合理的速度增量时：

- 拒绝当前可疑 LiDAR 更新。
- 禁止更新地图。
- 标记状态为 SUSPECT 或 LOST。
- 增大状态不确定度。
- 保留 IMU 原始数据供重新估计。

这里不建议直接把速度清零。

**第二级：飞控安全保护**

当速度超过平台物理上限，或者定位状态已经不可信时：

- 向上层发布定位不可用。
- 禁止继续按照错误位置执行轨迹。
- 由飞控安全策略决定悬停、降落或其他安全动作。

这两个机制应该分离。

对于你的应用，尤其是 GPS 不可用的室内无人机，不能把一个已经不可信的 LIO 位姿继续交给 Offboard 轨迹控制器。

### **5.4 一个特别值得考虑的保护：LiDAR 速度修正预算**

你的问题不是简单的速度过大，而是 LiDAR 错误姿态更新间接造成速度变化。

因此我建议专门记录：

\\[ \Delta v\_{LIO}=v\_{post}-v\_{pred} \\]

也就是每帧 LiDAR 更新前后的速度差。

如果连续多帧出现同方向的速度修正，而且与姿态修正趋势、几何退化同时发生，就应该提高异常置信度。

这个指标比单独等待 `|v|>2m/s` 更早。

需要注意，ESKF 中 LiDAR 姿态观测引起一定速度修正是正常的，因为状态之间存在协方差耦合。不能把 \\(\Delta v\_{LIO}\\) 一概视为异常。更合理的办法是结合预测协方差、观测创新以及时间窗口进行判断。

## **6. 我建议下一阶段的开发优先级**

综合现有实验，我不会同时开发所有算法，而是分成三个阶段。

### **第一阶段：验证当前修复是否真正成立**

当前最优先

针对正在跑的两组实验：

| 验证项           | exp_net3 | exp_net3_hub |
| ------------- | -------- | ------------ |
| 净姿态修正速率       | 3°/s     | 3°/s         |
| LOST 观测有效性门控  | 开启       | 开启           |
| Huber         | 原参数      | 0.05         |
| 首次 SUSPECT 时间 | 记录       | 记录           |
| 首次 LOST 时间    | 记录       | 记录           |
| 300s 位置偏差     | 记录       | 记录           |
| vmax          | 记录       | 记录           |
| 退出 LOST 后是否复漂 | 记录       | 记录           |
| 地图污染程度        | 记录       | 记录           |

特别关注两件事：

第一，进入 LOST 后，位置是否还在持续偏移。

第二，观测恢复时，是否立即重新进入错误修正状态。

第二件事比最终 vmax 更重要。即使 vmax 只有 13m/s，只要恢复后持续漂移，系统仍然不能用于自主建图。

另外，补充记录 `obs_valid` 为 false 时：

```
position_update
velocity_update
attitude_update
map_insert
```

四个状态分别发生了什么。

不要只检查有没有通过 `lost_slew`。

### **第二阶段：定位 272s 触发的真正原因**

建议做以下三个最小实验：

**实验 A：冻结地图**

优先级最高

265s 后不再向 IVox 写入点云，但仍使用冻结地图执行配准。

目的：判断地图自污染是否是重要原因。

**实验 B：几何约束分布**

输出 Hessian 六维特征值、特征向量、有效点空间分布和法向量分布。

目的：判断是某个自由度真正失去约束，还是错误对应关系导致优化偏离。

**实验 C：SO(3) 创新与 IMU 一致性**

输出 Lie 群姿态修正量、2s 净速率、方向一致性、NIS 和 LiDAR 速度修正量。

目的：找到最早能够识别错误的指标。

我认为这三个实验足以明显缩小根因范围。

### **第三阶段：建立完整的恢复机制**

等前面结果明确后，再实现：

1. Trusted Submap 和 Active Submap 分离。
2. 异常期间冻结地图更新。
3. LOST 状态保持预测，不接受未经验证的 LiDAR 修正。
4. 多假设 ICP 重定位。
5. 连续多帧验证。
6. 重初始化受污染的滤波状态。
7. 恢复建图。

这里可以参考 OR-LIM 的思路：不仅进行鲁棒 LIO，还通过固定时域优化和地图平滑处理传感器退化、多层建筑等场景中的位姿与几何一致性。它的设计思路与你需要解决的地图污染和状态恢复问题比较接近。

[image](https://www.google.com/s2/favicons?domain=https://www.sciencedirect.com\&sz=32)

科学直接



## **7. 我认为现有根因结论还需要修正的两个地方**

最后补充两个容易影响后续设计判断的细节。

### **7.1&#x20;**`deg_dir_v≈0.97`**&#x20;不应该被当作物理因果证据**

旋转修正轴与速度方向高度一致，这是很好的异常特征，但不能据此推断二者存在物理因果关系。

旋转向量和速度向量虽然都可以表示为三维向量，但前者代表旋转轴，后者代表平移运动方向。

建议保留这个指标，用于该 bag 的异常特征识别，但不要直接将其作为通用判据。

更值得观察的是：姿态修正方向与 Hessian 弱约束特征向量之间是否存在关联。

如果修正持续沿着弱约束方向发生，就更能说明几何退化正在放大错误配准。

### **7.2 重新核实 0.7g 水平假加速度的计算口径**

如果 0.7g 确实是经过当前估计姿态转换到世界坐标系后，扣除重力得到的水平残差，那么这个证据非常有力。

纯粹由重力投影误差产生 0.7g 水平分量，对应的倾角误差约为：

\\[ \theta=\arcsin(0.7)\approx44.4^\circ \\]

而你测得姿态修正速率大约为 13°/s，累计约 3.4s 就能达到这个角度量级。

这与 272s 后持续拖拽的现象在时间尺度上是吻合的。

但是，**单独观察加速度计体轴 x、z 分量，不能直接证明存在 0.7g 的世界系假加速度。** 还需要结合姿态计算：

\\[ a_W=R(a_m-b_a)+g \\]

并确认具体坐标系、加速度计符号约定和重力方向。

另外，`bgz` 增长到 0.09 rad/s 虽然未必是主因，但已经不能完全忽略。建议记录它和姿态创新的时间先后关系，判断它究竟是姿态错误的结果，还是进一步放大预测误差的因素。

## **8. 最终技术判断**

我把目前的结论归纳成下面这张表：

| 问题        | 当前判断           | 推荐方向                  |
| --------- | -------------- | --------------------- |
| 渐进姿态拖拽    | 已有充分数据支持       | SO(3) 净修正速率、方向一致性     |
| 错误对应关系    | Huber 实验提供较强证据 | 自适应鲁棒核、空间分布检验         |
| 几何退化      | 有迹象，但尚未确定根因    | 归一化 Hessian、弱约束方向     |
| 地图自污染     | 重要待验证假设        | 冻结地图实验、地图版本隔离         |
| LOST 后漏位姿 | 已发现具体的软件逻辑问题   | obs_valid 门控          |
| 恢复后复漂     | 当前架构没有真正解决     | Trusted Submap、多假设重定位 |
| 速度发散      | 姿态错误后的放大环节     | LiDAR 速度修正监控、独立安全保护   |

**我的最终建议是，不要继续将 Super-LIO 的抗漂移能力等同于某个 trust 阈值或退化检测参数。**

真正需要建立的是三层防护：

第一层

**错误观测检测**

净姿态修正速率、方向一致性、NIS、几何可观测性、鲁棒残差。

第二层

**错误状态隔离**

拒绝可疑 LiDAR 更新、冻结地图写入、保护速度和导航输出。

第三层

**状态恢复**

可信地图回锚、多假设重定位、滤波器状态重新初始化、验证后继续建图。

在开发优先级上，我会明确选择：

**先完成&#x20;**`exp_net3`**&#x20;和&#x20;**`exp_net3_hub`**&#x20;验证，再做 265s 冻结地图实验，最后才开发重定位。**

其中，冻结地图实验尤其关键。它可以帮助判断你现在面对的究竟是扫描与地图匹配本身的错误，还是错误位姿通过 IVox 持续污染地图形成的反馈失稳。

如果后者占主导，那么地图隔离和回滚的收益可能远大于继续微调 ICP 参数。