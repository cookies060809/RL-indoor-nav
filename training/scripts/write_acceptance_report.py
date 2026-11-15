from pathlib import Path
import sys
import json
import csv
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training/envs'))
from navrl.config import ROOT

def read(path): return json.loads((ROOT/path).read_text(encoding='utf-8-sig'))
environment=read('docs/reports/environment.json')
summary=read('docs/reports/model_summary.json')
lines=['# Windows 实测验收摘要','',
       '日期：2026 年 11 月 14 日。结果来自本项目实际运行文件，全部为二维环境结果。','',
       '## 环境与验证','',
       f"Python {environment['python'].split()[0]}；torch {environment['versions']['torch']}；SB3 {environment['versions']['stable-baselines3']}；Gymnasium {environment['versions']['gymnasium']}。",'',
       '依赖检查、Gymnasium 检查、17 项自动测试通过。Windows 已验证模型包哈希与固定输入推理；未运行 Ubuntu rclpy 或 Gazebo。','',
       '## 模型记录','',
       '| 运行目录 | 阶段 | 最佳验证 | 另外生成的测试任务 | 测试碰撞 |',
       '|---|---|---|---|---|']
for row in summary:
    test=f"{row['test_successes']}/{row['test_episodes']}" if row['test_episodes'] else '未执行'
    collision=f"{row['test_collision_rate']:.1%}" if row['test_collision_rate'] is not None else '-'
    lines.append(f"| {row['run']} | {row['stage']} / {row['extractor']} | {row['validation_successes']}/{row['validation_episodes']} | {test} | {collision} |")
lines+=['','各阶段的测试任务分布不同，不能把空场景与混合场景成功率直接作为算法排名。模型均为 seed 42 的初步训练；尚未完成三种子正式结论。',
        '选中的 best_model 不一定是最后一步，准确步数及来源见 model_summary.csv 和各运行目录 metadata/config/resume 文件。','',
        '## 混合场景安全过滤初步配对诊断','']
safe=read('training/runs/split_mixed_100k_s42/validation_safe_12/summary.json')
unsafe=read('training/runs/split_mixed_100k_s42/validation_unfiltered_12/summary.json')
lines+=['| 配置 | 成功 | 碰撞 | 平均干预时长 |','|---|---|---|---|']
for name,result in [('开启过滤',safe),('关闭额外过滤',unsafe)]:
    lines.append(f"| {name} | {result['successes']}/{result['episodes']} | {round(result['collision_rate']*result['episodes'])}/{result['episodes']} | {result['mean_intervention_duration']:.2f} s |")
lines+=['','同一固定模型、同一组 12 个验证任务种子。该组数据参与过模型选择，只作为工程诊断，不替代正式 Gazebo 消融或独立测试结论。',
        '保护过滤可能以停车换取避免碰撞，不能仅凭零碰撞判断策略本身足够安全。','',
        '## 混合场景失败原因','']
with (ROOT/'training/runs/split_mixed_100k_s42/test_24/episodes.csv').open(encoding='utf-8-sig') as f:
    counts=Counter(row['reason'] for row in csv.DictReader(f))
for key,value in sorted(counts.items()): lines.append(f'- {key}：{value} 次')
lines+=['','## 性能测试','',
        '| 模式 | 环境数 | PPO 交互步每秒 | 采样步每秒 |','|---|---|---|---|']
for filename in ('docs/reports/benchmark_dummy.json','docs/reports/benchmark_subproc.json'):
    for r in read(filename): lines.append(f"| {r['vec_env']} | {r['n_envs']} | {r['ppo_fps']:.0f} | {r['sample_fps']:.0f} |")
lines+=['','此项采用展平 MLP 的 1 万步小基准，首次编译预热后测量；不含周期验证和 Gazebo。RAM 与 CPU 字段是采样值，不代表峰值或全程均值。','',
        '## 已准备的模型包','',
        '- exports/split_empty_pretrain_v1.zip：空场景预训练与接口验证。',
        '- exports/split_single_pretrain_v1.zip：单障碍预训练。',
        '- exports/split_mixed_pretrain_v1.zip：混合布局初步模型。','',
        '三者均包含模型、配置、依赖、Python 源码、观测样例和固定推理参考。Windows 自检通过不等于 Ubuntu 自检通过。','',
        '## 仍需完成','',
        '测量 Gazebo 足迹与雷达外参、C++ BEV 真实对照、Ubuntu 库环境、ROS 安全节点与唯一速度出口、路径管理、实体重置、Gazebo 闭环、正式三种子训练及 A/B/C 实验。','']
(ROOT/'docs/reports/yy_v2/legacy_acceptance_summary.md').write_text('\n'.join(lines),encoding='utf-8')
print(ROOT/'docs/reports/yy_v2/legacy_acceptance_summary.md')
