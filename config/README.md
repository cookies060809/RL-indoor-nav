# 配置入口

本目录对应方案中的 config。split_empty32、split_single32、split_turn32、split_mixed32 是推荐的阶段配置；windows_recommended32 使用混合任务、4 子进程和 2 个 PyTorch 线程。

base、empty32 等保留展平 MLP 基线。formal48 为 48 格配置，不能直接续接 32 格模型。已交付模型以各自 models/<model_id>/config.yaml 为准，不因修改本目录而自动改变已训练模型。

requirements-core-lock.txt 用于 Linux 核心依赖，先从官方 CPU 源单独安装 torch 2.8。requirements-win-lock.txt 仅用于 Windows 重建。
