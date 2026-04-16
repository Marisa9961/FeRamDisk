# FeRamDisk: 基于 STM32G431 的铁电存储 U 盘

## 项目概述
FeRamDisk 是一款基于 **STM32G431** 主控与 **MB85RS2MTA** 存储介质的专用闪存盘。不同于传统的 NAND Flash 存储方案，本项目利用铁电随机存取存储器（FeRAM）的物理特性，实现了极高的读写耐久性与近乎瞬时的随机访问能力。

## 注意

### 1. 临时硬件适配说明
当前分支/版本针对 **实验性硬件环境** 进行了底层驱动修改，与生产环境（4-Chip 256KB 原版）存在关键差异：
* **目标芯片**: `MB85RS256BPNF` (32KB / 256K-bit)。
* **物理配置**: 当前代码逻辑仅适配 **2 片级联** (Total 64KB)，而非原定的 4 片。
* **寻址协议**: 已将 SPI 指令 Header 从 **4 字节 (24-bit 地址)** 缩减为 **3 字节 (16-bit 地址)**。

### 2. 代码合并禁忌 (Merge Warning)
**严禁将硬件参数配置直接 Merge 回 `main` 主线。**
在将 U 盘通用逻辑（如 USB MSC 协议栈、FAT12 读写优化）同步回主线时，请务必执行以下操作：
* **保持主线常量**: `CHIP_SIZE_BYTES` 必须维持 `256 * 1024`，`ADDR_BYTES` 必须维持 `3`。
* **推荐合并方式**: 
    1.  使用 `git cherry-pick <commit_id>` 仅挑选功能性代码提交。
    2.  或使用 `git checkout <branch> -- <file>` 针对性检出非配置类文件。

### 3. FAT12 布局变动
由于物理存储空间从 1MB 骤减至 64KB/128KB，文件系统布局参数已做调整：
* **保留扇区**: `RESERVED_SECTORS` 保持为 1。
* **根目录条目**: 需关注 `ROOT_DIR_ENTRIES` 是否占用过多空间，当前逻辑可能导致数据区极小。
* **校验**: 格式化后若电脑提示“容量错误”，请优先检查 `build_boot_sector` 中的 `total_sectors` 字段是否与物理芯片总块数对齐。

### 4. 硬件回归 Checklist
当你换回 **MB85RS2MPNF** (256KB) 芯片时，必须还原以下逻辑：
- `feram.rs` 中的 `read_on_chip` / `write_on_chip` 指令 Header 恢复为 4 字节（1 指令 + 3 地址）。
- `CHIP_SIZE_BYTES` 恢复为 `262144`。
- 重新运行 `ensure_mass_storage_volume()` 进行全盘初始化。
