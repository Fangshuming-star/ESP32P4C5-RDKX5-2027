# ESP32-P4C5 无线下载调试 完整工程包

本文件夹是一套**可复用的 ESP32-P4 + ESP32-C5 无线开发模板**，包含三部分：

| 部分 | 芯片 | 作用 |
|------|------|------|
| [`c5_firmware/`](./c5_firmware) | ESP32-C5 | 协处理器固件（SDIO 从机 + Wi-Fi 射频），**烧一次即可，之后不用动** |
| [`p4_host/`](./p4_host) | ESP32-P4 | P4 主机代码（纯代码，含 TCP 回显 + OTA 触发），可移植到任意新 P4 项目 |
| [`tools/`](./tools) | 电脑端 | 图形化调试工具（串口 + TCP + OTA + 固件烧录），与芯片代码分离 |

> 板子：OSPTEK ESP32-P4C5-Core。P4 无内置射频，Wi-Fi 由 C5 通过板内 SDIO 提供。
> 术语：**host = P4 主机**，**slave/coprocessor = C5 协处理器**。

---

## 一、目录结构

```
esp32p4c5_wireless parameter tuning and code download/
├── README.md                      # 本说明
├── c5_firmware/                   # C5 协处理器固件（烧一次即可）
│   ├── flash_c5.bat               # 命令行一键烧录 C5
│   ├── prebuilt/                  # 预编译固件（直接烧，无需重编译）
│   │   ├── bootloader.bin         #   烧到 0x2000
│   │   ├── partition-table.bin    #   烧到 0x8000
│   │   ├── ota_data_initial.bin   #   烧到 0xd000
│   │   └── slave.bin              #   烧到 0x10000
│   └── source/                    # C5 固件源码（需改 C5 行为时才重编译）
│       ├── main/
│       ├── CMakeLists.txt
│       ├── partitions_eh_cp_ota_4m.csv
│       └── sdkconfig.defaults
├── p4_host/                       # P4 主机代码（纯代码，无工具脚本）
│   ├── main/                      # main.c + CMakeLists + idf_component.yml + Kconfig
│   ├── CMakeLists.txt
│   ├── partitions.csv             # 16MB 双 OTA 分区
│   └── sdkconfig.defaults         # esp_hosted + wifi_remote + OTA 配置
└── tools/                         # 电脑端调试工具（与芯片代码分离）
    ├── p4_tool.py                 # 图形化工具（主程序）
    ├── 启动P4工具.bat             # 双击启动图形化工具
    ├── pc_tcp_client.py           # 命令行 TCP 测试客户端
    ├── pc_ota_server.py           # 命令行 OTA 触发脚本
    └── tool_config.json           # 工具配置（自动生成，记住上次设置）
```

### 图形化工具功能

`tools/p4_tool.py` 集成以下功能，全部可**中英文切换**：

| 功能 | 说明 |
|------|------|
| **串口监视** | RX 区实时显示串口输出，自动提取 P4 IP |
| **TCP 通讯** | TX 区记录发送、RX 区记录接收，分两个区域 |
| **OTA 升级** | 一键「构建 + 无线下载」，含下载进度条 + 成功/失败弹窗 |
| **固件烧录** | 直接在工具里给 C5/P4/自定义芯片烧固件（esptool） |
| **WiFi 配置** | 界面直接输入 WiFi 名称/密码，保存到 sdkconfig.defaults |

> ⚠️ **WiFi 名称（SSID）只支持英文**（字母、数字、常用符号）。输入中文时工具会弹提示并拒绝保存。

---

## 二、第一次使用：烧录 C5 固件（只需一次）

C5 固件负责提供 Wi-Fi，**烧一次后永久有效**，之后做任何 P4 项目都不用再烧 C5。

1. 把开发板**拨码开关拨到 C5 位置**。
2. 双击运行 `c5_firmware/flash_c5.bat`（或命令行传入串口号）：

```bat
c5_firmware\flash_c5.bat COM16
```

3. 看到 `[OK] C5 firmware flashed successfully.` 即完成。

> 说明：`flash_c5.bat` 默认串口 `COM16`，可用第一个参数覆盖。固件烧录偏移：
> `bootloader@0x2000`、`partition-table@0x8000`、`ota_data@0xd000`、`slave@0x10000`，
> flash 参数 `dio / 80m / 4MB`。

### 什么时候需要重编 C5 固件？

默认用 `prebuilt/` 里的现成 bin 即可。只有当你需要**修改 C5 侧 Wi-Fi 行为**（如功耗、缓冲区参数）时，才用 `source/` 重新编译：

```bash
cd c5_firmware/source
idf.py set-target esp32c5
idf.py build
idf.py -p COM16 flash
```

重新编译出的固件也会更新 `source/build/`，可把新的 `slave.bin` 等复制回 `prebuilt/` 归档。

---

## 三、新建 P4 项目并移植「无线下载」功能

这是本工程包的核心用途：**把 `p4_host/` 的无线下载调试能力搬到任意新的 P4 工程里。**

### 步骤

1. 用 ESP-IDF 新建你的 P4 项目（或用现有工程），`idf.py set-target esp32p4`。

2. 把以下内容复制到新工程：
   - `p4_host/main/` 里的 4 个文件（`main.c`、`CMakeLists.txt`、`idf_component.yml`、`Kconfig.projbuild`）
   - `p4_host/partitions.csv`
   - `p4_host/sdkconfig.defaults`（**关键**：含 esp_hosted + SDIO 引脚 + OTA 配置）

3. 把新工程根 `CMakeLists.txt` 的 `project()` 名改掉即可（保持 `p4_host_ota` 也行）。

4. 改 WiFi 配置：编辑 `sdkconfig.defaults` 里的这两行，改成你的路由器：

```
CONFIG_ESP_WIFI_SSID="你的WiFi名"
CONFIG_ESP_WIFI_PASSWORD="你的密码"
```

5. 编译 + 首次有线烧录：

```bash
idf.py build
idf.py -p COM16 flash monitor
```

首次必须 USB 烧一次（把 OTA 接收能力刷进去），之后就能无线升级了。

### 核心：无线下载功能长什么样

`p4_host/main/main.c` 里已经内置了一个 **TCP 回显服务器（端口 3333）**，它会：

- 收到普通消息 → 原样回显（用于调试 TCP 通讯）；
- 收到 `ota <固件URL>` 开头消息 → 启动 `esp_https_ota` 下载新固件 → 写入 OTA 分区 → 自动重启。

也就是说，你在**自己项目的 `app_main()` 里**，只需：

```c
wifi_init_sta();                                    // 连 Wi-Fi（走 C5）
xTaskCreate(&tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);  // 起 OTA 接收服务
```

就能复用整套无线下载调试能力。你的业务代码加在这两行之后即可。

---

## 四、无线下载（OTA）调试流程

### 方式 A：图形化工具（推荐）

1. 把 `p4_host/` 里的 `p4_tool.py`、`启动P4工具.bat` 复制到新工程目录。
2. 双击 `启动P4工具.bat` 打开界面。
3. 界面操作：
   - **连接串口**：选 COM 口 → 点「连接串口」，日志区实时显示，自动提取 P4 的 IP；
   - **TCP 通讯**：填消息点「发送」，验证与 P4 的无线通讯；
   - **OTA 升级**：点「1. 构建固件」→ 点「2. OTA 无线升级」，软件会自动：
     - 起 HTTP 服务器提供 `.bin`
     - 通过 TCP 触发 P4 下载
     - 检测下载成功 + 重启成功，弹窗提示结果。

### 方式 B：命令行

```powershell
# 构建（或在 VS Code 点 ESP-IDF 的「构建」按钮）
idf.py build

# 无线升级（P4 需已连上 Wi-Fi 并拿到 IP）
python pc_ota_server.py <P4的IP>

# 验证 TCP 通讯
python pc_tcp_client.py <P4的IP> 3333 "hello"
```

> 升级成功后，P4 串口日志会依次出现：
> `Starting OTA from: ...` → `OTA writing flash` → `OTA succeeded, rebooting...`

---

## 五、关键配置速查

### P4 侧 SDIO 引脚（p4_host/sdkconfig.defaults 已配好）

| 信号 | P4 GPIO |
|------|---------|
| CLK | 18 |
| CMD | 19 |
| D0 | 14 |
| D1 | 15 |
| D2 | 16 |
| D3 | 17 |
| Reset(C5 RST) | 54 |

板级预设：`CONFIG_ESP_HOSTED_P4X_C5_DEV_BOARD_FUNC_BOARD=y`，协处理器目标 `esp32c5`。

### C5 侧（无需配置引脚，硬件固定）

板级预设：`CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD=y`。

### 组件依赖（idf_component.yml 自动拉取）

- **P4 侧**：`espressif/esp_hosted: ^3.1.0` + `espressif/esp_wifi_remote: >=1.3.1`
- **C5 侧**：`espressif/esp_hosted: ^3.1.0`

> 两侧都用 `esp_hosted ^3.1.0`，握手协议自动兼容，这是最稳妥的组合。

---

## 六、常见问题

| 现象 | 处理 |
|------|------|
| 烧 C5 失败 | 确认拨码开关在 C5 位、串口号正确、USB 线可通讯 |
| P4 连不上 Wi-Fi | 确认 C5 已烧录、sdkconfig.defaults 里 SSID/密码正确 |
| 无线升级时 P4 未下载 | 检查 Windows 防火墙是否放行 8000 端口、电脑与 P4 是否同一网段 |
| 找不到 `idf.py` | 需在 ESP-IDF 环境（终端）运行，或让图形工具使用 `IDF_PYTHON_ENV_PATH` |
| P4 的 IP 变了 | 看串口日志 `got ip:xxx`，图形工具会自动提取 |

---

## 七、依赖环境

- ESP-IDF ≥ 5.5（本工程基于 v5.5.5）
- 组件由 `idf_component.yml` 自动拉取（需联网）
- 电脑端工具：Python 3.11+（`pyserial` 用于串口，`tkinter` 用于图形界面）
