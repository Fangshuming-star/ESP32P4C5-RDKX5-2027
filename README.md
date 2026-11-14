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
ESP32P4C5-RDKX5-2027/
├── README.md                      # 本说明
├── 如何集成到已有项目.md            # 把无线下载接进自己工程的步骤
├── c5_firmware/                   # C5 协处理器固件（烧一次即可）
│   ├── flash_c5.bat               # 一键烧录 C5（参数：串口号，默认 COM16）
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
├── p4_host/                       # P4 主机代码（参考工程，纯代码）
│   ├── main/                      # main.c + CMakeLists + idf_component.yml + Kconfig
│   ├── CMakeLists.txt
│   ├── partitions.csv             # 16MB 双 OTA 分区
│   └── sdkconfig.defaults         # esp_hosted + wifi_remote + OTA 配置
├── wifi_ota_component/            # 可复用组件（拷进自己工程的 components/）
└── tools/                         # 电脑端调试工具（与芯片代码分离）
    ├── p4_tool.py                 # 图形化工具（主程序）
    ├── 启动P4工具.bat             # 双击启动图形化工具
    ├── pc_tcp_client.py           # 命令行 TCP 测试客户端
    ├── pc_ota_server.py           # 命令行 OTA 触发脚本
    └── tool_config.json           # 工具配置（自动生成，记住上次设置）
```

> 本仓库只保留**源码与预编译固件**。`p4_host/build/`、`p4_host/managed_components/`、
> `p4_host/sdkconfig`、`c5_firmware/source/sdkconfig` 都是编译产物，
> 首次 `idf.py build` 会自动重新生成，因此不入库。
>
> `wifi_ota_component/` 是最小可移植模板（HTTP OTA）；实际项目里
> （如 `D:\ESP32-IDF\code\DIpances\components\wifi_ota`）已增强：
> TCP 推送 OTA、WiFi 运行时改账号、断线重连、失败诊断扫描 —— 集成时以工程内版本为准。

### 图形化工具功能

`tools/p4_tool.py` 集成以下功能，全部可**中英文切换**：

| 功能 | 说明 |
|------|------|
| **双通道** | 有线串口 / 无线 TCP 各自独立收发区，可单独关闭（关闭即断开连接） |
| **串口识别** | 按 VID/PID 自动挑出板子所在串口，显示描述，无需记 COM 号 |
| **IP 识别** | 串口日志 / TCP 对端 / 局域网扫描三级自动识别 |
| **无线下载** | TCP 推送（推荐，不怕防火墙）+ HTTP 下载，失败自动回退 |
| **固件烧录** | C5 / P4 预设（多文件、固定偏移），也可自定义 |
| **自动化** | 一键流程：保存 WiFi → 构建 → 等 IP → 无线下载；WiFi 配置直写工程 |
| **记忆** | 串口、IP、工程目录、固件路径、下载方式、布局全部记住 |

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

> 说明：`flash_c5.bat` 默认串口 `COM16`，可用第一个参数覆盖（如 `flash_c5.bat COM5`）。
> 它会自动在 `IDF_PYTHON_ENV_PATH` / `IDF_TOOLS_PATH` / `%USERPROFILE%\.espressif` 下找
> `esptool.exe`，找不到再用 PATH 里的 `esptool`。固件偏移：
> `bootloader@0x2000`、`partition-table@0x8000`、`ota_data@0xd000`、`slave@0x10000`，
> flash 参数 `dio / 80m / 4MB`。若提示连接失败，按住 BOOT 再点一下 RST 进入下载模式。

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

> **两种下载方式**（工具里可选，默认推荐第 1 种）：
>
> | 方式 | 原理 | 优点 | 注意 |
> |------|------|------|------|
> | **TCP 推送**（`ota-push <字节数>`） | PC 通过 3333 端口把固件二进制直接推给 P4 | 不需要电脑开 HTTP 端口，**不受防火墙影响**，最稳 | 需要设备固件支持 `ota-push`（本项目 wifi_ota 已支持） |
> | **HTTP 下载**（`ota http://电脑IP:8000/x.bin`） | P4 反向访问电脑上的 HTTP 服务器 | 兼容老固件 | 电脑需放行 8000 端口入站；实测比推送慢很多（1MB 约 130s） |
>
> **一次只能下一个 bin 是正常的**：OTA 只写 `ota_0`/`ota_1` 应用分区，
> 不能更新 bootloader 与分区表。需要一起更新时请用「固件烧录」页（多文件 + 固定偏移）。

### 方式 A：图形化工具（推荐）

1. 把 `tools/p4_tool.py`、`启动P4工具.bat` 复制到任意位置（工具会自动定位 P4 工程）。
2. 双击 `启动P4工具.bat` 打开界面。界面要点：
   - **自动识别串口**：按 VID/PID（Espressif / CP210x / CH34x / FTDI）挑出板子所在串口，也可手动选；
   - **自动识别 IP**：串口日志 `got ip:` / `IP=` → TCP 对端 → 局域网 3333 端口扫描，三级兜底；
   - **双通道**：有线串口、无线 TCP 各自有独立收发区，`✕` 可整体关闭（同时断开底层连接），顶部勾选可随时恢复；
   - **无线下载**：选「TCP 推送」→ 点「构建固件」→ 点「开始无线下载」，进度条 + 弹窗提示；
   - **HTTP 自检**：先用它确认电脑上的 8000 端口可被访问，再走 HTTP 方式；
   - **一键流程**：保存 WiFi → 构建 → 等 IP → 无线下载，全自动。
3. 所有路径与选项（串口、IP、工程目录、C5 固件目录、固件路径、下载方式）**关闭后再次打开会自动记住**。

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
> `推送进度 30%` → `OTA succeeded, rebooting...` → 重启后新的 `got ip:`
>
> 失败时日志会直接说明原因，例如：
> `首字节 0xXX != 0xE9，不是应用镜像`、`ota end failed`（镜像校验失败）、
> `http open failed`（HTTP 方式被防火墙拦截）。

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
