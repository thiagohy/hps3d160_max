# HPS-3D160-U — mapa completo de capacidades do hardware

Este documento consolida **todas** as capacidades do sensor HPS-3D160-U (variante
USB + I/O) que o SDK V1.8.6 permite usar, e como cada uma é acessada pelo sistema.

Fontes analisadas:

| Fonte | Conteúdo |
| --- | --- |
| `V1.8/lib/include/HPS3DBase_IF.h` | API pública C (26 funções) |
| `V1.8/lib/include/HPS3DUser_IF.h/.c` | camada de conveniência + parser de pacotes |
| `api.h` (V1.8.5, histórico do repo do fabricante) | **API completa**: ROI, GPIO, HDR, obstacle, splicing, bad pixels |
| `libHPS3DSDK64_1-8-6.so` | 345 símbolos exportados (≈90 `HE201_Api*` internos) |
| `HPS3D160简单通信协议.pdf` | protocolo binário (frames, CRC16, endereçamento) |
| `HPS-3D160_DS001` | especificações,optics, mechanics |

---

## 1. Sensor

| Item | Valor (medido no device `SN:HU23M12233D1901279`) |
| --- | --- |
| Princípio | ToF, VCSEL 850 nm, todos-sólido |
| Resolução | 160 × 60 (9.600 pixels) |
| FoV | 76° (H) × 32° (V) |
| FoV de iluminação | 82° × 36° |
| Alcance | 0,25 m – 12 m (alvo 90 % refletância) |
| Frame rate | até 35 fps (FOV completo); maior com ROI |
| Firmware | `V1.7.89 19-12-19` |
| SDK | `V1.8.6 23-10-30` |
| Temperatura do sensor | lida em runtime (≈30,8 °C) |
| Modos de integração | HDR disable / auto / super / simple (configurável) |

## 2. Canais de comunicação

| Canal | Estado | Notas |
| --- | --- | --- |
| USB (CDC-ACM) | **em uso** | `/dev/ttyACM0` no Linux, `COM3` no Windows |
| RS-232 | suportado | baud 9600…2.000.000, paridade, stop bits |
| RS-485 / CAN | suportado | `HE201_ConfigRS485`, `HE201_ConfigCAN` |
| LAN (Ethernet) | suportado | IP/porta configuráveis, keep-alive, reconnect |
| GPIO opto-isolado | **disponível** | 1 entrada + 1 saída na variante U (firmware reporta 3+3) |
| Endereçamento | 0x00–0xFE + broadcast 0xFF | `SetDevAddr` |
| Multi-câmera | até 16 dispositivos | `SetMultiCameraCode` (0–15) + detecção de interferência |

## 3. API pública (HPS3DAPI_*) — 26 funções

Conexão (`USBConnectDevice`, `EthernetConnectDevice`, `EthernetReconnectDevice`,
`CloseDevice`, `IsConnect`, `IsStart`), captura (`StartCapture`, `StopCapture`,
`SingleCapture`), eventos (`RegisterEventCallback`, `UnregisterEventCallback`),
identidade (`GetDeviceVersion`, `GetSDKVersion`, `GetSerialNumber`), configuração
(`SetDeviceUserID`, `SetROIGroupID`, `SetMultiCameraCode`, `ExportSettings`,
`SaveSettings`), filtros (`SetDistanceFilterConf`, `SetSmoothFilterConf`,
`SetDistanceOffset`, `SetOpticalPathCalibration`, `SetEdgeFilterEnable`) e
`SetEthernetKeepAlive`.

## 4. API completa (`HE201_Api*` / `HE201_*`) — habilitada pelo `SdkBridge`

O binário exporta a API do `api.h` V1.8.5, muito mais rica que a pública.
O `SdkBridge` (ver `src/sdk_bridge.cpp`) resolve os símbolos por `dlsym` e
associa o handle público ao handle interno, liberando:

### 4.1 ROI e limiares
- `GetNumberOfROI` → 8 ROIs, 3 limiares por ROI
- `GetROIConfById` → região (x0,y0,x1,y1), enable, e por limiar: valor, histerese,
  polaridade, tipo de referência (distância média/mín/máx, saturação, amplitude,
  amplitude válida, nº de pixels acima do limiar) e tipo de alarme (disable/GPIO)
- `SetROIRegion`, `SetROIEnable`, `SetROIThresholdEnable`, `SetROIThresholdConf`
- `SetROIAlarmType`, `SetROIReferenceType`, `SetSingleRoi`, `SetThreshold`
- `SelectROIGroup` / `GetROIGroupID` → 16 grupos (o firmware expõe 8 ROIs/grupo)

### 4.2 HDR / integração
- `GetHDRConfig` → modo (`super_hdr` no device), amplitude de sobreexposição (500),
  amplitude fraca (120), integração simple 200–2000 µs, super-HDR 4 frames / 2000 µs
- `SetHDRMode`, `SetHDRConfig`, `SetAutoHDRConfig`, `SetSimpleHDRConfig`,
  `SetSuperHDRConfig`, `SetHDRIntegrationTime`, `SetIntegrationTime`

### 4.3 GPIO
- `GetNumberOfGPIO` → contagem de entradas e saídas
- `GetGPIOInConf` / `SetGPIOIn` → pino, polaridade, função
  (`disable` ou `capture` = disparo de medição por sinal externo)
- `GetGPIOOutConf` / `SetGPIOOut` → pino, polaridade, função
  (`disable`, `roi_threshold0/1/2_alarm`)
- `SetGPOUT1_Val` / `SetGPOUT2_Val` / `SetGPOUT3_Val` → nível direto na saída

### 4.4 Obstáculos (evasão)
- `ObstacleConfigInit` / `GetObstacleConfig` → enable, nº de obstáculos, faixa
  válida (mm), valor inválido, cabeçalho de frame
- `SetObstaclePixelNumberThreshold` / `GetObstaclePixelNumberThreshold`
- `ObstacleAvoidance` → largura/altura da zona, distância mínima, altura de
  montagem, limiar de pixels, alarme e saída GPIO associada
- `ObstacleConvertToPointCloud`, `GetObstacleData`, `GetObstacleFrameData`

### 4.5 Point cloud e óptica
- `SetPointCloudEn` / `GetPointCloudEn`
- `SetPointCloudMirrorType` / `GetPointCloudMirrorType`
  (disable, horizontal, vertical, ambos)
- `ApiBaseToPointCloud`, `PointCloudRotateAndMove`, `SavePlyFile`
- `SetMountingAngleEnable` / `SetMountingAngleParamConf` / `GetMountingParamConf`
  → rotação do sistema de coordenadas + altura de instalação (mm)
- `SetOpticalEnable` / `GetOpticalParamConf` → compensação de caminho óptico

### 4.6 Filtros e limiares
- `SetDistanceFilterType` / `SetSimpleKalman` / `GetDistanceFilterConf`
  (filtro Kalman simples: K, threshold, nº de frames de verificação)
- `SetSmoothFilter` / `GetSmoothFilterConf` (disable, média, gauss + argumento)
- `SetThresholdOffset` / `GetThresholdOffset`
- `SetEdgeDetectionEnable` / `SetEdgeDetectionValue` / getters
- `SetDistanceOffset` / `GetDistanceOffset`

### 4.7 Anti-interferência multi-câmera
- `SetInterferenceDetectEn`, `…AmplitudeThreshold`, `…CaptureNumber`,
  `…IntegTime`, `…NumberCheck`, `GetInterferenceDetectConf`

### 4.8 Keep-alive / heartbeat
- `SetKeepAliveConfig(handle, enable, time_ms)`, `GetKeepAliveTime`,
  `GetKeepAliveEnale`, `SendKeepAlive`, `SetHeartBeatFlag`, `GetHeartBeatFlag`

### 4.9 Perfis, calibração e bad pixels
- `ProfileSaveToCurrent`, `ProfileClearCurrent`, `ProfileRestoreFactory`
- `SaveCalibrationTable`, `SaveMultModClkCalibrationTable`
- `SearchBadPixels`, `RemoveBadPixel`, `AddBadPixelByManual`,
  `ProfileClearBadPixelsTable`, `GetBadPixels`
- `GetFactoryInfo`, `GetFactoryMISCParam`, `GetHardwareInfo`
- `GotoBootLoader` (atualização de firmware)

### 4.10 Splice (várias câmeras em um único cloud)
- `SpliceSetDeviceNumber`, `SpliceGetDeviceNumber`, `SpliceLoadROIConfig`,
  `SpliceLoadRotateConfig`, `SpliceProcessFunc`, `SpliceRotateMove`,
  `SpliceSetCurrentGroupID`, `SpliceGetCurrentGroupID`

### 4.11 Transporte e debug
- `ConfigEthernet` / `ConfigRS232` / `ConfigRS485` / `ConfigCAN`
- `SetUartConfig` / `GetUartConfigStatus` / `SaveTransportConf`
- `SetDebugEnable` / `SetDebugFunc` / `GetDebugEnable`
- `SetSpecialMeasurementValue` / `ConfigSpecialMeasurementValue`
- `GetSensorTemperature`, `GetDeviceVersion`, `GetSerialNumber`, `GetDevAddr`

## 5. Protocolo binário (para quem quiser falar direto com o sensor)

- Comando: `F5 0A | len | cmd | endereço | params | CRC16-CCITT`
- Resposta: `F5 5F | len16 | endereço | RID | dados | CRC16 | 5F F5`
- Modos: `0x00` standby, `0x01` medida única, `0x02` contínuo
- 4 tipos de pacote de medição: full depth (19.200 B de distâncias), simple depth,
  full ROI, simple ROI
- Endereços: `0x00`–`0xFE`, broadcast `0xFF`
- Anti-interferência por código de câmera 0–15

## 6. O que o sistema já usa

| Recurso | Onde |
| --- | --- |
| Conexão USB/Ethernet + reconnect | `src/device_manager.cpp` |
| Captura contínua e única | `src/capture_service.cpp` |
| Conversão de pacote → point cloud (m) | `src/pointcloud_builder.cpp` |
| Filtros (distância, suavização, offset, óptico, borda) | `src/filter_config.cpp`, `device_manager.cpp` |
| Métricas FPS / drops / health | `src/health_monitor.cpp` |
| Gravação CSV + XYZ + matriz de distâncias + JSON | `src/recorder.cpp` |
| Publicação ROS `PointCloud2` | `src/ros_publisher.cpp` |
| Múltiplas câmeras / splicing de configuração | `src/multi_camera_manager.cpp` |
| **Relatório completo do hardware** | `src/sdk_bridge.cpp`, `src/capability_probe.cpp` |

### Ferramenta de diagnóstico

```bash
./build/hps3d160_max_node --port /dev/ttyACM0 --probe --probe-out report.json
```

Gera um JSON com identidade, link, óptica/HDR, filtros, ROIs (região + limiares),
GPIO, anti-interferência, point cloud, obstáculos e temperatura.

### Comparação de modos HDR (SNR)

```bash
./build/hps3d160_max_node --port /dev/ttyACM0 --hdr-compare 30 --hdr-out hdr.json
```

Alterna o sensor entre `disable`, `auto_hdr`, `super_hdr` e `simple_hdr`
(via `HE201_ApiSetHDRMode`) e mede, por modo:

| Métrica | Significado |
| --- | --- |
| `coverage` | fração de pixels com distância utilizável |
| `low_amplitude_ratio` | fração de pixels com sinal fraco (`LOW_AMPLITUDE`) |
| `saturated_ratio` | fração saturada / ADC overflow |
| `spatial_noise_mm` | mediana do desvio-padrão da distância em patches 8×8 (ruído) |
| `temporal_jitter_mm` | desvio-padrão da distância média entre frames |
| `snr_proxy` | distância média ÷ ruído espacial |
| `fps` | taxa efetiva por modo (super-HDR sintetiza 4 frames) |

**Nota:** o firmware atual (`V1.7.89`) **não envia amplitude por pixel** no payload
(16 B de estatísticas + 9.600 distâncias + point cloud de 12 B/ponto = 134.416 B),
por isso o SNR é estimado pelo comportamento espacial/temporal do depth map.

Exemplo real medido em `docs/hdr_comparison.json`.


## 7. Limitações conhecidas

- A API interna não é documentada oficialmente neste repositório: as assinaturas
  foram confirmadas por **disassembly** da biblioteca e validadas contra o device.
  Uma atualização do SDK pode mudar offsets/assinaturas — o `SdkBridge` valida o
  handle em runtime e desabilita o caminho avançado se a validação falhar.
- `HPS3D_ApiSetROIRegion` e `ObstacleConfigInit` recebem structs grandes por
  valor; o layout exato da V1.8.6 não é verificável, então esses *setters* não
  são usados pelo sistema (apenas os *getters*).
- Não há API pública para criar ROIs no firmware atual sem passar pelo software
  do fabricante — o sistema apenas lê e seleciona grupos.
- Não há API de HDR exposta publicamente; só via `HE201_Api*`.
