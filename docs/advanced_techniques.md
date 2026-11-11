# Técnicas avançadas com o HPS-3D160-U — fundamentadas por medição

Este documento não lista ideias genéricas: cada técnica abaixo está ancorada em
números medidos no sensor real (HPS-3D160-U, FW V1.7.89, SDK V1.8.6).

---

## 1. A base quantitativa: o que o sensor consegue medir de fato

### 1.1 Composição do ruído

`tools/fpn_probe.cpp` decompõe o ruído de distância em:

- **ruído aleatório** — variação frame a frame; **agrega** com 1/√N
- **FPN (fixed pattern noise / speckle)** — desvio fixo por pixel; **não agrega**

Medido (cena real, ~900 frames, pixels válidos em todos os frames):

| modo HDR | σ aleatório | FPN | 8×8 (64 px) | 16×16 (256 px) |
|---|---|---|---|---|
| `disable` | **17,4 mm** | 87,2 mm | 7,36 mm | 6,98 mm |
| `auto_hdr` | 21,0 mm | 86,2 mm | 7,59 mm | 6,44 mm |
| `super_hdr` | **28,2 mm** | 86,4 mm | 8,16 mm | 6,75 mm |
| `simple_hdr` | 17,7 mm | 87,8 mm | 7,14 mm | **6,15 mm** |

Destaques:

1. **O FPN (~87 mm) é 5× maior que o ruído aleatório** e é praticamente
   independente do modo HDR → é uma propriedade da óptica/VCSEL, não do firmware.
2. **`disable` tem o menor ruído por pixel** (17,4 mm) e `super_hdr` o maior
   (28,2 mm): a síntese de 4 frames introduz variabilidade. O `disable` também
   é o mais rápido (33 fps) — para medição fina, ele é o melhor modo.
3. A agregação espacial real (medida, não teórica) satura em ~6 mm com 256 px,
   porque o FPN deixa de agregar.

### 1.2 Limiar de detecção de deslocamento (medido)

Resíduos em relação ao plano local (remove a estrutura da cena), agregados em
blocos — a dispersão do bloco **é** o limiar de detecção:

| região | pixels | σ do bloco | detecção 3σ | detecção 3σ **com FPN calibrado** |
|---|---|---|---|---|
| 2×2 | 4 | 20,8 mm | 62 mm | 40 mm |
| 4×4 | 16 | 13,4 mm | 40 mm | 13,1 mm |
| 8×8 | 64 | 7,4 mm | 22 mm | 6,5 mm |
| 16×16 | 256 | 6,2 – 7,0 mm | 18 – 21 mm | **3,3 mm** |

### 1.3 Ruído por pixel vs. distância (outra cena, 590 frames)

| faixa | σ por pixel |
|---|---|
| 0,5 – 1,0 m | 15,6 mm |
| 1,0 – 1,5 m | 24,2 mm |
| 1,5 – 2,0 m | 47,0 mm |
| 2,0 – 3,0 m | 35,3 mm |
| 3,0 – 4,0 m | 39,1 mm |
| 4,0 – 6,0 m | 38,7 mm |

---

## 2. Técnicas viáveis, com o número que as habilita

### 2.1 Calibração de FPN por pixel — a chave habilitadora
O SDK expõe `HE201_GetBadPixels` / `HE201_SaveCalibrationTable` /
`HE201_ProfileClearBadPixelsTable`, mas **a tabela do sensor está vazia**
(verificado: `HE201_GetBadPixels` retorna 1 sem escrever nada).

Isso significa que a calibração de FPN **precisa ser feita no software**:
capturar N frames de superfícies planas, estimar o bias por pixel e aplicá-lo
na reconstrução do point cloud. Ganho medido: **20,9 mm → 3,3 mm** no limiar de
detecção com 256 px (6× melhor).

É a técnica de maior retorno sobre investimento de todo o conjunto:
- habilita detecção de **respiração** a 2–4 m
- habilita medição de **movimento de superfície** de ~1 cm (porta, cargas, braços)
- habilita **calibração metrológica** do sensor (transformação distância ↔ matéria)

### 2.2 Detecção de respiração (contato indireto)
Sinal do tórax: 5–20 mm. Limiar com 256 px: 3,3 mm **com FPN calibrado**
(18–21 mm sem). Com FPN calibrado e filtragem passa-baixa 0,1–0,5 Hz, é
**viável a 1,5–4 m**, marginal a mais.

### 2.3 Batimento cardíaco — inviável, quantitativamente
| sinal | σ (256 px, com FPN calibrado) | pixels necessários |
|---|---|---|
| 0,5 mm | 3,3 mm | 11.500 |
| 2,0 mm | 3,3 mm | 720 |

O sensor tem 9.600 pixels **no total** e o sinal do batimento ocupa 2–4 pixels
(8,3 mm/px a 1 m contra 0,3–2 mm de deslocamento). **Confirmado: não é
detectável** — e o sensor não tem coerência para Doppler.

### 2.4 Vibração /.mode frequency de estruturas
Ruído de 1 pixel: 17–28 mm. Para medir uma frequência modal de uma viga via
deslocamento de superfície seria preciso resolver micras — **inviável**. O que
*é* viável: **deslocamento de ≥ 2 cm** de uma superfície grande (porta,
carga pendurada, braço de robô, painel) a 2–4 m, via 256+ pixels.

### 2.5 Tracking multi-objeto e multi-alvo
Com o cloud métrico e 42 fps, tracking por JPDA/MHT é viável para 2–5 alvos.
Ruído de posição de um blob: σ/√N do blob → com 100 px, ~2–3 mm por frame;
com filtro temporal (Kalman +时不 measurement), erro de posição < 1 cm.

### 2.6 Medição de nível (líquidos, sólidos, neve)
Alvo horizontal a 0,5–3 m: usar a **média de uma ROI** (o ruído cai como 1/√N
para o sinal, e o FPN é cancelado pela média temporal se o alvo não se move).
Precisão real: ~1–3 mm com 1000+ pixels. **É o caso de uso em que o sensor é
ideal.**

### 2.7 SLAM / odometria visual-inercial
O point cloud métrico + IMU (BNO055/ICM-42688) → LIO-SAM / FAST-LIO.
Limite honesto: 160×60 é raso e esparso; funciona em interiores Known
(e a 30 fps há degradação). Para outdoor, o HPS-3D160 não substitui um LiDAR
de 128 linhas.

### 2.8 Depth completion (upsampling 160×60 → denso)
Técnica padrão de visão computacional: rede de completion (NLSPN,
CompletionFormer, DepthFormer) que transforma a depth map esparsa em depth
densa e com bordas suaves. Entrada 160×60 é **idealmente pequena** para
edge inference (Jetson Orin: < 5 ms). Usos: navegação, previsão de
velocidade de fluxo óptico, oclusão para tracking.

### 2.9 Fusão passiva: o sensor como "radar de baixa resolução"
Com 42 fps e alcance de 12 m, o HPS-3D160 funciona como um **radar de
imagem**: detecta qualquer objeto que se mova no cone, com resolução angular
de 0,475°. Técnicas: detecção de intrusão por variação de depth, contagem de
fluxo empassagens, detecção de queda por colapso abrupto de silhueta.

### 2.10 Calibração intrinsics a partir do próprio cloud
O modelo usado neste projeto (`tan θ = (80 − col)/104`, `tan φ = (30 − row)/104`)
é a calibração do fabricante. Com o cloud métrico é possível **re-calibrar
online**: comparar a distância medida com a distância geométrica de um plano
conhecido e ajustar `kFocalPx` e as centros (as intrinsics do SDK têm
`HE201_GetDistortionConf`, com `enable=1` e parâmetros ativos).

### 2.11 Correção de distorção e não-linearidade do ToF
`HE201_GetDistortionConf` devolve `enable=1` com parâmetros ativos no device.
Para distâncias grandes, o ToF tem não-linearidade que a correção de
distorção do SDK endereça. O `HE201_ApiSetDistortionArgSumMax/Min` permite
ajustar os limites — **é a alavanca para estender o alcance útil além de 4 m**.

### 2.12 Splicing multi-câmera
`HE201_SpliceSetDeviceNumber`, `HE201_SpliceLoadROIConfig`,
`HE201_SpliceLoadRotateConfig`, `HE201_SpliceProcessFunc` — o SDK é capaz de
**fundir N unidades em um único cloud**, com correção de rotação. Isso viabiliza
uma "câmera" de 320×60 ou 480×60 sem mudar a Óptica: cobertura angular,
redundância (safety) e triangulação (composed com offsetsKnown).

### 2.13 Modo obstáculo nativo + ROI/limiares como SRAS
`HE201_ApiObstacleConfigInit` + `HE201_ApiObstacleAvoidance` + GPIO do modelo
U-IO (entrada = trigger de captura, saída = alarme por ROI) implementa um
**scanner de segurança** (SIL/PL): campo de detecção por ROIs, 3 limiares por
ROI com histerese, alarme por GPIO, heartbeat e keep-alive.

### 2.14 Bad pixels e calibração de fábrica
`HE201_SearchBadPixels`, `HE201_RemoveBadPixel`, `HE201_AddBadPixelByManual`,
`HE201_ProfileClearBadPixelsTable`, `HE201_SaveCalibrationTable`,
`HE201_SaveMultModClkCalibrationTable` — manutenção de sensor em campo.

### 2.15 Compensação de temperatura
`HE201_ApiGetSensorTemperature` devolve a temperatura do sensor (medida:
30,9 °C). O erro de distância do ToF varia com a temperatura do VCSEL → uma
**tabela de compensação distância × temperatura** é a forma correta de
calibração metrológica (e é o que o "perfil de fábrica" do fabricante faz).

---

## 3. Técnicas que exigem hardware adicional

| Técnica | Por quê |
|---|---|
| Batimento cardíaco / PPG | ToF sem coerência; ruído 17–28 mm/px vs. sinal 0,3–2 mm |
| Ressonância / vibração | precisa de resolução sub-micrométrica |
| Velocidade radial (Doppler) | exige FMCW |
| Identidade / rPPG | sem RGB |
| Longo alcance (> 4 m) | ruído ×2,5 e FPN fixo |

**Par recomendado:** radar mmWave 60 GHz (presença + frequência cardíaca,
atravessa roupa, 1–5 m) + HPS-3D160 (forma, posição, distância métrica). Cada
sensor cobre a fraqueza do outro.

---

## 4. Ordem de implementação recomendada

| # | Técnica | Ganho | Custo |
|---|---|---|---|
| 1 | **Calibração de FPN por pixel** (nova `PointCloudBuilder`) | limiar 20,9 → 3,3 mm | baixo |
| 2 | **ROI adaptativa + filtro temporal (Kalman)** sobre a série de distância | respiração e movimento fino | baixo |
| 3 | **Correção de distorção** (`HE201_ApiSetDistortionArgSum*`) | alcance útil além de 4 m | baixo |
| 4 | **Splicing de 2 unidades** | cobertura angular + redundância | médio |
| 5 | **Depth completion** no edge | cloud denso para navegação | médio |
| 6 | **SLAM + IMU** | odometria | médio |
| 7 | **Classificador de silhueta** (PointNet leve) | pessoa vs. objeto | médio |
| 8 | **ROS 2 (Jazzy)** | Nav2/SLAM/Foxy+ | médio |
| 9 | **Modo obstáculo nativo** como SRAS | produto industrial | médio |
| 10 | **Radar mmWave complementar** | batimento + presença | médio |

---

## 5. Reprodução

```bash
./tools/build_tools.sh

# matriz de ruído por modo HDR e por tamanho de região
./build/tools/fpn_probe 20 /dev/ttyACM0 disable
./build/tools/fpn_probe 20 /dev/ttyACM0 auto
./build/tools/fpn_probe 20 /dev/ttyACM0 super
./build/tools/fpn_probe 20 /dev/ttyACM0 simple

# ruído vs distância
./build/tools/noise_vs_distance 14 /dev/ttyACM0

# busca de sinal cardíaco (com teste de sanidade e consistência)
./build/tools/vital_signs 90 /dev/ttyACM0 0.3 2.5
```
