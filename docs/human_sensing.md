# Pessoas e sinais fisiológicos no HPS-3D160-U

Resposta curta:

| Pergunta | Resposta |
|---|---|
| **Detectar pessoas** | **Sim** — é uma das aplicações原生 do sensor (AGV/evasão, segurança de área). Implementado neste projeto (`PresenceDetector`) e suportado nativamente pelo SDK (modo de obstáculos). |
| **Medir o batimento cardíaco** | **Não** — fisicamente fora do alcance deste sensor. Medido e demonstrado abaixo (o sinal do batimento é 20× a 150× menor que o ruído do sensor). |
| **Medir respiração** | **Marginalmente** — a 0,3–1,0 m, com filtragem temporal pesada; o deslocamento do peito (5–20 mm) fica na mesma ordem do ruído (6–17 mm). Não confiável. |

---

## 1. Por que o batimento cardíaco é impossível aqui

O HPS-3D160 é um sensor **ToF** (tempo de voo) com VCSEL de 850 nm: ele mede
**distância absoluta**, e não fase nem velocidade. Não há, portanto, Doppler
disponível. Mesmo ignorando isso, a medição direta do batimento é inviável:

### 1.1 Resolução espacial

| | valor |
|---|---|
| FoV horizontal | 76° em 160 px → **0,475°/px** |
| Largura de um pixel a 1,0 m | **8,3 mm** |
| Largura de um pixel a 0,5 m | 4,2 mm |
| Deslocamento da parede do peito por batimento | **0,3 – 2 mm** |

O sinal é **4× a 25× menor que um pixel**.

### 1.2 Ruído de medição (medido neste projeto)

Ferramentas: `tools/noise_probe.cpp` e `tools/noise_vs_distance.cpp`
(`./tools/build_tools.sh`).

**Ruído temporal vs. tamanho da ROI** (superfície estática a ~4 m, 507 frames):

| ROI | σ temporal | σ equivalente por pixel |
|---|---|---|
| 1 × 1 px | **46,93 mm** | 46,93 mm |
| 4 × 4 px (16 px) | 16,59 mm | 4,15 mm |
| 8 × 8 px (64 px) | 9,61 mm | 1,20 mm |
| 16 × 16 px (256 px) | **6,19 mm** | 0,39 mm |

**Ruído vs. distância** (por pixel, 590 frames):

| faixa | σ por pixel |
|---|---|
| 0,5 – 1,0 m | 15,6 mm |
| 1,0 – 1,5 m | 24,2 mm |
| 1,5 – 2,0 m | 47,0 mm |
| 2,0 – 3,0 m | 35,3 mm |
| 3,0 – 4,0 m | 39,1 mm |
| 4,0 – 6,0 m | 38,7 mm |

### 1.3 Conta de SNR

Para detectar um sinal de amplitude `A` com SNR ≥ 3 sobre ruído gaussiano, é
preciso `N ≥ 9·(σ/A)²` pixels coerentes:

| sinal | σ (ROI 16×16) | pixels necessárias | disponível? |
|---|---|---|---|
| batimento (A = 0,5 mm) | 6,19 mm | **13.800** | ✗ (o sensor tem 9.600 no total) |
| batimento forte (A = 2 mm) | 6,19 mm | **870** | ✗ (e o sinal está em ~2–4 px, não em 870) |
| respiração (A = 10 mm) | 6,19 mm | 35 | ✓ (marginal) |

O batimento cardíacooor está concentrate em poucos pixels — averaging não
ajuda, e a largura de um pixel já é maior que o sinal.

**Conclusão:** o batimento cardíaco exigiria um ToF de alta resolução
(≥ 320×240), com > 60 fps e resolução sub-milimétrica, a menos de 0,5 m —
ou um sensor dedicado (ver §4).

---

## 2. Detecção de pessoas — o que é viável

### 2.1 Nativo do SDK

O SDK expõe um **modo de obstáculos**, exatamente para detecção de pessoas e
obstáculos:

| Função | Função |
|---|---|
| `HE201_ApiObstacleConfigInit` | habilita, nº de obstáculos, faixa válida (mm), valor inválido |
| `HE201_ApiObstacleAvoidance` | devolve obstáculos com `LeftPoint`, `RightPoint`, `UpperPoint`, `UnderPoint`, `MinPoint` e nº de pixels |
| `HE201_ApiSetObstaclePixelNumberThreshold` | limiar de pixels por obstáculo |

### 2.2 Implementado neste projeto: `PresenceDetector`

`src/presence_detector.cpp` — detector geométrico sobre o point cloud métrico:

1. descarta pixels inválidos e o que estiver além do alcance
2. estima a **altura do chão** pelo histograma de alturas (o chão é a superfície
   inferior mais extensa) e remove os pixels do chão
3. agrupa os pixels restantes por DBSCAN em grade (ε = 12 cm)
4. classifica cada grupo como *person-like* quando:
   altura ∈ [0,55 ; 2,30] m, largura ≥ 0,22 m e distância ≤ alcance
5. publica `std_msgs/Bool` em `/hps3d160/presence` (ROS) e imprime o relatório
   com `--presence`

Teste unitário (`tests/test_presence_detector.cpp`, cenas sintéticas):

```
cena so com chao (2720 pts): 4 blob(s), 0 person-like      [OK] nao detecta pessoa
cena com pessoa 1.70 m a 2.0 m: 1 blob(s), 1 person-like; closest at 2 m, height 1.62 m
                                                         [OK] detecta presenca
                                                         [OK] contagem = 1
                                                         [OK] altura ~1.7 m
                                                         [OK] distancia ~2.0 m
cena com 2 pessoas: 2 blob(s), 2 person-like; closest at 1.5 m
                                                         [OK] detecta 2 pessoas
cena com objeto pequeno 0.35 m: 1 blob(s), 0 person-like   [OK] nao classifica como pessoa
cena sem pontos validos                                     [OK] ausente
```

Execução real no sensor (cena atual, 5 frames consecutivos):

```
[presence] frame 1: 4 blob(s), 2 person-like; closest at 2.974 m, height 2.25 m
    blob  3193 pts  h=2.25 m  w=1.80 m  z=2.97 m  <person-like>
    blob   960 pts  h=1.21 m  w=0.78 m  z=1.44 m  <person-like>
```

> **Limite honesto do detector:** a classificação é **geométrica** (proporções
> corporais). Um móvel, um pilar ou um monitor com proporção humana também será
> marcado como *person-like*. Para identificar *pessoas* de fato seria preciso
> um classificador sobre o point cloud — viável (a resolução de 160×60 é
> suficiente para uma rede leve), mas fora do escopo deste módulo.

### 2.3 O que mais dá para fazer com o sensor

| Recurso | Precisão realista |
|---|---|
| Presença / zona ocupada | alta (via ROI + threshold + alarme GPIO) |
| Contagem de pessoas | boa, com múltiplas ROIs e tracking |
| Altura de uma pessoa | ±10 cm a 2 m |
| Silhueta / postura grosseira | ~8 cm por pixel a 1 m |
| Velocidade de deslocamento | derivável do cloud, ±5 cm/frame |
| Detecção de queda / inclinação | viável com filtragem temporal |

---

## 3. Respiração (limite)

O deslocamento do tórax durante a respiração é de 5–20 mm. Com uma ROI de
16×16 px o ruído é 6,2 mm (a ~4 m) e 2,7 mm (média de uma faixa de pixels), então:

- a < 1 m: plausível, com passa-baixa 0,1–0,5 Hz e integração longa
  (`simple_hdr`/`super_hdr`)
- a > 2 m: ruído × 4 → inviável

Não foi implementado: exigiria o sensor muito próximo do paciente e uma
montagem dedicada.

---

## 4. Alternativas para sinais fisiológicos

| Técnica | Sensor | Custo | Observação |
|---|---|---|---|
| **rPPG** (fotopletismografia por câmera) | câmera RGB | baixo | o HPS-3D160 é monocromático ToF, sem RGB |
| **PPG de dedo** (MAX30102, MMAX30102) | sensor dedicado | ~US$ 3 | o mais simples e confiável |
| **Radar mmWave 60 GHz** (IWR6843, Seeed MR60BHA2) | radar | médio | sem contato, mede respiração e batimento, **funciona através de roupa e a 1–5 m** |
| **ToF dedicado de alta resolução** (TI OPT3101, 320×240) | ToF | médio | a literatura mede batimento cardíaco com ToF de alta resolução a 0,3–0,5 m |
| **LiDAR FMCW** (Vayyar, Acconeer A111) | radar | alto | batimento + respiração + presença |

---

## 5. Referências do SDK usadas

```
HE201_ApiObstacleConfigInit          // modo de obstáculos
HE201_ApiObstacleAvoidance            // extração de obstáculos
HE201_ApiSetObstaclePixelNumberThreshold
HE201_ApiSetROIThresholdConf         // limiares por ROI
HE201_ApiSetROIAlarmType              // alarme por GPIO
HE201_ApiSetGPIOOut
```

## 6. Reprodução

```bash
# detector de presença
./build/hps3d160_max_node --port /dev/ttyACM0 --presence --max-frames 10

# ruído temporal
./tools/build_tools.sh
./build/tools/noise_probe 10

# ruído vs distância
./build/tools/noise_vs_distance 14

# teste unitário do detector
g++ -std=c++17 -Iinclude -Iinclude/hps3d160_max -Ithird_party/hps_sdk/include \
    src/presence_detector.cpp tests/test_presence_detector.cpp -o /tmp/t && /tmp/t
```
