# Frequência de atualização — HPS-3D160-U

Ferramenta: `hps3d160_max_node --bench [SEGUNDOS] [--bench-out arquivo.json]`

```bash
./build/hps3d160_max_node --port /dev/ttyACM0 --bench 10 --bench-out docs/throughput.json
```

O benchmark alterna o modo HDR do sensor, mede a taxa de frames durante N
segundos de captura contínua e registra o intervalo mediano entre frames, a
largura de banda do payload e o tipo de pacote recebido.

---

## Resultado medido (10 s por configuração)

| configuração | FPS | intervalo mediano | payload | evento |
|---|---|---|---|---|
| `disable` (HDR fixo) | 33,0 | 30,19 ms | 4,44 MB/s | 3 (full depth) |
| **`auto_hdr`** | **42,4** | **23,62 ms** | 5,70 MB/s | 3 |
| `super_hdr` (4 frames sintetizados) | 16,8 | 59,34 ms | 2,26 MB/s | 3 |
| `simple_hdr` (integração adaptativa) | 19,5 | 49,11 ms | 2,62 MB/s | 3 |
| `packet_depth` (payload completo) | 20,6 | 48,78 ms | 2,77 MB/s | 3 |
| `packet_roi` (payload reduzido) | 20,2 | 49,60 ms | **0,01 MB/s** | 4 (simple depth) |

Rodadas repetidas dão resultados consistentes (`auto_hdr` entre 37,6 e 42,6 fps).

## Respostas diretas

| Pergunta | Resposta |
|---|---|
| **Frequência máxima de atualização** | **≈ 42,5 fps** (modo `auto_hdr`, medido) |
| Modo mais rápido | `auto_hdr` — 23,6 ms por frame |
| Modo mais lento | `super_hdr` — 59,3 ms (sintetiza 4 frames por amostra) |
| Datasheet | 35 fps em FOV completo (o firmware entrega mais que isso em `auto_hdr`) |
| Com ROI definido | o datasheet indica taxa **maior** que 35 fps (não validado: o SDK V1.8.6 não expõe a criação de ROIs) |
| End-to-end no ROS | **42,52 Hz** em `/hps3d160/point_cloud` com `--hdr-mode auto` → o ROS não é o gargalo |

## Observações

1. **O modo `auto_hdr` é o mais rápido**, contra-intuitivamente: em `disable` o
   firmware usa integração fixa, e o `auto_hdr` sintetiza a exposição de forma
   mais agressiva.
2. **A banda USB não é o gargalo**: 5,7 MB/s num link CDC-ACM é trivial. O
   limite é o tempo de integração do sensor.
3. **O tipo de pacote muda a banda em 100×** (2,77 → 0,01 MB/s) sem alterar o
   FPS. Para streaming com pouca banda use `packet_roi` (o callback passa a
   receber `HPS3D_SIMPLE_DEPTH_EVEN`, evento 4) — mas perde-se a profundidade
   por pixel e o point cloud.
4. **`super_hdr` custa 4× a latência** (4 frames sintetizados por amostra) e é
   o modo mais lento.
5. A taxa é estável: `min 15 ms / max 32 ms`, desvio-padrão de 1,9 ms em 318
   amostras no ROS.

## Como usar

```bash
# taxa máxima
./build/hps3d160_max_node --port /dev/ttyACM0 --hdr-mode auto

# ROS
rosrun hps3d160_max hps3d160_max_node --port /dev/ttyACM0 --hdr-mode auto
```

Os modos são aplicados por `HE201_ApiSetHDRMode` (API interna do SDK),
confirmados por readback em `HE201_ApiGetHDRConfig`.

## Relação com o guia de HDR

`docs/hdr_guide.md` mostra que, **em ambiente com boa luz**, os quatro modos
têm SNR equivalente — neste caso use `auto_hdr` também, porque ele é o mais
rápido. `simple_hdr` só compensa quando a luz é escassa e o jitter importa
(menor jitter temporal: 3,7 mm vs 4,9 mm).
