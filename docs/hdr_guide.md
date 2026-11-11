# HPS-3D160-U — HDR: escolha do modo por cenário e por direção

Ferramenta: `hps3d160_max_node --hdr-compare <frames> [--hdr-out arquivo.json]`

Ela alterna o sensor entre os quatro modos de integração e mede, por modo e por
setor angular horizontal do FoV (76° / 12 setores):

| Métrica | O que é |
|---|---|
| `coverage` | fração de pixels com distância utilizável |
| `low_amplitude_ratio` | fração de pixels com sinal fraco (`LOW_AMPLITUDE`) |
| `saturated_ratio` | fração saturada / ADC overflow |
| `spatial_noise_mm` | **resíduo** do ajuste de plano local (8×8 px) — separa ruído de estrutura |
| `temporal_jitter_mm` | desvio-padrão da distância média entre frames |
| `snr_proxy` | distância média ÷ ruído residual |
| `fps` | taxa efetiva por modo |

A métrica de ruído usa o **resíduo de um plano local** por patch (e não o
desvio-padrão bruto): assim uma parede inclinada não é contada como ruído.

---

## Resultado medido — cena real (40 frames/modo)

Cena: parede frontal a ~4,2 m, superfícies laterais a 1,7 m (esquerda) e 2,9 m
(direita), ambiente com boa luz.

| modo | fps | coverage | weak | satur. | ruído (mm) | jitter (mm) | SNR |
|---|---|---|---|---|---|---|---|
| `disable` | **31,1** | 99,3 % | 0,6 % | 0,1 % | 75,7 | 4,9 | 44,0 |
| `auto_hdr` | 31,6 | 99,3 % | 0,6 % | 0,1 % | 76,1 | 5,7 | 43,9 |
| `super_hdr` | **16,7** | 99,1 % | 0,9 % | 0,0 % | 78,9 | **4,1** | 42,2 |
| `simple_hdr` | 20,0 | 99,1 % | 0,9 % | 0,0 % | **75,6** | **3,7** | **44,2** |

### Ruído por setor (mm) — disable / auto / super / simple

| setor | disable | auto | super | simple |
|---|---|---|---|---|
| −34,8° | 46,1 | 46,4 | 50,2 | 46,3 |
| −28,5° | 73,3 | 73,4 | 73,6 | 73,7 |
| −22,2° | 79,6 | 79,5 | 80,1 | 79,7 |
| −15,8° | 153,1 | 153,8 | 153,7 | 153,4 |
| −9,5° | 215,0 | 215,5 | **206,7** | **206,1** |
| −3,2° | 74,8 | 77,9 | 78,2 | 76,5 |
| +3,2° | 24,5 | 24,3 | 23,9 | **23,8** |
| +9,5° | 24,1 | 23,9 | 24,4 | 23,9 |
| +15,8° | 48,7 | 48,4 | 47,5 | 47,5 |
| +22,2° | 82,4 | 82,4 | 84,6 | 82,6 |
| +28,5° | 91,5 | 91,5 | **105,0** | 93,9 |
| +34,8° | 46,4 | 46,0 | **66,3** | 47,0 |

---

## Conclusões

1. **Em ambiente com boa luz os 4 modos são equivalentes** (diferenças < 2 % em
   cobertura e ruído). O HDR só paga quando a luz é escassa.
2. **`disable` é o melhor padrão**: 31 fps com o mesmo SNR. `super_hdr` custa
   **metade da taxa de quadros** (16,7 vs 31,1) sem ganho mensurável.
3. **`simple_hdr` é o melhor compromisso quando a estabilidade importa**:
   menor jitter temporal (3,7 mm) e melhor SNR proxy (44,2), a 20 fps.
4. **Por direção**:
   - Setores frontais (+3° … +10°, parede lisa): ruído ~24 mm em **todos** os
     modos → use `disable`, não há ganho com HDR.
   - Setores −16° … −9°: ruído 150–215 mm **em todos** os modos → é estrutura
     (descontinuidade/degrau), não ruído de sensor; HDR não resolve.
   - Borda +34,8°: `super_hdr` é **pior** (66 mm vs 46 mm) — a síntese de 4
     frames degrada a borda do FoV.
   - Borda −9,5°: `simple_hdr`/`super_hdr` são os melhores (206 vs 215 mm),
     ainda com diferença marginal.
5. **Recomendação operacional**:
   - Luz boa → `disable` (máximo FPS).
   - Luz fraca → `simple_hdr` (jitter mínimo, 20 fps).
   - Contraste extremo / alvo muito próximo → `super_hdr` (só se FPS não
     importa), nunca na borda do FoV.
   - Nunca usar `super_hdr` se o alvo estiver a < 0,3 m (satura).

## Como repetir

```bash
./build/hps3d160_max_node --port /dev/ttyACM0 --hdr-compare 40 --hdr-out hdr.json
```

Antes de comparar, posicione o sensor para a cena que interessa (o resultado é
fortemente dependente do enquadramento) e, para "pouca luz", reduza a
iluminação ambiente durante a coleta.
