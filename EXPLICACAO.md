# Implementação

**Objetivo:** o robô real atravessa o labirinto sem bater nas paredes, com movimento suave, passando também por lugares estreitos.

---

## 1. `preprocess_map`: desconhecido vira parede
[astar.py:34-47](astar.py#L34-L47)

No mapa, `0` é parede, `128` é desconhecido e `255` é livre. Trocamos `128` por `0`.

**Por quê:** no robô real, entrar numa área desconhecida pode significar bater. É mais seguro tratá-la como parede.

---

## 2. `create_potential_field`: custo perto das paredes
[astar.py:49-80](astar.py#L49-L80)

Cada célula recebe um custo: **alto perto da parede, baixo longe dela**. O A* vai somar esse custo a cada passo e, assim, prefere o meio dos corredores.

1. `distance_transform_edt` calcula a distância de cada célula até a parede mais próxima.
2. `wall_influence * exp(-dist / buffer_factor)` transforma essa distância em custo.
   - `wall_influence`: **intensidade** (custo em cima da parede).
   - `buffer_factor`: **alcance** (até onde a parede ainda pesa).

**Por que exponencial:** cai de forma suave, sem curvas bruscas no caminho, e é finita na parede, então corredores estreitos continuam possíveis. Descartamos `1/dist`, que divide por zero na parede, e `max(0, buffer - dist)`, que tem uma quina e é menos suave.

⚠️ Se o custo das paredes for alto demais em relação ao custo de andar (principalmente `wall_influence`), o A* pode preferir um desvio longo a uma passagem estreita. Como o custo nunca é infinito, se não houver outra rota ele ainda passa pelo estreito.

---

