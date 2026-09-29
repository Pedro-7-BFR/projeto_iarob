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

## 3. `heuristic`: distância euclidiana
[astar.py:82-95](astar.py#L82-L95)

**8 direções:** o A* pode andar reto (custo 1) e na diagonal (custo √2). O robô não anda de lado nem de ré: ele vira até ficar de frente para o próximo ponto e anda para frente. Com 8 direções, as viradas são menores (45° em vez de 90°) e o caminho em diagonal não faz zigue-zague, então o movimento fica mais suave.

**Por que euclidiana:** é a distância em linha reta, `√(dx² + dy²)`, e por isso nunca superestima o custo real com 8 direções. A Manhattan (`dx + dy`) superestimaria nas diagonais. Por exemplo, de (0,0) a (3,3) ela daria 6, enquanto o custo real é 4,24.

⚠️ A heurística precisa combinar com os vizinhos do `find_path`. Se mudarem as direções, é preciso rever a heurística.

**Regra da quina (para o `find_path`):** um passo diagonal de `(l, c)` para `(l+1, c+1)` só é permitido se `(l+1, c)` e `(l, c+1)` não forem obstáculo. Assim o robô não corta a quina da parede.

---

