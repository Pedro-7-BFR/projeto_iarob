# Implementação

**Objetivo:** o robô real atravessa o labirinto sem bater nas paredes, com movimento suave, passando também por lugares estreitos.

---

## 1. `preprocess_map`: desconhecido vira livre (para o planejamento)
[astar.py:34-47](astar.py#L34-L47)

No mapa, `0` é parede, `128` é desconhecido e `255` é livre. Trocamos `128` por `255`.

**Por quê:** o robô tem conhecimento incompleto do labirinto. Ele anda até a borda do que conhece, o sensor revela mais, e ele replaneja. Em todos os mapas de teste, o objetivo `(60, 120)` fica no desconhecido. Com `128 → 0`, o A* não encontrou caminho em nenhum dos 5 mapas. Com `128 → 255`, encontrou em todos.

**Livre não significa conhecido.** Depois da conversão, `255` no `self.map_array` inclui o livre conhecido e o desconhecido. A diferença não se perde, porque o mapa original fica guardado:

| Mapa | Desconhecido | Usado por | Para quê |
|---|---|---|---|
| `self.map_array` | `255` (livre) | `find_path` (A*) | planejar a rota até o objetivo |
| `self.map` | `128` (original) | `know_path` | cortar o caminho onde entra no desconhecido |

Assim, o A* planeja pelo desconhecido, mas o robô só anda no que é conhecido.

**Diferença da docstring:** a docstring original diz "convertendo valores intermediários para obstáculos". Não seguimos isso ao pé da letra porque, com o desconhecido como obstáculo, o objetivo fica inalcançável. Além disso, o próprio template indica que o A* atravessa o desconhecido: o `know_path` existe para "remover trechos desconhecidos" do caminho, e o `run` o chama logo após o A*.

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

---

## 4. `find_path`: o A*
[astar.py:98-161](astar.py#L98-L161)

**Estruturas:**
- `fila` (`heapq`): fila de prioridade que sempre entrega a célula de menor `f = g + h`, a mais promissora.
- `g_score`: custo real para chegar em cada célula desde o início.
- `came_from`: de qual célula viemos para chegar em cada uma (rastro usado pelo `reconstruct_path`).

**Laço:** tira da fila a célula de menor `f`. Se for o objetivo, termina. Senão, testa os 8 vizinhos.

**Peneira dos vizinhos** (`continue` descarta o vizinho):
1. fora do mapa;
2. parede;
3. diagonal que corta quina: o passo de `(l, c)` para `(l+dl, c+dc)` só vale se `(l+dl, c)` e `(l, c+dc)` não forem parede.

**Custo:** `novo_g = g_score[atual] + passo + potential_field[vizinho]`, com passo 1 (reto) ou √2 (diagonal), mais a "multa" por estar perto da parede.

**Anotar:** só se o vizinho é novo ou se o caminho de agora é mais barato. Nesse caso, atualiza `g_score` e `came_from` e coloca o vizinho na fila com `f = novo_g + heuristic(vizinho, objetivo)`.

O A* não "anda" de célula em célula. Ele anota os vizinhos na fila, e o próximo `atual` é sempre o de menor `f` da fila inteira. É isso que garante o caminho mais barato.

---


## 5. `reconstruct_path`: seguir as migalhas
[astar.py:162-185](astar.py#L162-L185)

O `find_path` não devolve o caminho, só o `came_from` (de onde viemos para chegar em cada célula). Para montar o caminho:

1. começa no objetivo;
2. pergunta ao `came_from` "de onde vim?" e anda uma célula para trás, repetindo;
3. para quando chega no início, a única célula que não está no `came_from`;
4. inverte a lista, que foi montada de trás para frente, para ficar **início → objetivo**.

---

## 6. `know_path`: o robô só anda no conhecido
[astar.py:189-219](astar.py#L189-L219)

O A* planeja atravessando o desconhecido, mas o robô não pode andar por ali. O `know_path` percorre o caminho e **corta na primeira célula a menos de `unknown_margin` células do desconhecido**.

- Usa o `self.map` (original, onde o desconhecido ainda é `128`), e não o `self.map_array` (onde virou `255`).
- `distance_transform_edt(self.map != 128)` dá a distância de cada célula até o desconhecido, com a mesma função do campo potencial.
- Se nenhuma célula do caminho ficar perto do desconhecido, o caminho chega ao objetivo, e `GOAL_REACHEABLE = True`.
- Se cortou, o robô anda até perto da borda, o mapa cresce e o navegador chama o `run` de novo.
- `path[:max(i, 1)]` mantém pelo menos o ponto inicial, para nunca devolver uma lista vazia.

**Por que uma margem, e não parar colado na borda:** o desconhecido pode esconder uma parede. O campo potencial não "enxerga" paredes ali, então o caminho pode chegar colado na borda. Se o robô parasse ali e houvesse parede, poderia bater antes de replanejar. Parando a `unknown_margin` células (padrão 3), o sensor ainda enxerga à frente sem risco. O valor deve ser ajustado no robô real.

Nos mapas de teste, o caminho para a ~3 células do desconhecido, e o trecho conhecido cresce do `map1` ao `map5` (37 → 216 células).

---
