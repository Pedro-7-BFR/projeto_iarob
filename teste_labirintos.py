"""
Labirintos de teste com o objetivo CONHECIDO, para testar situações que os
mapas do professor (map1..map5) não cobrem: corredor estreito e obstáculo extra.

Rodar:  python teste_labirintos.py
Os mapas já são criados no formato que o AStarPathfinder espera depois do prep_map:
0 = parede, 255 = livre (aqui não há desconhecido).
"""
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from astar import AStarPathfinder

LINHAS, COLUNAS = 60, 100
INICIO, OBJETIVO = (30, 10), (30, 90)


def sala_vazia():
    """Sala livre com borda de parede."""
    mapa = np.full((LINHAS, COLUNAS), 255, dtype=np.uint8)
    mapa[0, :] = mapa[-1, :] = mapa[:, 0] = mapa[:, -1] = 0
    return mapa


def parede_com_passagens(passagens):
    """Parede vertical no meio da sala, com aberturas nas linhas indicadas."""
    mapa = sala_vazia()
    mapa[:, 48:52] = 0
    for linha_ini, linha_fim in passagens:
        mapa[linha_ini:linha_fim, 48:52] = 255
    return mapa


def obstaculo_no_meio():
    """Sala livre com um bloco no meio, no caminho em linha reta."""
    mapa = sala_vazia()
    mapa[22:38, 44:56] = 0
    return mapa


CENARIOS = {
    # única saída é uma passagem de 5 células: tem que passar por ela
    "corredor estreito (5)": parede_com_passagens([(28, 33)]),
    # passagem de 2 células: bem apertada, ainda tem que passar
    "corredor muito estreito (2)": parede_com_passagens([(29, 31)]),
    # passagem estreita no caminho reto ou larga lá embaixo (mais longe)
    "estreito perto x largo longe": parede_com_passagens([(28, 32), (46, 58)]),
    # bloco no meio do caminho: tem que desviar mantendo distância
    "obstáculo no meio": obstaculo_no_meio(),
}


def maior_virada(pontos):
    """Maior ângulo de virada (graus) entre segmentos consecutivos."""
    maior = 0
    for k in range(1, len(pontos) - 1):
        a = math.atan2(pontos[k][0] - pontos[k - 1][0], pontos[k][1] - pontos[k - 1][1])
        b = math.atan2(pontos[k + 1][0] - pontos[k][0], pontos[k + 1][1] - pontos[k][1])
        d = abs(math.degrees(b - a)) % 360
        maior = max(maior, min(d, 360 - d))
    return round(maior)


fig, eixos = plt.subplots(2, 2, figsize=(12, 7))
for eixo, (nome, mapa) in zip(eixos.flat, CENARIOS.items()):
    astar = AStarPathfinder(mapa, INICIO, OBJETIVO, wall_influence=10.0, buffer_factor=3.0)
    came_from, fim = astar.find_path()

    eixo.imshow(mapa, cmap="gray")
    eixo.set_title(nome)
    eixo.axis("off")

    if not fim:
        print(f"{nome}: NÃO encontrou caminho")
        continue

    caminho = astar.know_path(astar.reconstruct_path(came_from, fim))
    simplificado = astar.simplify_path(caminho)

    # distância de cada célula até a parede, para medir a folga do caminho
    dist_parede = distance_transform_edt(mapa)
    folga = min(dist_parede[c] for c in caminho)

    print(f"{nome}: chegou ao objetivo? {astar.GOAL_REACHEABLE} | "
          f"{len(simplificado)} pontos | menor distância da parede {folga:.1f} células | "
          f"maior virada {maior_virada(simplificado)}°")

    l, c = zip(*caminho)
    eixo.plot(c, l, color="magenta", linewidth=1)
    ls, cs = zip(*simplificado)
    eixo.plot(cs, ls, "r--", linewidth=2)
    eixo.scatter([INICIO[1]], [INICIO[0]], color="green", s=60)
    eixo.scatter([OBJETIVO[1]], [OBJETIVO[0]], color="blue", s=60)

plt.tight_layout()
if len(sys.argv) > 1:
    plt.savefig(sys.argv[1], dpi=110)   # opcional: salvar a imagem em vez de abrir a janela
else:
    plt.show()
