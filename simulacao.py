"""
Simulação do robô DESCOBRINDO o mapa enquanto anda (usada pelos testes).

- mapa_real: o labirinto de verdade (o robô não conhece).
- conhecido: o que o robô já viu. Começa todo desconhecido (128) e vai sendo
  revelado por um "laser" simulado: raios em 360°, com alcance limitado, que
  param na primeira parede (não enxerga através dela).
- A cada ciclo: planeja com o que conhece (AStarPathfinder), anda o trecho
  devolvido pelo know_path, enxerga de novo e replaneja. Igual ao navegador real.
"""
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from astar import AStarPathfinder


def parede(mapa, ponto1, ponto2, espessura=2):
    """Desenha uma parede (valor 0) de ponto1 a ponto2. Pontos em (coluna, linha), como no cv2."""
    cv2.line(mapa, ponto1, ponto2, 0, espessura)


def mapa_vazio(linhas, colunas):
    """Mapa todo livre, com borda de parede."""
    mapa = np.full((linhas, colunas), 255, dtype=np.uint8)
    mapa[0, :] = mapa[-1, :] = mapa[:, 0] = mapa[:, -1] = 0
    return mapa


def enxergar(conhecido, mapa_real, posicao, alcance):
    """Revela no mapa conhecido o que o laser vê a partir da posição (raios em 360°)."""
    linhas, colunas = mapa_real.shape
    l0, c0 = posicao
    for grau in range(360):
        dl, dc = math.sin(math.radians(grau)), math.cos(math.radians(grau))
        for r in range(alcance + 1):
            l, c = round(l0 + r * dl), round(c0 + r * dc)
            if not (0 <= l < linhas and 0 <= c < colunas):
                break
            conhecido[l, c] = mapa_real[l, c]   # agora o robô sabe o que tem ali
            if mapa_real[l, c] == 0:            # bateu na parede: o raio para
                break


def simular(mapa_real, inicio, objetivo, alcance_laser=15, max_ciclos=40,
            titulo="", wall_influence=10.0, buffer_factor=3.0):
    """Roda o ciclo planeja → anda → enxerga até chegar, travar ou bater. Mostra o resultado."""
    conhecido = np.full(mapa_real.shape, 128, dtype=np.uint8)
    posicao = inicio
    enxergar(conhecido, mapa_real, posicao, alcance_laser)
    trajeto = [posicao]
    pontos_de_parada = [posicao]
    chegou = False

    for ciclo in range(1, max_ciclos + 1):
        astar = AStarPathfinder(conhecido, posicao, objetivo,
                                wall_influence=wall_influence, buffer_factor=buffer_factor)
        came_from, fim = astar.find_path()
        if not fim:
            print(f"ciclo {ciclo}: NÃO encontrou caminho a partir de {posicao}")
            break

        trecho = astar.know_path(astar.reconstruct_path(came_from, fim))
        simplificado = astar.simplify_path(trecho)
        if len(trecho) <= 1:
            print(f"ciclo {ciclo}: travado em {posicao} (não consegue avançar)")
            break

        # anda o trecho, enxergando pelo caminho (a cada 5 células)
        bateu = False
        for k, celula in enumerate(trecho[1:], start=1):
            if mapa_real[celula] == 0:
                print(f"ciclo {ciclo}: BATEU na parede em {celula}!")
                bateu = True
                break
            trajeto.append(celula)
            if k % 5 == 0:
                enxergar(conhecido, mapa_real, celula, alcance_laser)
        posicao = trajeto[-1]
        enxergar(conhecido, mapa_real, posicao, alcance_laser)
        pontos_de_parada.append(posicao)

        print(f"ciclo {ciclo}: andou até {posicao} | {len(simplificado)} pontos no trecho | "
              f"chegou? {astar.GOAL_REACHEABLE}")
        if bateu:
            break
        if astar.GOAL_REACHEABLE:
            chegou = True
            break

    # ---------------- resultado ----------------
    dist_parede = distance_transform_edt(mapa_real)
    passos = sum(math.dist(a, b) for a, b in zip(trajeto, trajeto[1:]))
    print(f"\nchegou? {chegou} | ciclos: {len(pontos_de_parada) - 1} | distância andada: {passos:.0f} células | "
          f"menor distância da parede: {min(dist_parede[c] for c in trajeto):.1f} células")

    fig, (esq, dir_) = plt.subplots(1, 2, figsize=(14, 6))
    for eixo, mapa, sub in [(esq, mapa_real, "Labirinto real + trajeto"),
                            (dir_, conhecido, "O que o robô conheceu (cinza = nunca viu)")]:
        eixo.imshow(mapa, cmap="gray", vmin=0, vmax=255)
        l, c = zip(*trajeto)
        eixo.plot(c, l, "r-", linewidth=2)
        lp, cp = zip(*pontos_de_parada)
        eixo.scatter(cp, lp, color="orange", s=30, zorder=3, label="parou e replanejou")
        eixo.scatter([inicio[1]], [inicio[0]], color="green", s=80, zorder=4, label="Início")
        eixo.scatter([objetivo[1]], [objetivo[0]], color="blue", s=80, zorder=4, label="Objetivo")
        eixo.set_title(sub)
        eixo.axis("off")
    esq.legend(loc="lower right")
    if titulo:
        fig.suptitle(titulo)
    plt.tight_layout()
    plt.show()
    return chegou, trajeto
