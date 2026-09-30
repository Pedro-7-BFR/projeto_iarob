"""
Labirinto aproximado da lousa, simulando o robô DESCOBRINDO o mapa enquanto anda
(a simulação está em simulacao.py).

As proporções foram estimadas a partir do desenho, que NÃO está em escala.
Rodar:  python teste_lousa.py
"""
from simulacao import parede, mapa_vazio, simular

INICIO = (110, 13)      # fora do labirinto, embaixo à esquerda
OBJETIVO = (13, 140)    # saída da direita
ALCANCE_LASER = 15      # em células (quanto menor, mais o robô precisa ir descobrindo)

mapa = mapa_vazio(116, 146)

parede(mapa, (6, 1), (6, 106))       # parede da esquerda
parede(mapa, (6, 1), (145, 1))       # parede de cima
parede(mapa, (95, 27), (145, 27))    # linha de baixo da saída da direita
parede(mapa, (95, 27), (95, 114))    # parede da direita
mapa[28:, 96:] = 0                   # fora do labirinto (abaixo da saída)

# "T de cabeça para baixo" (⊥), maior: base oca e aberta embaixo + vão estreito subindo
parede(mapa, (20, 75), (20, 114))    # base: lado esquerdo
parede(mapa, (20, 75), (57, 75))     # base: topo (esquerda)
parede(mapa, (72, 75), (95, 75))     # base: topo (direita)
parede(mapa, (57, 75), (57, 45))     # vão estreito: parede esquerda
parede(mapa, (72, 75), (72, 45))     # vão estreito: parede direita
parede(mapa, (71, 20), (58, 37))     # obstáculo diagonal

# obstáculos extras para deixar mais difícil
parede(mapa, (6, 45), (45, 45))      # prateleira saindo da parede esquerda
parede(mapa, (30, 60), (45, 60))     # obstáculo no corredor da esquerda
parede(mapa, (40, 1), (40, 32))      # parede pendurada no teto (forma um beco em cima à esquerda)
parede(mapa, (40, 32), (58, 37))     # fecha a passagem por cima do obstáculo diagonal
parede(mapa, (85, 1), (85, 19))      # parede pendurada perto da saída (aperta a passagem)

simular(mapa, INICIO, OBJETIVO, ALCANCE_LASER, titulo="Labirinto da lousa (aproximado)")
