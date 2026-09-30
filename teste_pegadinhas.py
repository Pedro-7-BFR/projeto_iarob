"""
Labirinto de PEGADINHAS: lugares que parecem bons caminhos mas não são.
O robô descobre o mapa enquanto anda (simulacao.py), então só percebe a
pegadinha quando chega perto o bastante para o laser ver.

Pegadinhas:
  1. "U" virado para o início, bem na linha reta até o objetivo.
     O caminho mais curto no desconhecido passa por dentro dele.
  2. Corredor de cima que aponta direto para o objetivo, mas é fechado no fim.
  3. Duas aberturas na barreira do meio: a de cima parece mais perto, mas dá num beco.

Rodar:  python teste_pegadinhas.py
"""
from simulacao import parede, mapa_vazio, simular

INICIO = (45, 8)
OBJETIVO = (45, 170)
ALCANCE_LASER = 15

mapa = mapa_vazio(90, 180)

# 1. "U" aberto para a esquerda, na frente do início
parede(mapa, (30, 25), (65, 25))     # braço de cima
parede(mapa, (30, 65), (65, 65))     # braço de baixo
parede(mapa, (65, 25), (65, 65))     # fundo do U

# 2. corredor de cima: parece ir direto ao objetivo, mas é fechado
parede(mapa, (80, 15), (175, 15))    # piso do corredor de cima (separa ele do resto)
parede(mapa, (80, 1), (80, 8))       # entrada estreita... (deixa passar entre linha 8 e 15)
parede(mapa, (175, 1), (175, 15))    # fim fechado

# 3. barreira do meio com duas aberturas
parede(mapa, (110, 15), (110, 88))   # barreira vertical
mapa[28:36, 109:112] = 255           # abertura de cima (parece mais perto do objetivo)...
parede(mapa, (112, 22), (140, 22))   # ...mas leva a um bolsão fechado
parede(mapa, (112, 42), (140, 42))
parede(mapa, (140, 22), (140, 42))
mapa[74:84, 109:112] = 255           # abertura de baixo: a passagem verdadeira

simular(mapa, INICIO, OBJETIVO, ALCANCE_LASER, titulo="Labirinto de pegadinhas")
