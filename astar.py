import heapq
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import math

class AStarPathfinder:
    def __init__(self, map_array: np.array, start: tuple, goal: tuple, wall_influence=5.0, buffer_factor=2.0):
        """
        Inicializa o A* com mapa, ponto inicial, objetivo e parâmetros de influência.

        Args:
            map_array (np.array): Mapa binário (obstáculos e caminho livre).
            start (tuple): Ponto inicial (linha, coluna).
            goal (tuple): Ponto objetivo (linha, coluna).
            wall_influence (float): Peso da proximidade das paredes.
            buffer_factor (float): Escala da influência das paredes.
        """
        self.start = start
        self.goal = goal
        self.wall_influence = wall_influence
        self.buffer_factor = buffer_factor
        self.GOAL_REACHEABLE = False  
        
        # Prepara o mapa, expandindo suas bordas e ajustando o array.
        self.map = map_array.copy()
        self.map_array = self.preprocess_map(map_array)

        # Cria um campo potencial baseado no mapa para influenciar o caminho.
        self.potential_field = self.create_potential_field()
        

    def preprocess_map(self, map_array: np.array) -> np.array:
        """
        Ajusta o mapa, convertendo valores intermediários para obstáculos.

        Args:
            map_array (np.array): Mapa original.

        Returns:
            np.array: Mapa processado.
        """

        mapa = map_array.copy()
        mapa[mapa == 128] = 255
        return mapa

    def create_potential_field(self) -> np.array:
        
        # Gera campo potencial com base na distância de obstáculos.

        # Cada célula do mapa recebe um custo que é alto perto das paredes e
        # diminui conforme se afasta delas. O A* soma esse custo ao custo de
        # cada passo, então passa a preferir caminhos pelo meio dos corredores,
        # evitando que o robô real encoste nas bordas do labirinto.

        # - wall_influence: custo máximo, aplicado em cima da parede (dist = 0).
        #   Quanto maior, mais o A* "foge" das paredes.
        # - buffer_factor: alcance da influência, em células. Quanto maior,
        #   mais longe da parede o custo ainda é relevante. Se for grande demais,
        #   passagens estreitas ficam caras e o robô pode evitá-las.

        # Para cada célula, calcula a quantas células ela está da parede mais próxima.
        # Exemplo (0 = parede, 255 = livre):
        
        #   map_array:          dist:
        #   0   0   0   0       0  0  0  0
        #   0 255 255 255       0  1  1  1
        #   0 255 255 255       0  1  2  2
        
        # Colado na parede -> 1; mais para o meio -> 2; na própria parede -> 0.
        dist = distance_transform_edt(self.map_array)


        # Converte distância em custo usando uma exponencial decrescente
        # (escolhida por ser suave e nunca infinita, permitindo passar em
        # corredores estreitos). Perto da parede o custo é alto; longe, tende a zero.
        # wall_influence = 10 | buffer_factor = 3
        return self.wall_influence * np.exp(-dist / self.buffer_factor)

    def heuristic(self, a: tuple, b: tuple) -> float:
        """
        Calcula a heurística entre dois pontos.
        - a, b são tuplas
        """

        dx = a[0] - b[0]    # diferença nas linhas
        dy = a[1] - b[1]    # diferença nas colunas

        # considerando 8 direçõess (pode se mover na diagonal 45°):
        # - Euclidiana (linha reta): nunca superestima com 8 direções.
        # - Manhattan superestimaria nas diagonais (ex.: 6 contra 4,24 real).

        return math.sqrt(dx**2 + dy**2)


    def find_path(self):
        
        # Executa o algoritmo A* para encontrar caminho até o objetivo.

        fila = []
        h = self.heuristic(self.start, self.goal)
        heapq.heappush(fila, (h, self.start))

        # heapq.heappush(fila, item) coloca um item na fila.
        # heapq.heappop(fila) tira e devolve o menor item.

        g_score = {self.start: 0}   # dicionário que guarda quanto custa chegar em cada célula mapeada até ali
        came_from = {}  # guarda de onde cada célula veio, rastro de migalhas

        direcoes = [(-1,0), (1,0), (0,-1), (0,1), # retos
                    (-1,-1), (-1, 1), (1, -1), (1,1)] #diagonais

        # tamanho do mapa (shape devolve tupla)
        linhas, colunas = self.map_array.shape 

        while fila:  # enquanto houver células para explorar
            f, atual = heapq.heappop(fila)

            if atual == self.goal:
                return came_from, atual

            for dl, dc in direcoes:
                vizinho = (atual[0] + dl, atual[1] + dc)

                # verificar se é fora do mapa
                if not (0 <= vizinho[0] < linhas and 0 <= vizinho[1] < colunas):
                    continue

                # verificar se é parede
                if self.map_array[vizinho] == 0:
                    continue

                passo = 1

                if dl != 0 and dc != 0: # verifica se o movimento vai p/ diagonal
                    # verifica em x
                    passo = math.sqrt(2)
                    raspaox = (atual[0] + dl, atual[1])
                    raspaoy = (atual[0], atual[1] + dc)

                    # verifica se os vizinhos de raspao da diagonal é parede
                    if self.map_array[raspaox] == 0 or self.map_array[raspaoy] == 0:
                        continue # pula pro proximo vizinho

                # quanto custa chegar nesse novo vizinho
                # - quanto custou chegar até onde ele está
                # - quanto custa esse passo
                # - a multa por estar proximo da parede
                novo_g = g_score[atual] + passo + self.potential_field[vizinho]

                if vizinho not in g_score or novo_g < g_score[vizinho]:
                    g_score[vizinho] = novo_g # custo para chegar no vizinho
                    came_from[vizinho] = atual # de onde veio, celula "pai"
                    f = novo_g + self.heuristic(vizinho, self.goal) # custo total estimado
                    heapq.heappush(fila, (f, vizinho)) # coloca o vizinho na fila para ser explorado depois

        print("Caminho não encontrado")
        return None, None

    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        """
        Reconstrói o caminho a partir do ponto final até o inicial.
        
        Args:
            came_from (dict): O dicionário de predecessores no caminho.
            current (tuple): O ponto final (objetivo).
        
        Returns:
            list: Lista de tuplas com caminho reconstruído.
        """

        # começa pelo objetivo e segue as "migalhas" de trás para frente
        caminho = [current]

        # o início é a única célula que não está no came_from (não veio de lugar nenhum),
        # então o laço para quando chega nele
        while current in came_from:
            current = came_from[current]   # anda uma célula para trás
            caminho.append(current)

        # a lista foi montada do objetivo até o início; inverte para ficar início → objetivo
        caminho.reverse()
        return caminho

    def know_path(self, path: list) -> list:
        """
        Remove trechos desconhecidos e ajusta o caminho, se necessário.

        Args:
            path (list): Caminho completo.

        Returns:
            list: Caminho ajustado.
        """
        return None

    def simplify_path(self, path: list) -> list:
        """
        Simplifica o caminho removendo direções repetidas.
        
        Args:
            path (list): Caminho completo.

        Returns:
            list: Caminho simplificado.
        """
        return None

    def plot_path(self, path: list, simplified_path: list):
        """
        Exibe o mapa com o caminho completo e o simplificado.

        Args:
            path (list): O caminho completo encontrado.
            simplified_path (list): O caminho simplificado encontrado.
        """
        simplified_path = self.simplify_path(path)

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.scatter(self.start[1], self.start[0], color='green', s=100, label='Início')
        plt.scatter(self.goal[1], self.goal[0], color='blue', s=100, label='Objetivo')

        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, color='magenta', linewidth=1, label='Caminho Completo')
            simp_x, simp_y = zip(*simplified_path)
            plt.plot(simp_y, simp_x, color='red', linewidth=2, linestyle='--', label='Caminho Simplificado')
        else:
            plt.title("Caminho não encontrado")

        plt.legend()
        plt.axis('equal')
        plt.show()

    def run(self, show_path=True):
        """
        Essa função é chamada pelo navegador para executar o algoritmo A* e gerar o caminho.
        Executa o processo completo: busca, reconstrução, simplificação e visualização do caminho.
        
        Args:
            show_path (bool): Se True, exibe o caminho graficamente.

        Returns:
            list or None: Caminho simplificado ou None se não encontrado.
        """
        print("Iniciando busca pelo caminho...")
        came_from, final_node = self.find_path()

        if final_node:
            print("Reconstruindo caminho...")
            path = self.reconstruct_path(came_from, final_node)
            
            print("Robo não anda no disconhecido")
            path = self.know_path(path)

            print("Caminho encontrado, simplificando...")
            simplified_path = self.simplify_path(path)

            print("Plotando o caminho...")
            if show_path:
                self.plot_path(path, simplified_path)
            
            return simplified_path
        else:
            print("Nenhum caminho pôde ser encontrado.")
            return None


def prep_map(map_path: str) -> np.array:
    """
    Prepara o mapa carregando e processando a imagem de entrada.

    Args:
        map_path (str): O caminho do arquivo do mapa.

    Returns:
        np.array: O mapa processado como um array numpy.
    """
    map_array = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    map_array[map_array == 0] = 0
    map_array[map_array == 205] = 128
    map_array[map_array == 254] = 255
    map_array[(map_array >= 60) & (map_array != 128) & (map_array != 255)] = 0
    map_array = map_array.astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    map_array = cv2.morphologyEx(map_array, cv2.MORPH_OPEN, kernel)
    map_array = np.flipud(map_array)
    map_array = np.pad(map_array, ((0, 200), (0, 200)), 'constant', constant_values=128)
    return map_array


def main():
    map_array = prep_map('map5.pgm')
    astar = AStarPathfinder(map_array, (60, 20), (60, 120), wall_influence=10.0, buffer_factor=3.0)
    astar.run()


if __name__ == '__main__':
    main()
