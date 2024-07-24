import os
import time
import csv

def ler_matriz_arquivo(caminho_arquivo):
    """Lê uma matriz de um arquivo de texto e retorna a matriz como uma lista de listas"""
    with open(caminho_arquivo, 'r') as arquivo:
        linhas = arquivo.readlines()
        tamanho = int(linhas[0].strip())
        matriz = []

        for i in range(1, tamanho + 1):
            linha = list(map(int, linhas[i].strip().split()))
            matriz.append(linha)
        
        return matriz

def dijkstra(matriz, origem):
    """Executa o algoritmo de Dijkstra para encontrar a menor distância da origem para todos os outros nós"""
    tamanho = len(matriz)
    distancias = [float('inf')] * tamanho
    distancias[origem] = 0
    visitados = [False] * tamanho

    for _ in range(tamanho):
        min_dist = float('inf')
        min_index = -1

        for i in range(tamanho):
            if not visitados[i] and distancias[i] < min_dist:
                min_dist = distancias[i]
                min_index = i

        if min_index == -1:
            break

        visitados[min_index] = True

        for v in range(tamanho):
            if matriz[min_index][v] != 0 and not visitados[v]:
                if distancias[min_index] + matriz[min_index][v] < distancias[v]:
                    distancias[v] = distancias[min_index] + matriz[min_index][v]

    return distancias

def bellman_ford(matriz, origem):
    """Executa o algoritmo de Bellman-Ford para encontrar a menor distância da origem para todos os outros nós"""
    tamanho = len(matriz)
    distancias = [float('inf')] * tamanho
    distancias[origem] = 0

    # Relaxamento das arestas (tamanho - 1) vezes
    for _ in range(tamanho - 1):
        for u in range(tamanho):
            for v in range(tamanho):
                if matriz[u][v] != 0:
                    if distancias[u] + matriz[u][v] < distancias[v]:
                        distancias[v] = distancias[u] + matriz[u][v]
    
    # Verificação de ciclos negativos
    for u in range(tamanho):
        for v in range(tamanho):
            if matriz[u][v] != 0:
                if distancias[u] + matriz[u][v] < distancias[v]:
                    raise ValueError("O grafo contém um ciclo de peso negativo")
    
    return distancias

def processar_arquivos(pasta_entradas, caminho_csv):
    """Processa todos os arquivos na pasta de entradas, executa os algoritmos de Dijkstra e Bellman-Ford, e salva os resultados em um CSV"""
    if not os.path.exists(pasta_entradas):
        print(f"O diretório {pasta_entradas} não foi encontrado.")
        return
    
    arquivos = [f for f in os.listdir(pasta_entradas) if f.endswith('.txt')]

    # Abrir o arquivo CSV para escrita
    with open(caminho_csv, 'w', newline='') as csvfile:
        fieldnames = ['Número do Teste', 'Arquivo', 'Algoritmo', 'Tempo Total', 'Matriz de Distâncias Igual']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for arquivo in arquivos:
            caminho_arquivo = os.path.join(pasta_entradas, arquivo)
            matriz = ler_matriz_arquivo(caminho_arquivo)

            for teste in range(1, 8):  # Executa 7 testes para cada arquivo

                # Medir o tempo e calcular distâncias com Dijkstra
                start_time = time.time()
                distancias_dijkstra = dijkstra(matriz, 0)  # Começando do nó 0
                tempo_dijkstra = time.time() - start_time

                # Medir o tempo e calcular distâncias com Bellman-Ford
                start_time = time.time()
                try:
                    distancias_bellman_ford = bellman_ford(matriz, 0)  # Começando do nó 0
                    tempo_bellman_ford = time.time() - start_time

                    # Verificar se as matrizes de distâncias são iguais
                    matriz_distancias_igual = distancias_dijkstra == distancias_bellman_ford
                except ValueError as e:
                    distancias_bellman_ford = []
                    tempo_bellman_ford = time.time() - start_time
                    matriz_distancias_igual = False

                # Salvar os resultados no CSV para Dijkstra
                writer.writerow({
                    'Número do Teste': f'{arquivo}_Dijkstra_{teste}',
                    'Arquivo': arquivo,
                    'Algoritmo': 'Dijkstra',
                    'Tempo Total': tempo_dijkstra,
                    'Matriz de Distâncias Igual': matriz_distancias_igual
                })

                # Salvar os resultados no CSV para Bellman-Ford
                writer.writerow({
                    'Número do Teste': f'{arquivo}_Bellman-Ford_{teste}',
                    'Arquivo': arquivo,
                    'Algoritmo': 'Bellman-Ford',
                    'Tempo Total': tempo_bellman_ford,
                    'Matriz de Distâncias Igual': matriz_distancias_igual
                })

                # Imprimir os resultados
                print(f"Arquivo: {arquivo}")
                print(f"Teste: {teste}")
                print(f"Tempo Dijkstra: {tempo_dijkstra:.6f} segundos")
                print(f"Tempo Bellman-Ford: {tempo_bellman_ford:.6f} segundos")
                print(f"Matriz de Distâncias Igual: {matriz_distancias_igual}")
                print(f"Distâncias Dijkstra: {distancias_dijkstra}")
                print(f"Distâncias Bellman-Ford: {distancias_bellman_ford}")
                print()

# Caminho para a pasta de entradas (substitua pelo caminho correto)
pasta_entradas = '/home/gabriel/Documentos/Mestrado/Matérias/2 - EDAA/1 - Analise de Algoritmos/Avaliação-EDAA-2Bimestre/avaliacaoedaa-2bimestre/entradas'
# Caminho para o arquivo CSV de saída (substitua pelo caminho correto)
caminho_csv = '/home/gabriel/Documentos/Mestrado/Matérias/2 - EDAA/1 - Analise de Algoritmos/Avaliação-EDAA-2Bimestre/avaliacaoedaa-2bimestre/resultados.csv'

# Processa os arquivos na pasta de entradas e salva os resultados em um CSV
processar_arquivos(pasta_entradas, caminho_csv)
