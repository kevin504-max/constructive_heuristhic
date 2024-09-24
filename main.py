import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Solicita ao usuário que insira o nome da cidade
cidade = input("Insira o nome da cidade (ex: Belo Horizonte): ")
print(f"Iniciando o processo para a cidade: {cidade}, Minas Gerais, Brazil")

cidade_MG = cidade + ", Minas Gerais, Brazil"

# Baixa o grafo da rede viária da cidade
print("Baixando o grafo da rede viária...")
G = ox.graph_from_place(cidade_MG, network_type='drive')
print("Grafo baixado com sucesso!")

# Captura pontos de interesse (POIs) da cidade
print("Buscando pontos de interesse na cidade...")
pois = ox.features_from_place(cidade_MG, tags={'amenity': True})

# Solicita ao usuário que insira o número de POIs a serem selecionados
num_pois = input("Insira o número de pontos de interesse que deseja selecionar: ")
num_pois = int(num_pois)

# Seleciona um número aleatório de POIs
random_pois = pois.sample(n=num_pois)

# Exibe as coordenadas e o tipo dos POIs selecionados
print("Pontos de interesse encontrados:")
poi_coords = []
for index, row in random_pois.iterrows():
    coords = row['geometry'].centroid
    poi_name = row.get('name', 'Nome desconhecido')
    if pd.isna(poi_name) or poi_name == 'nan':
        poi_name = 'Nome desconhecido'
    poi_coords.append((coords.y, coords.x, poi_name))

# Exibe a ordem e o nome dos POIs
for i, (poi_lat, poi_lon, poi_name) in enumerate(poi_coords):
    print(f"{i + 1}: {poi_name} - ({poi_lat}, {poi_lon})")

# Solicita ao usuário que insira as coordenadas de origem
orig_coords = input("Insira as coordenadas da origem (latitude, longitude): ")
orig_lat, orig_lon = map(float, orig_coords.split(','))

# Encontra o nó mais próximo da origem
orig_node = ox.distance.nearest_nodes(G, orig_lon, orig_lat)

# Inicializa a lista de rotas e o número de carteiros
num_carteiros = int(input("Insira o número de carteiros (n): "))
rotas = [[] for _ in range(num_carteiros)]

# Implementa uma heurística construtiva
def heuristica_construtiva(G, orig_node, poi_coords, num_carteiros):
    remaining_pois = poi_coords.copy()
    # Distribui os POIs entre os carteiros
    for i in range(len(remaining_pois)):
        carteiro_index = i % num_carteiros  # Alterna entre os carteiros
        rotas[carteiro_index].append(remaining_pois[i])
    return rotas

# Gera as rotas usando a heurística construtiva
rotas = heuristica_construtiva(G, orig_node, poi_coords, num_carteiros)

# Plota todas as rotas para cada carteiro com cores diferentes
colors = plt.cm.viridis(np.linspace(0, 1, num_carteiros))  # Gera uma paleta de cores

# Plotando o grafo
fig, ax = ox.plot_graph(G, node_size=0, bgcolor='k', show=False)

# Lista para armazenar todas as rotas
all_routes = []

for i, rota in enumerate(rotas):
    if rota:
        print(f"Rota do carteiro {i + 1}:")
        total_route = []
        prev_node = orig_node
        for (poi_lat, poi_lon, poi_name) in rota:
            poi_node = ox.distance.nearest_nodes(G, poi_lon, poi_lat)
            route = ox.shortest_path(G, prev_node, poi_node, weight='length')
            total_route += route
            prev_node = poi_node

        # Filtra apenas nós válidos e pares de nós com arestas
        valid_route = []
        for j in range(len(total_route) - 1):
            u = total_route[j]
            v = total_route[j + 1]
            if G.has_node(u) and G.has_node(v) and G.has_edge(u, v):
                valid_route.append(u)

        if valid_route:
            print(f"Adicionando a rota do carteiro {i + 1} à lista de rotas...")
            all_routes.append((valid_route, colors[i]))  # Armazena a rota e a cor correspondente
            print(f"Rota do carteiro {i + 1} adicionada com sucesso!")
        else:
            print(f"Nenhuma rota válida encontrada para o carteiro {i + 1}.")

# Plotar todas as rotas no mapa
for route, color in all_routes:
    print("Plotando o grafo e destacando as rotas...") 
    ox.plot_graph_route(G, route, route_linewidth=6, node_size=0, ax=ax, edge_color=color)
    
plt.title(f"Rotas dos Carteiros")
plt.show()
print("Processo concluído!")
