import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import random
import pandas as pd  # Importa pandas para verificar NaN

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
pois = ox.features_from_place(cidade_MG, tags={'amenity': True})  # Atualizado para a nova função

# Seleciona um número aleatório de POIs (por exemplo, 5)
num_pois = 5
random_pois = pois.sample(n=min(num_pois, len(pois)))  # Evita amostrar mais do que existem

# Exibe as coordenadas e o tipo dos POIs selecionados
print("Pontos de interesse encontrados:")
poi_coords = []
for index, row in random_pois.iterrows():
    index_position = random_pois.index.get_loc(index)  # Obtém a posição do índice
    coords = row['geometry'].centroid  # Obtém o centróide do POI
    poi_name = row.get('name', 'Nome desconhecido')  # Obtém o nome do POI
    if pd.isna(poi_name):  # Verifica se o nome é NaN
        poi_name = 'Nome desconhecido'
    poi_coords.append((coords.y, coords.x, poi_name))
    print(f"{index_position + 1}: {coords.y}, {coords.x} - {poi_name}")

# Solicita ao usuário que insira as coordenadas de origem
orig_coords = input("Insira as coordenadas da origem (latitude, longitude): ")
orig_lat, orig_lon = map(float, orig_coords.split(','))
print(f"Coordenadas de origem recebidas: ({orig_lat}, {orig_lon})")

# Encontra o nó mais próximo da origem
print("Encontrando o nó mais próximo da origem...")
orig_node = ox.distance.nearest_nodes(G, orig_lon, orig_lat)
print(f"Nó de origem encontrado: {orig_node}")

# Inicializa uma lista para armazenar a rota total
total_route = []

# Loop pelos POIs selecionados para calcular a rota de um a outro
for i, (poi_lat, poi_lon, poi_name) in enumerate(poi_coords):
    # Encontra o nó do POI
    poi_node = ox.distance.nearest_nodes(G, poi_lon, poi_lat)
    print(f"Nó do POI '{poi_name}' encontrado: {poi_node}")

    # Calcula o caminho mais curto entre a origem e o POI
    if i == 0:  # Para o primeiro POI, usa a origem
        route = ox.shortest_path(G, orig_node, poi_node, weight='length')
    else:  # Para os demais, usa o último nó de destino como origem
        route = ox.shortest_path(G, prev_poi_node, poi_node, weight='length')

    # Verifica se a rota é válida
    if route and len(route) > 0:
        # Adiciona a rota ao total
        total_route += route
        print(f"Caminho para '{poi_name}' calculado com sucesso!")
        prev_poi_node = poi_node  # Atualiza o nó anterior
    else:
        print(f"A rota para '{poi_name}' não foi encontrada.")

# Verifica se a rota total é válida
if total_route:
    # Filtra apenas nós válidos e pares de nós com arestas
    valid_route = []
    
    for i in range(len(total_route) - 1):
        u = total_route[i]
        v = total_route[i + 1]
        # Verifica se há arestas entre os nós
        if G.has_node(u) and G.has_node(v) and G.has_edge(u, v):
            valid_route.append(u)
    
    # Adiciona o último nó se ele for válido
    if valid_route and total_route[-1] not in valid_route:
        valid_route.append(total_route[-1])

    if valid_route:
        # Plota o grafo e destaca a rota total
        print("Plotando o grafo e destacando a rota total...")
        print("Aguarde um momento...")
        fig, ax = ox.plot_graph_route(G, valid_route, route_linewidth=6, node_size=0, bgcolor='k')
        plt.show()
    else:
        print("Nenhuma rota válida encontrada para plotar.")
else:
    print("Nenhuma rota válida encontrada para plotar.")

print("Processo concluído!")
