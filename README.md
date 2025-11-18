# Courier Quest

## Cómo ejecutar
```bash
python -m src
```

## Requisitos
- Python 3.10+
- Instalar dependencias:
```bash
pip install -r requirements.txt
```

# Estructuras de datos utilizadas

## Pedidos

La lógica de los pedidos se divide en 3 archivos:

1. **job_logic**: Coordina el flujo de los pedidos en el juego y los dibuja en pantalla. Interactúa con `job_loader` y `job_manager` para manejar los pedidos.
2. **job_loader**: Carga y mantiene un repositorio **único** de objetos `Job` (para evitar duplicados).
3. **job_manager**: Administra las estructuras de datos de los pedidos y las operaciones clave.

### Estructuras encontradas en `job_logic`

- **pickup_markers**: Lista de puntos o marcadores en pantalla donde aparecen los puntos de “recogida” visibles.  
- **dropoff_markers**: Lista de puntos o marcadores en pantalla donde aparecerán los puntos de “entrega” aceptados.

### Estructuras encontradas en `job_loader`

- **_jobs**: Cuando se hace el *fetch* de datos, se almacenan todos los pedidos del API en este **diccionario**, donde se guardan los objetos `Job` así: `{ "id_job": job }`.  
  De esta manera se puede acceder al job por medio del id. Esto permite, en próximas estructuras, almacenar solo el id y no duplicar los objetos `Job` en cada estructura que se necesite en un orden diferente.

### Estructuras encontradas en `job_manager`

- **release_queue**: **Cola** de ids de jobs que controla el orden en el que se van lanzando pedidos para que el jugador los pueda aceptar o no.  
- **_base_ids_sorted**: **Lista** (copia) de todos los ids de los jobs; es útil para realizar varias acciones, pero principalmente para poder rellenar la cola `release_queue` cuando esta se queda sin pedidos.  
- **history**: **Lista** de todos los trabajos que se han lanzado; guarda información relevante en cada entrada, como el id del job, si se aceptó o no, y si se entregó a tiempo.  
- **inventory**: **Lista** de ids de los jobs que el jugador sí aceptó y debe entregar. Cuando se entregan, salen del inventario y se registran en el historial. Con la tecla **E** se puede entrar a una interfaz gráfica donde es posible ver y modificar el orden del inventario.

### Estructura de datos usada en player.py:
**pos_history**:
La estructura de datos usada para player fue una pila implementada mediante deque, esta cola se uso para almacenar las posiciones del jugador para poder hacer un deshacer o “undo( )” más adelante. Se uso una pila porque se requería devolver las últimas posiciones en las que estuvo y luego las primeras hasta llegar a la primera posición. 

## Mapa

La lógica del mapa se divide en dos partes: MapLoader, que se encarga de la información del mapa y sus características, y TileRenderer, cuya función es darle un aspecto agradable al mapa.

### Estructuras de datos usadas en TileRenderer:

- **cache**: Es un diccionario que se utiliza como caché con imágenes cargadas según el símbolo y la variante de la imagen. La clave en este diccionario es una tupla que contiene el símbolo y la variante específica para cada tile.
El acceso a la imagen es O(1) y no se necesitan operaciones de modificación o eliminación.

### Estructuras de datos usadas en MapLoader:

- **tiles**: Es una lista anidada que almacena, para cada posición del mapa, el símbolo y la variante de imagen correspondiente, por lo que `tiles[y][x] = [símbolo, variante]`.
El acceso directo a tiles por coordenadas tiene una complejidad algorítmica de O(1).
No se necesita modificar nada después de cargar el mapa.

## Clima

La lógica del clima se divide en tres clases: WeatherManager, que maneja la lógica del cambio de climas y la duración de cada uno; WeatherVisuals, encargado de mostrar los efectos visuales de cada clima; y Cloud, que es usado por WeatherVisuals para dar dinamismo a ciertos climas.

### Estructuras de datos usadas en WeatherVisuals:
- **clouds**: Es una lista que almacena los objetos Cloud que están activos.
- **wind_gusts**: Es una lista que almacena datos sobre los efectos de las ráfagas de viento; cada ráfaga es otra lista de atributos: `[x, y, speed, length, thickness, phase, freq, amp]`.

## Game Over Menu

### Estructuras encontradas en `game_over`

- **_rows**: Lista de diccionarios que guardan informacion sobre los 3 jugadores con los mejores puntajes y el jugador actual.

## Inteligencia Artificial
### Estructuras encontradas en easy_algorithm
- **dropoff_queue** y **pickup_queue** : Son una cola que almacena los trabajos válidos en el mapa y los desencola despues de cierto tiempo o si se entrego el pedido.
- **direction**: Es un vector de 2 posiciones que almacena una dirección valida a la que se moverá el enemigo.
  
Este algoritmo selecciona alguno de los trabajos validos en el mapa y los encola, despues de cierto tiempo, o si se entrego el pedido se vuelve a seleccionar uno al azar de la cola.
Luego selecciona alguna direccion valida al azar y mueve al enemigo a esta evitando los edificios.

### Estructuras encontradas en medium_algorithm

### Estructuras encontradas en hard_algorithm
- **dropoffs**: Es una lista de diccionarios que contienen información de cada entrega pendiente, incluyendo posición y tiempo restante.
- **pickups**: Es una lista de diccionarios que representan pedidos disponibles para recoger, con datos como posición y tiempo restante.
- **target_marker**: Es un diccionario que almacena el objetivo actual seleccionado (ya sea un dropoff o un pickup).
- **open_heap**: Es una lista utilizada como cola de prioridad (heap) que guarda tuplas de la forma `(f_score, (tx, ty))`. Se usa en A* para elegir el siguiente nodo a explorar.
- **came_from**: Es un diccionario donde cada clave es un tile `(tx, ty)` y su valor es el tile anterior. Se usa para reconstruir el camino final una vez que A* encuentra el destino.
- **g_score**: Es un diccionario que almacena el costo real acumulado para llegar a cada tile desde el inicio.
- **f_score**: Es un diccionario que guarda el costo total estimado para cada tile `(g_score + heurística)`. Se utiliza para priorizar nodos en el A*.
- **directions**: Es una lista de tuplas que representan los 8 movimientos posibles en el mapa.
- **visited**: Es un conjunto que almacena los tiles procesados por el algoritmo A*, evitando revisarlos nuevamente.
- **path**: Es una lista de tuplas que representan la ruta final desde el inicio hasta el objetivo después de reconstruirla.

Este algoritmo revisa primero si existen entregas pendientes (dropoffs). Si no las hay, busca un pedido disponible (pickup) entre todos los pedidos del mapa, tomando en cuenta la cercanía al enemigo y el tiempo restante de cada pedido.
Una vez seleccionado el objetivo, el algoritmo calcula un camino desde la posición actual del enemigo hasta el objetivo, ya sea para entregar un pedido o para recoger uno.
El sistema de búsqueda de caminos está implementado con el algoritmo A*. Este A* considera únicamente rutas transitables, por lo que evita edificios y selecciona el camino más rapido. Además, incorpora penalizaciones como los parques que reducen la velocidad, por lo que el algoritmo ajusta el costo de esos tiles para evitar que el enemigo los atraviese si hay alternativas más rápidas.

