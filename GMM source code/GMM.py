threshold = 0.001
candidateSearchRange = 100
sd = 5


def _boundary(sg_map):
    roadSegments = open(sg_map).read().split('\n')
    sg_x_min = 200
    sg_x_max = -math.inf
    sg_y_min = 100
    sg_y_max = -math.inf
    for i, seg in enumerate(roadSegments):
        if i < len(roadSegments) - 1:
            seg = seg.split(',')
            sg_x_max = max(sg_x_max, float(seg[0]))
            sg_x_min = min(sg_x_min, float(seg[0]))
            sg_x_max = max(sg_x_max, float(seg[2]))
            sg_x_min = min(sg_x_min, float(seg[2]))
            sg_y_max = max(sg_y_max, float(seg[1]))
            sg_y_min = min(sg_y_min, float(seg[1]))
            sg_y_max = max(sg_y_max, float(seg[3]))
            sg_y_min = min(sg_y_min, float(seg[3]))
    return sg_x_min, sg_y_min, sg_x_max, sg_y_max


def get_bounding_box_offset_radius(lon, lat, radius):
    R = 6378137
    dn = radius + 50
    de = radius + 50
    dLat = abs(float(dn) / R)
    dLon = abs(de / (R * math.cos(math.pi * lat / 180)))
    temp_xmax = lon + dLon * 180 / math.pi
    temp_xmin = lon - dLon * 180 / math.pi
    temp_ymax = lat + dLat * 180 / math.pi
    temp_ymin = lat - dLat * 180 / math.pi
    return temp_xmin, temp_ymin, temp_xmax, temp_ymax


def _A_star(start, end, graph):
    matched_route = nx.astar_path(graph, start, end, weight='weight')
    distance = nx.astar_path_length(graph, start, end, weight='weight')
    return distance, matched_route


def _SPFA(start, end, graph):
    dist = {}
    inQueue = {}
    predecessor = {}
    for node in list(graph.nodes):
        dist[node] = math.inf
        inQueue[node] = False
    if start not in dist:
        print('---> [Error] start node is not included in the graph <---')
    elif end not in dist:
        print('---> [Error] end node is not included in the graph <---')
    else:
        dist[start] = 0

    q = deque()
    q.append(start)
    inQueue[start] = True

    while (len(q) > 0):
        u = q.popleft()
        inQueue[u] = False
        for v in graph[u]:
            if dist[v] > dist[u] + graph[u][v]['weight']:
                dist[v] = dist[u] + graph[u][v]['weight']
                predecessor[v] = u
                if not inQueue[v]:
                    q.append(v)
                    inQueue[v] = True
    trace = []
    trace.append(end)
    end_v = end
    for _ in range(len(predecessor)):
        if predecessor[end_v] == start:
            trace.append(start)
            break
        else:
            pred = predecessor[end_v]
            trace.append(pred)
            end_v = pred

    trace.reverse()
    return dist[end], trace


def _node_extract_given_loc_dist_first(loc, graph, loc2road):
    key = str(loc[0]) + '_' + str(loc[1])
    edge = loc2road[key][0]
    points = edge.split(',')
    node1name = points[0] + ',' + points[1]
    node2name = points[2] + ',' + points[3]
    if not node1name in graph:
        print('something is wrong: the node {0} is not included by grpah'.format(node1name))
    else:
        return node1name, node2name
    if not node2name in graph:
        print('something is wrong: the node {0} is not included by grpah'.format(node2name))


def _road_graph_construction(sg_x_min, sg_y_min, sg_x_max, sg_y_max, sg_map):
    G = nx.Graph()
    sgBBOX = (sg_x_min, sg_y_min, sg_x_max, sg_y_max)
    sgindex = Index(sgBBOX, None, None, None, None, 10, 20)
    roadSegments = open(sg_map).read().split('\n')
    for i, line in enumerate(roadSegments):
        if i < len(roadSegments) - 1:
            line = line.split(',')
            itemName = line[0] + ',' + line[1] + ',' + line[2] + ',' + line[3]
            x_min = min(float(line[0]), float(line[2]))
            x_max = max(float(line[0]), float(line[2]))
            y_min = min(float(line[1]), float(line[3]))
            y_max = max(float(line[1]), float(line[3]))
            itemBBOX = (x_min, y_min, x_max, y_max)
            sgindex.insert(itemName, itemBBOX)

            node1name = line[0] + ',' + line[1]
            node2name = line[2] + ',' + line[3]
            G.add_node(node1name)
            G.add_node(node2name)
            G.add_edge(node1name, node2name, count=0, weightSum=0, weight=0, new_weight=0)
    return G, sgindex


def _mm_data_format(file_dir, rate):
    loc_list = []
    data = open(file_dir).read().split('\n')
    for i, datum in enumerate(data):
        if i < len(data) - 1:
            if rate == '1':
                datum = datum.split('\t')  # for raw data
            else:
                datum = datum.split(',')  # for sampled data
            lat = float(datum[1])
            lon = float(datum[0])
            location = [lon, lat]
            loc_list.append(location)
    return loc_list


def gmm(num, rate):
    loc2candidate = {}  # link location to all road candidates
    loc2road = {}  # link location to the nearest road segment

    sg_map = 'your/map/directory' + num + '.map'
    file_dir = 'your/trajectory/directory' + num + '_' + rate + '.track'
    result_dir = 'your/matchedRoute/directory' + num + '_' + rate + 'gmm_a_newMap.route'

    sg_x_min, sg_y_min, sg_x_max, sg_y_max = _boundary(sg_map)
    G, sgindex = _road_graph_construction(sg_x_min, sg_y_min, sg_x_max, sg_y_max, sg_map)
    loc_list = _mm_data_format(file_dir, rate)
    rep_traj = rdp(loc_list, epsilon=threshold)
    roadSegments = open(sg_map).read().split('\n')
    r_route = open(result_dir, "a")

    for i in range(len(rep_traj) - 1):
        partial_data = loc_list[loc_list.index(rep_traj[i]):loc_list.index(rep_traj[i + 1]) + 1]
        route = _gmm_main(partial_data, rep_traj, i, G, sg_x_min, sg_x_max, sg_y_min, sg_y_max, sgindex, roadSegments,
                          False, loc2candidate, loc2road)  # no PDF
        for i in range(len(route) - 1):
            roadSegment = route[i] + ',' + route[i + 1] + '\n'
            r_route.write(roadSegment)
    r_route.close()


