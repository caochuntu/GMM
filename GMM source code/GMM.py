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


def _visualization_loc_route_network(mm_results, loc2road, rep_traj, data_list, pos_key, sg_x_min, sg_x_max, sg_y_min,
                                     sg_y_max, roadSegments):
    plt.figure()
    xx = []
    yy = []

    for i, datum in enumerate(data_list):
        lat0 = float(datum[1])
        lon0 = float(datum[0])
        xx.append(lon0)
        yy.append(lat0)
    plt.plot(xx, yy, 'p', color='gray', markersize=0.2, linewidth=0.2, markerfacecolor='white', markeredgecolor='red',
             markeredgewidth=0.1)

    for i, line in enumerate(roadSegments):
        road = line  # type: string
        if i < len(roadSegments) - 1:
            line = line.split(',')
            plt.plot([float(line[0]), float(line[2])], [float(line[1]), float(line[3])], color='silver', linestyle='-',
                     linewidth='0.1')

    for i in range(len(mm_results) - 1):
        node1 = mm_results[i]
        node1 = node1.split(',')
        node2 = mm_results[i + 1]
        node2 = node2.split(',')
        lat1 = float(node1[1])
        lon1 = float(node1[0])
        lat2 = float(node2[1])
        lon2 = float(node2[0])
        plt.plot([lon1, lon2], [lat1, lat2], color='black', linestyle='-', linewidth='0.1')

    repx = []
    repy = []
    for key in rep_traj[pos_key:pos_key + 2]:
        repx.append(key[0])
        repy.append(key[1])
        loc2raod_key = str(key[0]) + '_' + str(key[1])
        nearSeg = loc2road[loc2raod_key][0]
        nearSeg = nearSeg.split(',')
        plt.plot([float(nearSeg[0]), float(nearSeg[2])], [float(nearSeg[1]), float(nearSeg[3])], color='cyan',
                 linewidth='0.1')
    plt.plot(repx, repy, 'o', color='blue', markersize=0.2, linewidth=0.2, markerfacecolor='white',
             markeredgecolor='blue', markeredgewidth=0.1)

    plt.axis([sg_x_min, sg_x_max, sg_y_min, sg_y_max])
    plt.axis('off')
    file_name = 'your/target/directory' + num + '_' + rate + '_' + str(pos_key) + '.pdf'
    plt.savefig(file_name, bbox_inches='tight')

    plt.close()


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


def _gmm_main(data_list, rep_traj, pos_key, G, sg_x_min, sg_x_max, sg_y_min, sg_y_max, sgindex, roadSegments, visualize,
              loc2candidate, loc2road):
    for node1, node2, edgeData in G.edges(data=True):
        if edgeData['weightSum'] > 0 or edgeData['weight'] > 0:
            edgeData['weight'] = 0
            edgeData['weightSum'] = 0

    candidate_number = []
    for i, datum in enumerate(data_list):
        lat0 = float(datum[1])
        lon0 = float(datum[0])
        pointBBOX = (get_bounding_box_offset_radius(lon0, lat0, candidateSearchRange))
        matches = sgindex.intersect(pointBBOX)
        candidate_number.append(len(matches))

        if len(matches) < 1:
            print(datum)

        location_pair = [lon0, lat0]
        if location_pair in rep_traj:
            loc2candidate_key = str(lon0) + '_' + str(lat0)
            loc2candidate[loc2candidate_key] = matches
            loc2road[loc2candidate_key] = ['no_data', math.inf]

        for nearRoad in matches:
            points = nearRoad.split(',')
            lat1 = float(points[1])
            lon1 = float(points[0])
            lat2 = float(points[3])
            lon2 = float(points[2])
            dlon = lon1 - lon2
            dlat = lat1 - lat2
            theta = ((lon1 - lon2) * (lon1 - lon0) + (lat1 - lat2) * (lat1 - lat0)) / (dlon * dlon + dlat * dlat)
            if theta >= 0 and theta <= 1:
                lonc = lon1 + theta * (lon2 - lon1)
                latc = lat1 + theta * (lat2 - lat1)
                d_min = geodesic((lat0, lon0), (latc, lonc)).m
            else:
                d_end1 = geodesic((lat0, lon0), (lat1, lon1)).m
                d_end2 = geodesic((lat0, lon0), (lat2, lon2)).m
                d_min = min(d_end1, d_end2)
            node1name = points[0] + ',' + points[1]
            node2name = points[2] + ',' + points[3]

            prob = math.exp(-(pow((d_min / sd), 2)) / 2) / (math.sqrt(2 * 3.14) * sd)
            G[node1name][node2name]['weightSum'] += prob

            if location_pair in rep_traj:
                if d_min < loc2road[loc2candidate_key][1]:
                    loc2road[loc2candidate_key][1] = d_min
                    loc2road[loc2candidate_key][0] = nearRoad

    for node1, node2, edgeData in G.edges(data=True):
        if edgeData['weightSum'] > 0:
            edgeData['weight'] = edgeData['weightSum']
            edgeData['weightSum'] = 0

    beta = 0.8
    for i in range(50):
        for node1, node2, edgeData in G.edges(data=True):
            edgeData['new_weight'] = edgeData['weight'] * beta
            for neighbor in G[node1]:
                if neighbor != node2:
                    edgeData['new_weight'] += G[node1][neighbor]['weight'] * (1 - beta) / (
                                len(G[node1]) + len(G[node2]) - 2)
            for neighbor in G[node2]:
                if neighbor != node1:
                    edgeData['new_weight'] += G[node2][neighbor]['weight'] * (1 - beta) / (
                                len(G[node1]) + len(G[node2]) - 2)

        for node1, node2, edgeData in G.edges(data=True):
            edgeData['weight'] = edgeData['new_weight']
            edgeData['new_weight'] = 0

        wG = nx.Graph()
        for node1, node2, edgeData in G.edges(data=True):
            if edgeData['weight'] > 0:
                wG.add_edge(node1, node2, weight=edgeData['weight'])

        if len(sorted(nx.connected_components(wG))) == 1:
            break

    for node1, node2, edgeData in wG.edges(data=True):
        edgeData['weight'] = 1.0 / edgeData['weight']
        if edgeData['weight'] == 0:
            print('something is wrong here: edge with 0 weight')

    start_node, start_assist = _node_extract_given_loc_dist_first(rep_traj[pos_key], wG, loc2road)
    end_node, end_assist = _node_extract_given_loc_dist_first(rep_traj[pos_key + 1], wG, loc2road)

    if start_node == end_node:
        match_results = start_node, start_assist
    else:
        distance, match_results = _SPFA(start_node, end_node, wG)

    match_results = list(match_results)

    if visualize:
        _visualization_loc_route_network(match_results, loc2road, rep_traj, data_list, pos_key, sg_x_min, sg_x_max,
                                         sg_y_min, sg_y_max, roadSegments)

    return match_results


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


