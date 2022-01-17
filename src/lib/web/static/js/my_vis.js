const container = document.getElementById("mynetwork");
const exportArea = document.getElementById("input_output")

const network_options = {
    manipulation: {
        enabled: true
    },
    physics: {
        enabled: false
    },
    edges: {
        smooth: {
            enabled: false,
            type: 'discrete',
            forceDirection: 'horizontal',
            roundness: 0.8
        },
        arrows: {
            to: {
                enabled: true
            }
        }
    },
    height: '100%',
    width: '100%',
    groups: {
        ros_topic: { color: {background: 'red'}},
        ros_sub: { color: {background: 'yellow'}},
        ros_pub: { color: {background: 'green'}},
    },
}

function exportNetwork() {
    clearOutputArea();
    const nodes = objectToArray(network.getPositions());
    nodes.forEach(addConnections);

    // pretty print node data
    exportArea.value = JSON.stringify(nodes, undefined, 2);
}

function importNetwork() {
    const inputValue = exportArea.value;
    const inputData = JSON.parse(inputValue);

    const data = {
        nodes: getNodeData(inputData),
        edges: getEdgeData(inputData),
    };

    network = new vis.Network(container, data, network_options);
}

function getNodeData(data) {
    const networkNodes = [];

    data.forEach(function (elem) {
        networkNodes.push({
            id: elem.id,
            label: elem.id,
            x: elem.x,
            y: elem.y,
            group: elem.group,
        });
    });

    return new vis.DataSet(networkNodes);
}

function getNodeById(data, id) {
    for (let n = 0; n < data.length; n++) {
        if (data[n].id.toString() === id.toString()) {
            return data[n];
        }
    }
    console.log(data)
    console.log(id)
    throw "Can not find id '" + id + "' in data";
}

function getEdgeData(data) {
    const networkEdges = [];

    data.forEach(function (node) {
        // add the connection
        node.connections.forEach(function (connId) {
            networkEdges.push({from: node.id, to: connId});
            let cNode = getNodeById(data, connId);

            const elementConnections = cNode.connections;

            // remove the connection from the other node to prevent duplicate connections
            const duplicateIndex = elementConnections.findIndex(function (
                connection
            ) {
                return connection.toString() === node.id.toString();
            });

            if (duplicateIndex.toString() !== "-1") {
                elementConnections.splice(duplicateIndex, 1);
            }
        });
    });

    return new vis.DataSet(networkEdges);
}

function package_to_network(packageObj){
    let nodes = []
    parse_node_list(packageObj.subs, "sub", nodes)
    parse_node_list(packageObj.pubs, "pub", nodes)
    console.log(nodes)
    exportArea.value = JSON.stringify(nodes, null, 2)
    importNetwork()
}

/**
 *
 * @param topic_id
 * @param node_id
 * @param nodes
 */
function subscribe_topic(topic_id, node_id, nodes) {
    for(let e of nodes){
        if(e.id === topic_id) {
            e.connections.push(node_id)
            return
        }
    }
    // if topic not found in the list -> make a new one
    nodes.push(new my_vis_node(topic_id, [node_id], "ros_topic"))
}

/**
 *
 * @param topic_id
 * @param nodes
 */
function publish_topic(topic_id, nodes){
    for(let e of nodes){
        if(e.id === topic_id) {
            return
        }
    }
    // if topic not found in the list -> make a new one
    nodes.push(new my_vis_node(topic_id, [], "ros_topic"))
}

/**
 *
 * @param nodeList
 * @param type
 * @param nodes
 */
function parse_node_list(nodeList, type, nodes) {
    for (let e of nodeList) {
        parse_node(e, type, nodes)
    }
}

/**
 *
 * @param node
 * @param type
 * @param nodes
 */
function parse_node(node, type, nodes) {
    const group = "ros_" + type
    let connections = []
    if (type === "pub") {
        publish_topic(node.topic, nodes)
        connections.push(node.topic)
    } else if (type === "sub") {
        subscribe_topic(node.topic, node.node_name, nodes)
    }
    nodes.push(new my_vis_node(node.node_name, connections, group))
}

class my_vis_node {
    /**
     * id of the node
     * @type {string}
     */
    id
    /**
     * outgoing connections to other nodes
     * @type {[string]}
     */
    connections
    /**
     * group/type of this node
     * @type {string}
     */
    group

    constructor(id, connections, group) {
        this.id = id;
        this.connections = connections;
        this.group = group;
    }
}