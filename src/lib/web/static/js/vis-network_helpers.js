const container = document.getElementById("mynetwork");
const exportArea = document.getElementById("input_output")
let network = undefined

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
        },
        color: "black"
    },
    height: '100%',
    width: '100%',
    groups: {
        ros_topic: { color: {background: '#FFAA77', border: '#FFAA77'}},
        ros_sub: { color: {background: '#ADD8E6', border: '#ADD8E6'}},
        ros_pub: { color: {background: '#ADD8E6', border: '#ADD8E6'}},
    },
}

function setupNetwork(nodes, edges){
    network = new vis.Network(container, {nodes: nodes, edges: edges}, network_options);
}

/**
 *
 * @param packageObj
 * @returns {vis_network_data}
 */
function package_to_network(packageObj){
    const net = new vis_network_data()
    const pack = new srosc_pkg(packageObj)
    parse_node_list(pack.subs, "sub", net)
    parse_node_list(pack.pubs, "pub", net)
    console.log(net)
    setupNetwork(net.nodes, net.edges)
    return net
}

/**
 *
 * @param {[sub_pub_node]} nodeList
 * @param {string} type
 * @param {vis_network_data} net
 */
function parse_node_list(nodeList, type, net) {
    for (let e of nodeList) {
        parse_node(e, type, net)
    }
}

/**
 *
 * @param {sub_pub_node} node
 * @param {string} type
 * @param {vis_network_data} net
 */
function parse_node(node, type, net) {
    if (type === "pub") {
        net.add_edge(node.node_name, node.topic)
    } else if (type === "sub") {
        net.add_edge(node.topic, node.node_name)
    }
    net.add_node(node.node_name, type)
    net.add_node(node.topic, "topic")
}


class vis_network_data {
    /**
     * List of nodes in the network.
     * @type {[my_vis_node]}
     */
    nodes = []
    /**
     * List of edges in the network.
     * @type {[{from: string, to: string}]}
     */
    edges = []

    /**
     * Add a node to the network.
     * @param id
     * @param type
     */
    add_node(id, type) {
        for(let elem of this.nodes){
            if(elem.id === id){
                return
            }
        }
        this.nodes.push(new my_vis_node(id, type))
    }
    add_edge(from, to) {
        for (let elem of this.edges) {
            if(elem.from === from && elem.to === to){
                return
            }
        }
        this.edges.push({from: from, to: to})
    }
}

class my_vis_node {
    /**
     * id of the node
     * @type {string}
     */
    id
    /**
     * group/type of this node
     * @type {string}
     */
    group
    /**
     * label of this node
     * @type {string}
     */
    label

    constructor(id, type) {
        this.id = id;
        this.label = type + ": " + id;
        this.group = "ros_" + type
    }
}