/**
 * Container to display the network.
 * @type {HTMLElement}
 */
const container = $("#mynetwork")

container.on("contextmenu", function (event) {
    event.preventDefault()
})
/**
 *
 * @type {Network}
 */
let network = undefined

/**
 * Options to be used fo vis-network creation.
 */
const network_options = {
    manipulation: {
        enabled: true
    },
    physics: {
        enabled: true
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
    interaction: {
        dragNodes: false,
        dragView: false,
        zoomView: false,
    },
    nodes: {
        shape: 'box',
    },
}

/**
 * Sets
 * @param nodes {[my_vis_node]}
 * @param edges {}
 */
function setupNetwork(nodes, edges){
    network = new vis.Network(container[0], {nodes: nodes, edges: edges}, network_options);

    // adding event-listeners
    network.on("doubleClick", function (params){
        const node_name = network.body.nodes[params.nodes[0]].id
        switch (network.body.nodes[params.nodes[0]].options.group) {
            case "ros_pub":
                project.remove_pub(node_name)
                break
            case "ros_sub":
                project.remove_sub(node_name)
                break
            case "ros_topic":
                alert("Topics cannot be directly removed. Instead delete all nodes connected to it.")
        }
        package_to_network(project)
    })
    network.on("oncontext", function (params){
        if(!params.nodes[0])
            return
        const node_name = params.nodes[0]
        const node = project.get_node_by_node_name(node_name)
        if(new_node_dialog_prefill(node))
            open_new_node_dialog()
        else
            alert("Cannot be edited!")
    })
}

/**
 *
 * @param project {ros2_ui_project}
 * @returns {vis_network_data}
 */
function package_to_network(project){
    const net = new vis_network_data()
    parse_node_list(project.subs, "sub", net)
    parse_node_list(project.pubs, "pub", net)
    //console.log(net)
    setupNetwork(net.nodes, net.edges)
    display_package_details()
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