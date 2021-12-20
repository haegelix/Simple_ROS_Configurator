let network;
let container;
let exportArea;
let importButton;
let exportButton;

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
        }
    },
    height: '100%',
    width: '100%',
    groups: {
        ros_topic: { color: {background: 'red'}},
        ros_node: { color: {background: 'yellow'}}
    },
}

function init() {
    container = document.getElementById("mynetwork");
    exportArea = document.getElementById("input_output");
    importButton = document.getElementById("import_button");
    exportButton = document.getElementById("export_button");

    //draw();
    //exportNetwork();
    exportArea.value = "" +
        "[\n" +
        "  {\n" +
        "    \"id\": \"0\",\n" +
        "    \"connections\": [\n" +
        "      1,\n" +
        "      2\n" +
        "    ],\n" +
        "    \"group\": \"ros_topic\"\n" +
        "  },\n" +
        "  {\n" +
        "    \"id\": \"1\",\n" +
        "    \"connections\": [\n" +
        "      \"0\"\n" +
        "    ],\n" +
        "    \"group\": \"ros_node\"\n" +
        "  },\n" +
        "  {\n" +
        "    \"id\": \"2\",\n" +
        "    \"connections\": [\n" +
        "      \"0\"\n" +
        "    ],\n" +
        "    \"group\": \"ros_node\"\n" +
        "  }\n" +
        "]"
    importNetwork()
    //draw()
}

function addConnections(elem, index) {
// need to replace this with a tree of the network, then get child direct children of the element
    elem.connections = network.getConnectedNodes(index);
}

function destroyNetwork() {
    network.destroy();
}

function clearOutputArea() {
    exportArea.value = "";
}

function draw() {
// create a network of nodes
    //const data = getScaleFreeNetwork(5);

    network = new vis.Network(container, data, network_options);

    clearOutputArea();
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

function sendNetwork(){
    api.mySetter(exportArea.value)
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

function objectToArray(obj) {
    return Object.keys(obj).map(function (key) {
        obj[key].id = key;
        return obj[key];
    });
}

init();