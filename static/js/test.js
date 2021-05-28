
// These globals will be injected into a page that will use them.
/* eslint no-unused-vars: "off" */

// This is quite old and I don't want to waste too much time here. We probably
// should stop using this altogether as the examples should be easy and
// straightforward to understand and this only obscures it.
/* eslint require-jsdoc: "off" */

/* global Alea:false seededRandom:true */

/**
 * Created by Alex on 5/20/2015.
 *
 * @remarks
 * This depends on Alea from https://unpkg.com/alea@1.0.0/alea.js.
 */

/**
 * @param path
 * @param success
 * @param error
 */
function loadJSON(path, success, error) {
  const xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function () {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        success(JSON.parse(xhr.responseText));
      } else {
        error(xhr);
      }
    }
  };
  xhr.open("GET", path, true);
  xhr.send();
}

/**
 * @param nodeCount
 */
function getScaleFreeNetwork(nodeCount) {
  const nodes = [];
  const edges = [];
  const connectionCount = [];

  // randomly create some nodes and edges
  for (let i = 0; i < nodeCount; i++) {
    nodes.push({
      id: i,
      label: String(i),
    });

    connectionCount[i] = 0;

    // create edges in a scale-free-network way
    if (i == 1) {
      const from = i;
      const to = 0;
      edges.push({
        from: from,
        to: to,
      });
      connectionCount[from]++;
      connectionCount[to]++;
    } else if (i > 1) {
      const conn = edges.length * 2;
      const rand = Math.floor(seededRandom() * conn);
      let cum = 0;
      let j = 0;
      while (j < connectionCount.length && cum < rand) {
        cum += connectionCount[j];
        j++;
      }

      const from = i;
      const to = j;
      edges.push({
        from: from,
        to: to,
      });
      connectionCount[from]++;
      connectionCount[to]++;
    }
  }

  return { nodes: nodes, edges: edges };
}

const seededRandom = Alea("SEED");
console.log(seededRandom)

/**
 * @param nodeCount
 */
function getScaleFreeNetworkSeeded(nodeCount) {
  const nodes = [];
  const edges = [];
  const connectionCount = [];
  let edgesId = 0;

  // randomly create some nodes and edges
  for (let i = 0; i < nodeCount; i++) {
    nodes.push({
      id: i,
      label: String(i),
    });

    connectionCount[i] = 0;

    // create edges in a scale-free-network way
    if (i == 1) {
      const from = i;
      const to = 0;
      edges.push({
        id: edgesId++,
        from: from,
        to: to,
      });
      connectionCount[from]++;
      connectionCount[to]++;
    } else if (i > 1) {
      const conn = edges.length * 2;
      const rand = Math.floor(seededRandom() * conn);
      let cum = 0;
      let j = 0;
      while (j < connectionCount.length && cum < rand) {
        cum += connectionCount[j];
        j++;
      }

      const from = i;
      const to = j;
      edges.push({
        id: edgesId++,
        from: from,
        to: to,
      });
      connectionCount[from]++;
      connectionCount[to]++;
    }
  }

  return { nodes: nodes, edges: edges };
}


/*##################################################*/

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
            forceDetection: 'horizontal',
            roundness: 0.8
        }
    },
    height: '100%',
    width: '100%',
}

function init() {
    container = document.getElementById("mynetwork");
    exportArea = document.getElementById("input_output");
    importButton = document.getElementById("import_button");
    exportButton = document.getElementById("export_button");

    draw();
    exportNetwork();
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
    const data = getScaleFreeNetwork(5);

    network = new vis.Network(container, data, network_options);

    clearOutputArea();
}

function exportNetwork() {
    clearOutputArea();
    const nodes = objectToArray(network.getPositions());
    nodes.forEach(addConnections);

    // pretty print node data
    const exportValue = JSON.stringify(nodes, undefined, 2);
    exportArea.value = exportValue;
    resizeExportArea();
}

function importNetwork() {
    var inputValue = exportArea.value;
    var inputData = JSON.parse(inputValue);

    var data = {
        nodes: getNodeData(inputData),
        edges: getEdgeData(inputData),
    };

    network = new vis.Network(container, data, network_options);

    resizeExportArea();
}

function getNodeData(data) {
    var networkNodes = [];

    data.forEach(function (elem, index, array) {
        networkNodes.push({
            id: elem.id,
            label: elem.id,
            x: elem.x,
            y: elem.y,
        });
    });

    return new vis.DataSet(networkNodes);
}

function getNodeById(data, id) {
    for (var n = 0; n < data.length; n++) {
        if (data[n].id == id) {
            // double equals since id can be numeric or string
            return data[n];
        }
    }

    throw "Can not find id '" + id + "' in data";
}

function getEdgeData(data) {
    var networkEdges = [];

    data.forEach(function (node) {
        // add the connection
        node.connections.forEach(function (connId, cIndex, conns) {
            networkEdges.push({from: node.id, to: connId});
            let cNode = getNodeById(data, connId);

            var elementConnections = cNode.connections;

            // remove the connection from the other node to prevent duplicate connections
            var duplicateIndex = elementConnections.findIndex(function (
                connection
            ) {
                return connection == node.id; // double equals since id can be numeric or string
            });

            if (duplicateIndex != -1) {
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

function resizeExportArea() {
    exportArea.style.height = 1 + exportArea.scrollHeight + "px";
}

init();