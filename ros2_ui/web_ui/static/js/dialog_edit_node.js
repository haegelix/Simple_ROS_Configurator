DBG = undefined

function init() {
    $("#new_node_type").on("input", new_node_type_changed)
    $('#new_node_form').on('submit', function (event) {
        event.preventDefault()
        if (createNode()) {
            close_new_node_dialog()
        }
        //$(this).trigger("reset")
    });
    $('#sub_user_code').val(
        "" +
        "# this code will only be used if you specify -user-supplied- as callback (see above)" + "\n" +
        "" + "\n" +
        "def init():" + "\n" +
        "    pass" + "\n" +
        "" + "\n" +
        "# this example will just print the data to stdout" + "\n" +
        "def callback(value):" + "\n" +
        "    print(value.data)" + "\n" +
        ""
    ).prop('disabled', true)
    $('#sub_callback').on("input", function (event) {
        if ($('#sub_callback').val() === "-user-supplied-") {
            $('#sub_user_code').prop('disabled', false)
        } else {
            $('#sub_user_code').prop('disabled', true)
        }
    })
}

/**
 *
 * @param node_type {string}
 * @param key {string}
 * @param value {any}
 * @returns {string}
 */
function setNodeValue(node_type, key, value) {
    const val = "" + value
    return $('#' + node_type + "_" + key).val(val)
}

/**
 *
 * @param node {sub_pub_node}
 * @return {boolean}
 */
function new_node_dialog_prefill(node) {
    let node_type = ""
    if(node instanceof pub){
        node_type = "pub"
        setNodeValue(node_type, "src", node.src)
    } else if (node instanceof sub) {
        node_type = "sub"
        setNodeValue(node_type, "callback", node.callback)
        setNodeValue(node_type, "user_code", node.user_code)
        $('#sub_callback').trigger("input")
    } else {
        console.error("Node type unknown.")
        return false
    }
    setNodeValue(node_type, "node_name", node.node_name)
    setNodeValue(node_type, "topic", node.topic)
    setNodeValue(node_type, "msg_type", node.msg_type)
    setNodeValue(node_type, "uses_stdout", "" + node.uses_stdout)
    setNodeValue(node_type, "needs_tty", "" + node.needs_tty)
    $('#new_node_type').val(node_type).trigger("input")
    return true
}

function open_new_node_dialog() {
    $('#new_node_dialog').get(0).showModal()
}

function close_new_node_dialog() {
    $('#new_node_dialog').get(0).close()
}

const types = ["pub", "sub"]

function new_node_type_changed() {
    const type = $('#new_node_type').val()
    for (let t of types) {
        $('#new_node_' + t).hide()
    }
    $('#new_node_' + type).show()
}

/**
 *
 * @param node_type {string}
 * @param key {string}
 * @returns {string}
 */
function getNodeValue(node_type, key) {
    return $('#' + node_type + "_" + key).val()
}

test = "djbf"
valid_node_name = RegExp("[a-z0-9_]{3,30}")
valid_topic = RegExp("[a-z0-9_]{3,30}")

function val_list_item(text) {
    const li = document.createElement("li")
    li.innerText = text
    return li
}

function createNode() {
    const node_type = $('#new_node_type').val()
    const node_name = getNodeValue(node_type, "node_name")
    const topic = getNodeValue(node_type, "topic")
    const msg_type = getNodeValue(node_type, "msg_type")
    const uses_stdout = getNodeValue(node_type, "uses_stdout") === "true"
    const needs_tty = getNodeValue(node_type, "needs_tty") === "true"

    // pub specific
    const src = getNodeValue("pub", "src")

    // sub specific
    const callback = getNodeValue("sub", "callback")
    const user_code = getNodeValue("sub", "user_code")

    const validation_list = $("#new_node_validation_errors_list")
    validation_list.empty()
    let valid = true
    if (!node_name.match(valid_node_name)) {
        validation_list.append(val_list_item("node_name shall only contain lowercase letters, digits and underscores"))
        validation_list.append(val_list_item("node_name shall not be longer than 30 or shorter than 3 characters"))
        valid = false
    }
    if (!topic.match(valid_topic)) {
        validation_list.append(val_list_item("topic shall only contain lowercase letters, digits and underscores"))
        validation_list.append(val_list_item("topic shall not be longer than 30 or shorter than 3 characters"))
        valid = false
    }
    if (msg_type === "invalid") {
        validation_list.append(val_list_item("choose a message type"))
        valid = false
    }
    if (node_type === "pub" && src === "invalid") {
        validation_list.append(val_list_item("choose a msg-source"))
        valid = false
    }
    if (node_type === "sub" && callback === "invalid") {
        validation_list.append(val_list_item("choose a callback"))
        valid = false
    }

    if (!valid)
        return false

    /**
     * Assemble an object to have all kinds of necessary attributes.
     */
    let obj = {node_name, topic, msg_type, uses_stdout, needs_tty, src, callback, user_code}
    if (node_type === "pub") {
        project.add_node(new pub(obj))
    } else if (node_type === "sub") {
        project.add_node(new sub(obj))
    }

    package_to_network(project)
    return true
}

init()
new_node_type_changed()