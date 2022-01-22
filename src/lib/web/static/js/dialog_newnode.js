function init(){
    $('#new_node_form').on('submit', function (event) {
        event.preventDefault()
        createNode()
        $(this).reset()
        close_new_node_dialog()
    });
}

function open_new_node_dialog() {
    $('#new_node_dialog').get(0).showModal()
}

function close_new_node_dialog() {
    $('#new_node_dialog').get(0).close()
}

function new_node_type_changed() {
    const type = $('#new_node_type').val()
    const types = ["pub", "sub"]
    for (let t of types) {
        $('#new_node_' + t).hide()
    }
    $('#new_node_' + type).show()
}

function createNode(){
    console.log("TODO create node")
}

init()
new_node_type_changed()