document.addEventListener("DOMContentLoaded", setup)

let socket = io("/package");

let pubs = 0
let subs = 0

/**
 * Setup EventHandlers for this website.
 */
function setup() {
    document.getElementById("pub_add").addEventListener("click", pub_add)
    document.getElementById("sub_add").addEventListener("click", sub_add)
    document.getElementById("newpackage").onsubmit = submit

    socket.on('connect', function() {
        console.log("WebSocket connected!")
    });

    socket.on('newpackage', function(message) {
        console.log("Answer for newpackage.")
        console.log(message)
    });
}

/**
 * Formats the entered data into an object.
 * @returns {{pubs: *[], additional_imports: *[], subs: *[], package_name, package_info: {license, test_depends: string[], maintainer_mail, exec_depends: string[], description, version, maintainer}}}
 */
function format_package() {
    return {
        package_name: document.getElementById("package_name").value,
        package_info: {
            version: document.getElementById("version").value,
            license: document.getElementById("license").value,
            description: document.getElementById("description").value,
            maintainer: document.getElementById("maintainer_name").value,
            maintainer_mail: document.getElementById("maintainer_email").value,
            test_depends: [
                "ament_copyright",
                "ament_flake8",
                "ament_pep257",
                "python3-pytest"
            ],
            exec_depends: [
                "rclpy",
                "std_msgs"
            ]
        },
        subs: get_subs(),
        pubs: get_pubs(),
        additional_imports: []
    }
}

/**
 * will be called on submit.
 * @returns {boolean} false to prevent further handling.
 */
function submit() {
    socket.emit('newpackage', {data: format_package(), filename: document.getElementById("filename").value})
    return false
}

function get_field_val(fsid, name){
    return document.getElementById(fsid + "_" + name).value
}


/**
 * Get all pubs.
 * @returns {*[]} List of pubs.
 */
function get_pubs(){
    pubs = []
    for (let elem of document.getElementById("pubs").getElementsByTagName("fieldset")) {
        pubs.push({
            node_name: get_field_val(elem.id, "node_name"),
            topic: get_field_val(elem.id, "topic"),
            type: get_field_val(elem.id, "type"),
            src: get_field_val(elem.id, "src")
        })
    }
    return pubs
}

/**
 * Get all subs.
 * @returns {*[]} List of subs.
 */
function get_subs(){
    subs = []
    for (let elem of document.getElementById("subs").getElementsByTagName("fieldset")) {
        subs.push({
            node_name: get_field_val(elem.id, "node_name"),
            topic: get_field_val(elem.id, "topic"),
            type: get_field_val(elem.id, "type"),
            callback: get_field_val(elem.id, "callback")
        })
    }
    return subs
}

/**
 * Add a publisher.
 */
function pub_add() {
    const id = "pub_" + (pubs + 1)

    let fs = document.createElement("fieldset")
    fs.id = id

    let legend = document.createElement("legend")
    legend.innerText = id
    fs.appendChild(legend)
    fs.appendChild(delete_button_factory(id))

    fs.appendChild(labeled_input_factory(id, "node_name", "Node name"))
    fs.appendChild(labeled_input_factory(id, "topic", "Topic"))
    fs.appendChild(labeled_input_factory(id, "type", "Type"))
    fs.appendChild(labeled_input_factory(id, "src", "Source"))

    let p_templ = document.createElement("p")
    let label_templates = document.createElement("label")
    label_templates.innerText = "Templates"
    let p_links = document.createElement("p")
    p_links.appendChild(get_pub_template(id, "stdin", "std_msgs.msg.String", "-stdin-"))
    p_links.appendChild(get_span(", "))
    p_links.appendChild(get_pub_template(id, "button", "std_msgs.msg.String", "-button-"))
    p_templ.appendChild(label_templates)
    p_templ.appendChild(p_links)
    fs.appendChild(p_templ)

    document.getElementById("pubs").appendChild(fs)
    pubs += 1
}

function get_pub_template(fsid, name, type, src){
    let templ = document.createElement("a")
    templ.href = "#" + fsid
    templ.onclick = function () {
        document.getElementById(fsid + "_type").value = type
        document.getElementById(fsid + "_src").value = src
    }
    templ.innerText = name
    return templ
}

function get_sub_template(fsid, name, type, callback){
    let templ = document.createElement("a")
    templ.href = "#" + fsid
    templ.onclick = function () {
        document.getElementById(fsid + "_type").value = type
        document.getElementById(fsid + "_callback").value = callback
    }
    templ.innerText = name
    return templ
}

function get_span(text){
    const s = document.createElement("span")
    s.innerText = text
    return s
}

/**
 * Add a subscriber.
 */
function sub_add() {
    const id = "sub_" + (subs + 1)
    let fs = document.createElement("fieldset")
    fs.id = id

    let legend = document.createElement("legend")
    legend.innerText = id
    fs.appendChild(legend)
    fs.appendChild(delete_button_factory(id))

    fs.appendChild(labeled_input_factory(id, "node_name", "Node name"))
    fs.appendChild(labeled_input_factory(id, "topic", "Topic"))
    fs.appendChild(labeled_input_factory(id, "type", "Type"))
    fs.appendChild(labeled_input_factory(id, "callback", "Callback"))

    let p_templ = document.createElement("p")
    let label_templates = document.createElement("label")
    label_templates.innerText = "Templates"
    let p_links = document.createElement("p")
    p_links.appendChild(get_sub_template(id, "stdout", "std_msgs.msg.String", "-stdout-"))
    p_links.appendChild(get_span(", "))
    p_links.appendChild(get_sub_template(id, "led", "std_msgs.msg.String", "-led-"))
    p_templ.appendChild(label_templates)
    p_templ.appendChild(p_links)
    fs.appendChild(p_templ)

    document.getElementById("subs").appendChild(fs)
    subs += 1
}

/**
 * Construct a delete button.
 * @param delete_id ID of the element to be deleted on click.
 * @returns {HTMLParagraphElement} Built DOM element.
 */
function delete_button_factory(delete_id) {
    const input = document.createElement("input")
    input.type = "button"
    input.value = "-"

    input.addEventListener("click", function () {
        document.getElementById(delete_id).remove()
    })

    return input
}

/**
 * Construct a delete button.
 * @param id ID of the surrounding fieldset.
 * @param name Name of this field.
 * @param labeltext Text for the label.
 * @returns {HTMLParagraphElement} Built DOM element.
 */
function labeled_input_factory(id, name, labeltext) {
    const idname = id + "_" + name
    const p = document.createElement("p")

    const label = document.createElement("label")
    label.htmlFor = idname
    label.innerText = labeltext
    p.appendChild(label)

    const input = document.createElement("input")
    input.type = "text"
    input.id = idname
    input.name = name
    input.setAttribute("required", true)
    p.appendChild(input)

    return p
}